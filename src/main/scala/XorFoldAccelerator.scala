package xorfold

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config._

import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.rocket._

class XorFoldAccelerator(opcodes: OpcodeSet)(implicit p: Parameters) 
  extends LazyRoCC(opcodes) {

  override lazy val module = 
    new XorFoldAcceleratorModuleImp(this)
}

class XorFoldAcceleratorModuleImp(
  outer: XorFoldAccelerator
)(implicit p: Parameters) 
    extends LazyRoCCModuleImp(outer)
    with HasCoreParameters {

  /*
   * ------------------------------------------------------------
   * Accumulator
   * ------------------------------------------------------------
   */

  val accum = RegInit(0.U(xLen.W))

  /*
   * ------------------------------------------------------------
   * Memory-operation registers
   * ------------------------------------------------------------
   *
   * These registers preserve all information needed after the
   * original RoCC command has left the command queue..
   */ 
  val ptr = RegInit(0.U(xLen.W))
  
  /* 
   * For straming reads:
   * 
   * Number of memory requests that have not yet been issued.
   *  
   * funct=3 starts with remaining = 1
   * funct=4 starts with remaining = n
   * funct=5 used count = 1, although it is currently a single write.
   */
  val remaining = RegInit(0.U(xLen.W))

  // Preserve privilege information for autonomous memery accesses.
  val memDprv = RegInit(0.U(2.W))
  val memDv   = RegInit(false.B)

  // Store payload used by funct=5.
  val memWriteData = RegInit(0.U(xLen.W))

  /* 
   * ------------------------------------------------------------
   * Memory operation type
   * ------------------------------------------------------------
   *  
   * This replaces memIsWrite.
   *  
   * memFold:
   *  Memory response contains data that should be XORed into accum.
   *  
   * memWriteResult: 
   *  Memory operation stores memWriteData. Response only signals
   *  completion; accum is not modified.
   */

  val memFold :: memWriteResult :: Nil = Enum(2)
  val memOp = RegInit(memFold)

  /* ------------------------------------------------------------
   * Main FSM
   * ------------------------------------------------------------
   * 
   * sIdle:
   *  Waiting for a RoCC command.
   * 
   * sMemReq:
   *  Present memory request until accepted.
   *  
   * sMemResp: 
   *  Wait for completion/response. 
   */
  val sIdle :: sMemReq :: sMemResp :: Nil = Enum(3)
  val state = RegInit(sIdle)

  /*
   * ------------------------------------------------------------
   * RoCC command queue / decode
   * ------------------------------------------------------------
   */

  val cmd = Queue(io.cmd)

  val funct = cmd.bits.inst.funct

  val doReset       = funct === 0.U
  val doFold        = funct === 1.U
  val doFoldMem     = funct === 3.U
  val doFoldMemN    = funct === 4.U
  val doWriteResult = funct === 5.U 

  /* 
   * ------------------------------------------------------------
   * Shared memory-operation starter
   * ------------------------------------------------------------
   *  
   * Every operation that enters sMemReq should go through this
   * helper.
   *  
   * That gives us one place where all persistent transaction
   * state is initialized.
   */

  def startMemOp(
      addr: UInt,
      count: UInt,
      op: UInt,
      writeData: UInt,
      dprv: UInt,
      dv: Bool
  ): Unit = {
    ptr          := addr
    remaining    := count 
    memOp        := op
    memWriteData := writeData
    memDprv      := dprv 
    memDv        := dv 

    state := sMemReq  
  }

  /*
   * ------------------------------------------------------------
   * RoCC command handling
   * ------------------------------------------------------------
   */

  val idle = state === sIdle 

  /* 
   * xd means the instruction expects a response through io.resp.
   *  
   * If the response interface cannot accept it, don't consume
   * the command yet.
   */
  val doResp = cmd.bits.inst.xd 

  val stallResp = doResp && !io.resp.ready 

  /* 
   * While the memory FSM is active, no subsequent RoCC command
   * may execute.
   *  
   * This is what serializes:
   * 
   *    fold_mem_n(...)
   *    xorfold_read()
   *  
   * The read cannot fire until the final memory response has been
   * processed and state returns to sIdle.
   */
  cmd.ready := idle && !stallResp

  /*
   * funct = 0
   * 
   * Reset accumulator.
   */
  when (cmd.fire && doReset) {
    accum := 0.U
  }

  /*  
   * funct = 1
   *  
   * fold(rs1, rs2)
   * 
   * accum ^= rs1 + rs2
   */
  when (cmd.fire && doFold) {
    accum := 
      accum ^ (cmd.bits.rs1 + cmd.bits.rs2)
  }

  /* 
   * funct = 3
   *  
   * fold_mem(ptr)
   *  
   * One-word memory fold.
   *  
   * One-word read stream.
   */
  when (cmd.fire && doFoldMem) {
    startMemOp(
      addr      = cmd.bits.rs1,
      count     = 1.U,
      op        = memFold,
      writeData = 0.U,
      dprv      = cmd.bits.status.dprv,
      dv        = cmd.bits.status.dv
    ) 
  }

  /* 
   * funct = 4
   *  
   * fold_mem_n(ptr, n)
   *  
   * rs1 = starting pointer (address)
   * rs2 = word count (number of xLen-sized words)
   *  
   * n == 0 is a no-op.
   */
  when (cmd.fire && doFoldMemN) {
    when (cmd.bits.rs2 =/= 0.U) {
      startMemOp(
        addr      = cmd.bits.rs1,
        count     = cmd.bits.rs2,
        op        = memFold,
        writeData = 0.U,
        dprv      = cmd.bits.status.dprv,
        dv        = cmd.bits.status.dv   
      ) 
    }
  }

  /* 
   * funct = 5
   *  
   * write_result(dst_ptr)
   *  
   * rs1 = destination address
   *  
   * Capture the accumulator at command acceptance so the complete
   * store transaction is independent of cmd.bits afterward.
   */
  when (cmd.fire && doWriteResult) {
    startMemOp(
      addr      = cmd.bits.rs1,
      count     = 1.U,
      op        = memWriteResult,
      writeData = accum,
      dprv      = cmd.bits.status.dprv,
      dv        = cmd.bits.status.dv   
    )
  }

  /*
   * ------------------------------------------------------------
   * CPU response
   * ------------------------------------------------------------
   *
   * funct = 2
   *
   * read()
   *
   * There is no dedicated doRead/when block: funct=2 sets xd (via
   * the ROCC_INSTRUCTION_D macro) and carries no other behavior, so
   * it's already fully handled by this generic response path, which
   * returns accum to any instruction that sets xd.
   */

  io.resp.valid :=
    cmd.valid && 
    doResp && 
    idle  

  io.resp.bits.rd := 
    cmd.bits.inst.rd
  
  io.resp.bits.data := 
    accum

  /*
   * ------------------------------------------------------------
   * Memory request
   * ------------------------------------------------------------
   * 
   * Once the FSM enters sMemReq, the requesst no longer depends
   * on cmd.bits.
   */

  io.mem.req.valid := 
    state === sMemReq 

  io.mem.req.bits.addr := 
    ptr
  
  io.mem.req.bits.tag := 
    0.U 

  /* 
   * Memory command depends on the latched memory operation type.
   * 
   * Loads for funct=3/4.
   * Store for funct=5.
   */
  io.mem.req.bits.cmd := 
    Mux(
      memOp === memWriteResult, 
      M_XWR, 
      M_XRD
    )

  /* 
   * xLen = 64:
   * 
   *  xLen / 8 = 8 bytes
   *  log2Ceil(8) = 3
   *  
   * so this requests an 8-byte memory operation.
   */
  io.mem.req.bits.size := 
    log2Ceil(xLen / 8).U 
  
  io.mem.req.bits.signed := 
    false.B 
  
  /* 
   * Read data field is unused.
   *  
   * For funct=5, this is the value written to memory.
   */
  io.mem.req.bits.data :=
    Mux(
      memOp === memWriteResult, 
      memWriteData, 
      0.U
    )

  io.mem.req.bits.phys := 
    false.B 
  
  io.mem.req.bits.dprv := 
    memDprv
  
  io.mem.req.bits.dv := 
    memDv 
  
  io.mem.req.bits.no_resp := 
    false.B 

  /* 
   * ------------------------------------------------------------
   * Memory request accepted
   * ------------------------------------------------------------
   */

  when (state === sMemReq && io.mem.req.fire) {

    when (memOp === memWriteResult) {
      /* 
       * funct=5 currently performs exactly one store.
       *  
       * We therefore simply wait for its completion response.
       *  
       * If a future funct=6 implements write_result_n, this is
       * where the write-side pointer/count logic would need to
       * become symmetric with the read-side logic below.
       */
      state := sMemResp

    } .otherwise {
      /* 
       * Streaming read.
       *  
       * Only advance ptr when the cache actually accepts the
       * request. 
       */
      ptr :=
        ptr + (xLen / 8).U
      
      /* 
       * remaining means:
       * 
       *  number of requests not yet issued
       *  
       * Therefore decrement when re.fire happens, not when the
       * response arrives.
       */
      remaining :=
        remaining - 1.U 

      state := 
        sMemResp
    }
  }

  /*
   * ------------------------------------------------------------
   * Memory response / completion
   * ------------------------------------------------------------
   */

  when (state === sMemResp && io.mem.resp.valid) {

    when (memOp === memWriteResult) {

      /* 
       * Store completed.
       *  
       * The response data field is not a value to fold.
       * accum remains unchanged.
       */
      state := sIdle 
    } .otherwise {
      /* 
       * Read completed.
       *  
       * Fold loaded data into the accumulator.
       */
      accum :=
        accum ^ io.mem.resp.bits.data 

      /* 
       * remaining was already decremented when the corresponding 
       * request fired.
       *  
       * Therefore:
       *  
       *  remaining == 0
       *  
       * means the response that just arrived belongs to the final
       * load in the stream.
       */
      when (remaining === 0.U) {
        state := sIdle 
      } .otherwise {
        state := sMemReq
      }
    }
  }

  /*
   * ------------------------------------------------------------
   * Accelerator status
   * ------------------------------------------------------------
   */

  io.busy :=
    cmd.valid ||
    state =/= sIdle 

  io.interrupt := false.B 

}
