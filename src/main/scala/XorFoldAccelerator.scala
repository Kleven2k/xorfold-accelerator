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
   * This implementation currently assumes RV64:
   *  
   *  - memory operations are 8 bytes
   *  - checksum streaming processes one 64-bit beat at a time
   *  - each beat contains four RFC 1071 16-bit words
   */
  require(xLen == 64, "XorFoldAccelerator currently requires RV64")

  /*
   * ------------------------------------------------------------
   * Accumulator
   * ------------------------------------------------------------
   */

  /* 
   * Existing XOR-fold accumulator.
   */
  val accum = RegInit(0.U(xLen.W))

  /* 
   * Independent one's-complement checksum accumulator.
   *  
   * Keeping this seperate from accum prevents XOR operations and
   * RFC checksum operations from implicitly changing each other's
   * state.
   */
  val checksumAccum = RegInit(0.U(xLen.W))

  /*
   * ------------------------------------------------------------
   * Memory-operation registers
   * ------------------------------------------------------------
   */

  /*
   * Address of the next memory operation.
   */ 
  val ptr = RegInit(0.U(xLen.W))
  
  /* 
   * Number of memory requests that have not yet been issued.
   *  
   * For streaming reads this is decremented when mem.req.fire
   * occurs, not when the response arrives.
   */
  val remaining = RegInit(0.U(xLen.W))

  // Preserve privilege information for autonomous memory accesses.
  val memDprv = RegInit(0.U(2.W))
  val memDv   = RegInit(false.B)

  // Store payload used by funct=5.
  val memWriteData = RegInit(0.U(xLen.W))

  /* 
   * ------------------------------------------------------------
   * Memory operation type
   * ------------------------------------------------------------
   *  
   * memXorFold:
   *  Load data and XOR it into accum.
   *  
   * memChecksum:
   *  Load data and one's-complement-add it into checksumAccum.
   *  
   * memWriteResult: 
   *  Store memWriteData to memory.
   */

  val memXorFold :: memChecksum :: memWriteResult :: Nil = Enum(3)
  val memOp = RegInit(memXorFold)

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

  val doReset            = funct === 0.U
  val doFold             = funct === 1.U
  // funct=2 is the ordinary accum read via the generic response path.
  val doFoldMem          = funct === 3.U
  val doFoldMemN         = funct === 4.U
  val doWriteResult      = funct === 5.U 

  val doChecksumReset    = funct === 6.U 
  val doChecksumAddN     = funct === 7.U 
  val doChecksumFinalize = funct === 8.U 

  /* 
   * ------------------------------------------------------------
   * Helper: start a memory transaction
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
   * Helper: one's-complement addition
   * ------------------------------------------------------------
   *  
   * Add two 64-bit values while preserving the carry-out bit.
   *  
   * +& produces a 65-bit result:
   *  
   *    [64]   = carry
   *    [63:0] = low result
   *  
   * The carry is then wrapped back into bit 0
   *  
   * A second carry fold is not required here. The maximum sum of 
   * two 64-bit inputs is:
   * 
   *    (2^64 - 1) + (2^64 - 1) = 2^65 - 2
   *  
   * so the impossible problematic pattern 1_FFFF...FFFF can never
   * result from one two-operand addition.   
   */

  def onesComplementAdd64(a: UInt, b: UInt): UInt = {
    val wideSum = a +& b 

    val folded =
      wideSum(63, 0) +& wideSum(64)

    folded(63, 0)
  }

  /* 
   * ------------------------------------------------------------
   * Helper: convert a little-endian RV64 load into four
   * network-order 16-bit checksum words.
   * ------------------------------------------------------------
   *  
   * Example bytes in increasing memory addresses:
   * 
   *  00 01  00 02  00 03  00 04
   * 
   * RV64 load value: 
   *
   *  0x0400030002000100
   * 
   * RFC 1071 needs the 16-bit values:
   *
   *  0x0001
   *  0x0002
   *  0x0003
   *  0x0004
   * 
   * Therefore swap the two bytes within every 16-bit lane.
   */

  def networkOrder16Lanes(d: UInt): UInt = {
    Cat(
      d(55, 48), d(63, 56),
      d(39, 32), d(47, 40),
      d(23, 16), d(31, 24),
      d(7, 0),   d(15, 8)
    )
  }

  /* 
   * ------------------------------------------------------------
   * RFC 1071 finalization
   * ------------------------------------------------------------
   * 
   * checksumAccum contains four 16-bit lanes:
   * 
   *   [63:48]
   *   [47:32]
   *   [31:16]
   *   [15:0]
   * 
   * Add all four lanes, fold any carry back into the low 16 bits,
   * then take the one's complement.
   */

  val checksumChunk0 = checksumAccum(15, 0)
  val checksumChunk1 = checksumAccum(31, 16)
  val checksumChunk2 = checksumAccum(47, 32)
  val checksumChunk3 = checksumAccum(63, 48)

  /* 
   * Four maximum 16-bit values sum to:
   * 
   *   4 * 0xffff = 0x3fffc
   * 
   * which fits in 18 bits.
   */
  val checksumChunkSum =
    checksumChunk0.pad(18) +
    checksumChunk1.pad(18) +
    checksumChunk2.pad(18) +
    checksumChunk3.pad(18)

  /* 
   * First end-around fold:
   *
   * upper two carry bits are added into the low 16 bits.
   */
  val checksumFold1 =
    checksumChunkSum(15, 0).pad(17) +
    checksumChunkSum(17, 16).pad(17)

  /* 
   * Second fold handles a possible carry from checksumFold1.
   */
  val checksumFold2 =
    checksumFold1(15, 0) +&
    checksumFold1(16)

  /* 
   * RFC 1071 final checksum.
   */
  val checksumFinal =
    ~checksumFold2(15, 0)

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
   * XOR accumulator reset.
   */
  when (cmd.fire && doReset) {
    accum := 0.U
  }

  /*  
   * funct = 1
   *  
   * Existing register/register XOR fold:
   * 
   *   fold(rs1, rs2)
   * 
   *   accum ^= rs1 + rs2
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
   * One-word XOR memory fold.
   */
  when (cmd.fire && doFoldMem) {
    startMemOp(
      addr      = cmd.bits.rs1,
      count     = 1.U,
      op        = memXorFold,
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
        op        = memXorFold,
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
   * Store the existing XOR accumulator to memory.
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
   * funct = 6
   * 
   * checksum_reset()
   * 
   * Reset only the checksum accumulator.
   * 
   * The existing XOR accum remains untouched.
   */

  when (cmd.fire && doChecksumReset) {
    checksumAccum := 0.U
  }

  /* 
   * funct = 7
   * 
   * checksum_add_n(ptr, n)
   * 
   * rs1 = starting buffer address
   * rs2 = number of 8-byte memory beats
   * 
   * This does NOT reset checksumAccum first. Multiple funct=7
   * operations may therefore be chained after one funct=6 reset.
   * 
   * n == 0 is a no-op.
   */

  when (cmd.fire && doChecksumAddN) {
    when (cmd.bits.rs2 =/= 0.U) {
      startMemOp(
        addr      = cmd.bits.rs1,
        count     = cmd.bits.rs2,
        op        = memChecksum,
        writeData = 0.U,
        dprv      = cmd.bits.status.dprv,
        dv        = cmd.bits.status.dv 
      )
    }
  }

  /* 
   * funct = 8
   * 
   * checksum_finalize()
   * 
   * No state-changing when-block is required.
   * 
   * The generic response path below returns checksumFinal for this
   * funct when xd is set.
   * 
   * checksumAccum itself is not modified.
   */

  /*
   * ------------------------------------------------------------
   * CPU response
   * ------------------------------------------------------------
   *
   * funct=2:
   *   return accum
   * 
   * funct=8:
   *   return the finalized 16-bit Internet checksum, zero-extended
   *   ti xLen.
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
    Mux(
      doChecksumFinalize,
      checksumFinal.pad(xLen),
      accum
    )

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
   * 
   * Both XOR folding and checksum accumulation perform loads.
   * write_result performs a store.
   */
  io.mem.req.bits.cmd := 
    Mux(
      memOp === memWriteResult, 
      M_XWR, 
      M_XRD
    )

  /* 
   * 64-bit / 8-byte accesses on RV64.
   * 
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
  
  io.mem.req.bits.mask := 
    Fill(xLen / 8, 1.U(1.W))

  io.mem.req.bits.phys    := false.B 
  io.mem.req.bits.dprv    := memDprv
  io.mem.req.bits.dv      := memDv 
  io.mem.req.bits.no_resp := false.B 

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
       * Both XOR streaming and checksum streaming walk forward by
       * one 64-bit memory beat.
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
       * Neither accum nor checksumAccum changes.
       */
      state := sIdle 
    } .otherwise {

      /* 
       * ------------------------------------------------------------
       * Read-response datapath
       * ------------------------------------------------------------
       */

      when (memOp === memXorFold) {

        /* 
         * Existing XOR behavior.
         */
        accum :=
          accum ^ io.mem.resp.bits.data 
      } .elsewhen (memOp === memChecksum) {

        /* 
         * Interpret each 16-bit lane in network byte order before
         * performing one's-complement addition.
         */
        val checksumWord =
          networkOrder16Lanes(io.mem.resp.bits.data)

        checksumAccum :=
          onesComplementAdd64(
            checksumAccum,
            checksumWord
          )
      }


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
