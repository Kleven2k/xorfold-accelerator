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
   *   - memory operations are 8 bytes
   *   - checksum streaming processes one 64-bit beat at a time
   *   - each beat contains four RFC 1071 16-bit words
   */
  require(xLen == 64, "XorFoldAccelerator currently requires RV64")

  /*
   * ------------------------------------------------------------
   * Accumulators
   * ------------------------------------------------------------
   */

  /* 
   * Existing XOR-fold accumulator.
   */
  val accum = RegInit(0.U(xLen.W))

  /* 
   * RFC 1071 checksum accumulator.
   * 
   * Invariant:
   * 
   *   checksumAccum(15, 0)
   * 
   * always contains the current 16-bit one's-complement running
   * sum.
   * 
   * Bits [63:16] are always zero.
   * 
   * This scalar representation makes RFC 1624 incremental updates
   * direct and unambiguous.  
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
   *   Load data and XOR it into accum.
   *  
   * memChecksum:
   *   Load data and one's-complement-add it into checksumAccum.
   *  
   * memWriteResult: 
   *   Store memWriteData to memory.
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
   *   Present memory request until accepted.
   *  
   * sMemResp: 
   *   Wait for completion/response. 
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
  val doChecksumUpdate   = funct === 9.U

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
    state        := sMemReq  
  }

  /* 
   * ------------------------------------------------------------
   * Helper: 16-bit one's-complement addition
   * ------------------------------------------------------------
   *  
   * RFC 1071 and RFC 1624 use one's-complement arithmetic.
   * 
   * +& preserves the carry-out:
   *
   *   [16]   = carry
   *   [15:0] = low result
   * 
   * The carry is then wrapped back into bit 0.  
   */

  def onesComplementAdd16(a: UInt, b: UInt): UInt = {
    
    val wideSum =
      a(15, 0) +& b(15, 0)
    
    val folded =
      wideSum(15, 0) +& wideSum(16)

    folded(15, 0)
  }

  /* 
   * ------------------------------------------------------------
   * Helper: convert a little-endian RV64 memory beat into four
   * network-order 16-bit words
   * ------------------------------------------------------------
   *  
   * Example memory bytes:
   * 
   *   00 01  00 02  00 03  00 04
   * 
   * RV64 load value: 
   *
   *   0x0400030002000100
   * 
   * RFC 1071 needs the 16-bit values:
   *
   *   0x0001
   *   0x0002
   *   0x0003
   *   0x0004
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
   * Helper: fold four 16-bit words into one RFC 1071 sum
   * ------------------------------------------------------------
   * 
   * Input:
   *   [63:48] = word3
   *   [47:32] = word2
   *   [31:16] = word1
   *   [15:0]  = word0
   * 
   * Output:
   *   
   *   one 16-bit one's-complement sum. 
   */
  def fold64To16(x: UInt): UInt = {
    
    val chunk0 = x(15, 0)
    val chunk1 = x(31, 16)
    val chunk2 = x(47, 32)
    val chunk3 = x(63, 48)

    /* 
    * Maximum:
    *
    *   4 * 0xffff = 0x3fffc
    * 
    * so 18 bits are sufficient.
    */
    val chunkSum =
      chunk0.pad(18) +
      chunk1.pad(18) +
      chunk2.pad(18) +
      chunk3.pad(18)

    /* 
    * First end-around carry fold.
    * 
    * Add bits [17:16] back into the low 16 bits.
    */
    val fold1 =
      chunkSum(15, 0).pad(17) +
      chunkSum(17, 16).pad(17)
    
    /* 
    * A second fold handles a possible carry from fold1.
    */
    val fold2 =
      fold1(15, 0) +&
      fold1(16)

    fold2(15, 0)
  }

  /* 
   * ------------------------------------------------------------
   * RFC 1071 finalization
   * ------------------------------------------------------------
   * 
   * checksumAccum is already maintained as a scalar 16-bit sum.
   * 
   * Therefore finalization is simply the one's complement.
   */
  val checksumFinal =
    ~checksumAccum(15, 0)

  /*
   * ------------------------------------------------------------
   * RoCC command handling
   * ------------------------------------------------------------
   */

  val idle = 
    state === sIdle 

  /* 
   * xd means the instruction expects a response through io.resp.
   *  
   * If the response interface cannot accept it, don't consume
   * the command yet.
   */
  val doResp = 
    cmd.bits.inst.xd 

  val stallResp = 
    doResp && !io.resp.ready 

  /* 
   * While the memory FSM is active, no subsequent RoCC command
   * may execute.
   *  
   * This is what serializes:
   * 
   *   fold_mem_n(...)
   *   xorfold_read()
   *  
   * The read cannot fire until the final memory response has been
   * processed and state returns to sIdle.
   */
  cmd.ready := 
    idle && !stallResp

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
   * funct = 9
   * 
   * checksum_update(oldWord, newWord)
   * 
   * RFC 1624 incremental checksum update.
   * 
   * rs1[15:0] = old 16-bit network-order word
   * rs2[15:0] = new 16-bit network-order word
   * 
   * RFC 1624:
   *
   *   C' = ~(~C + ~old + new)
   * 
   * checksumAccum stores the uncomplemented running sum S, where:
   * 
   *   C = ~S
   * 
   * therefore:
   *
   *   C' = ~(S + ~old + new)
   * 
   * So update the internal sum as:
   *
   *   S' = S + ~old + new
   * 
   * A later checksum_finalize() returns ~S', which is C'.
   * 
   * This operation: 
   *
   *   - is register-only
   *   - does not use memory
   *   - does not change the FSM
   *   - does not modify XOR accum
   *   - has no rd response
   */
  when (cmd.fire && doChecksumUpdate) {

    /* 
     * RFC 1624 operates on 16-bit words.
     */
    val oldWord =
      cmd.bits.rs1(15, 0)

    val newWord =
      cmd.bits.rs2(15, 0)

    /* 
     * Remove the old field contribution:
     *
     *   S + ~old
     */
    val afterRemoveOld =
      onesComplementAdd16(
        checksumAccum(15, 0),
        ~oldWord 
      )

    /* 
     * Add the replacement field:
     *
     *   (S + ~old) + new
     */
    val updatedSum =
      onesComplementAdd16(
        afterRemoveOld,
        newWord
      )

    /* 
     * Maintain the scalar checksum invariant:
     *
     *   checksumAccum[63:16] = 0
     *   checksumAccum[15:0]  = current sum
     */
    checksumAccum :=
      updatedSum.pad(xLen)
  }

  /*
    * ------------------------------------------------------------
    * CPU response
    * ------------------------------------------------------------
    *
    * funct=2:
    *   return XOR accum
    * 
    * funct=8:
    *   return finalized RFC 1071 checksum
    *   zero-extended to xLen
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
  
  /* 
   * Full xLen-wide byte mask.
   */
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
      state := 
        sMemResp

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
      state := 
        sIdle 
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
         * Step 1:
         * 
         * Convert each 16-bit lane from the little-endian memory
         * representation into the RFC/network-order value.
         */
        val checksumWord =
          networkOrder16Lanes(
            io.mem.resp.bits.data
          )
        
        /* 
         * Step 2:
         * 
         * Fold the four RFC 16-bit words contained in this 64-bit
         * memory beat into one 16-bit one's-complement contribution.
         */
        val beatSum =
          fold64To16(
            checksumWord 
          )
        
        /* 
         * Step 3:
         * 
         * Add the beat contribution into the persistent scalar
         * checksum sum.
         */
        val nextChecksumSum =
          onesComplementAdd16(
            checksumAccum(15, 0),
            beatSum
          )
        
        /* 
         * Preserve the invariant:
         *
         *   checksumAccum[63:16] = 0
         */
        checksumAccum :=
          nextChecksumSum.pad(xLen)
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

        state := 
          sIdle 
      } .otherwise {
        
        state := 
          sMemReq
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
