package xorfold

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config._

import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.rocket._
import freechips.rocketchip.diplomacy.IdRange

class XorFoldAccelerator(opcodes: OpcodeSet)(implicit p: Parameters) 
  extends LazyRoCC(opcodes) {

  override lazy val module = 
    new XorFoldAcceleratorModuleImp(this)

  /* 
   * ------------------------------------------------------------
   * Raw TileLink master
   * ------------------------------------------------------------
   * 
   * Used by:
   * 
   *   funct=11:
   *     one 64-bit Get
   * 
   *   funct=12:
   *     N-word streaming Get with up to two outstanding requests
   * 
   * Source IDs:
   *
   *   0
   *   1
   * 
   * funct=11 always uses source ID 0.
   * 
   * funct=12 may use source IDs 0 and 1 simultaneously.
   * 
   * HasLazyRoCC automatically connects altNode into the tile-local
   * TileLink master crossbar.
   */
  override val atlNode =
    TLClientNode(
      Seq(
        TLMasterPortParameters.v1(
          Seq(
            TLMasterParameters.v1(
              name     = "XorFoldTl",
              sourceId = IdRange(0, 2)
            )
          )
        )
      )
    )
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
  require(
    xLen == 64, 
    "XorFoldAccelerator currently requires RV64"
  )

  /*
   * ============================================================
   * Accumulators
   * ============================================================
   */

  /* 
   * XOR-fold accumulator.
   */
  val accum = 
    RegInit(0.U(xLen.W))

  /* 
   * RFC 1071 checksum accumulator.
   * 
   * Invariant:
   * 
   *   checksumAccum(15, 0)
   * 
   * contains the current scalar 16-bit one's-complement sum.
   * 
   * Bits [63:16] remains zero.
   * 
   * The scalar representation also makes RFC 1624 incremental
   * updates direct and unambiguous.  
   */
  val checksumAccum = 
    RegInit(0.U(xLen.W))

  /*
   * ============================================================
   * HellaCache memory-operation registers
   * ============================================================
   */

  /*
   * Address of the next HellaCache memory operation.
   */ 
  val ptr = 
    RegInit(0.U(xLen.W))
  
  /* 
   * Number of HellaCache requests that have not yet been issued.
   */
  val remaining = 
    RegInit(0.U(xLen.W))

  /* 
   * Preserve privilege information after the initiating command
   * leaves the RoCC command queue.
   */
  val memDprv = 
    RegInit(0.U(2.W))
  
  val memDv = 
    RegInit(false.B)

  /* 
   * Latched write payload.
   * 
   * funct=5:
   *   one write
   * 
   * funct=10:
   *   broadcast same value to n writes
   */
  val memWriteData = 
    RegInit(0.U(xLen.W))

  /* 
   * ------------------------------------------------------------
   * HellaCache memory operation type
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
   * 
   * funct=5 and funct=10 both use memWriteResult.
   */

  val memXorFold :: memChecksum :: memWriteResult :: Nil = Enum(3)

  val memOp = 
    RegInit(memXorFold)

  /* ------------------------------------------------------------
   * HellaCache FSM
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

  val state = 
    RegInit(sIdle)

  /* 
   * ============================================================
   * Single-request raw TileLink engine - funct=11
   * ============================================================
   */

  /* 
   * Address captures from rs1 when funct=11 is accepted.
   * 
   * It remains stable from command acceptance until the TileLink
   * A-channel request is accepted.
   */
  val tlAddr =
    RegInit(0.U(xLen.W))

  /* 
   * Shared debug-only TileLink error flag.
   * 
   * Used by funct=11 and funct=12.
   * 
   * Set when:
   *
   *   - edge.Get reports that a requested transfer is illegal
   *   - a D-channel response is denied
   *   - returned data is marked corrupt
   * 
   * Software cannot read this flag yet. It is intended for
    waveform/debug inspection.
   */
  val tlError =
    RegInit(false.B)

  /* 
   * funct=11 FSM:
   *
   * tlIdle:
   *   no single-request TL operation active
   * 
   * tlReq:
   *   generate/present one 64-bit Get
   * 
   * tlResp:
   *   request accepted; wait for D response
   */
  val tlIdle :: tlReq :: tlResp :: Nil = Enum(3)

  val tlState =
    RegInit(tlIdle)

  /* 
   * ============================================================
   * Two-outstanding TileLink streaming engine - funct=12
   * ============================================================
   *
   * Unlike funct=11, this engine does not use a monolithic
   * request/response FSM.
   * 
   * TileLink A and D channels may make progress independently
   * during the same cycle.
   */

  /* 
   * True while one funct=12 stream is active.
   */
  val tlStreamActive =
    RegInit(false.B)

  /* 
   * Address of the next 64-bit word that has not yet been issued.
   */
  val tlStreamPtr =
    RegInit(0.U(xLen.W))

  /* 
   * Number of words that have not yet been issued.
   * 
   * Important:
   *
   *   tlToIssue == 0
   * 
   * means every request has been launched.
   * 
   * It does NOT necessarily mean every response has returned.
   */
  val tlToIssue =
    RegInit(0.U(xLen.W))

  /* 
   * Per-source outstanding state.
   * 
   * tlBusy(0):
   *   source ID 0 currently has a Get in flight
   * 
   * tlBusy(1):
   *   source ID 1 currently has a Get in flight
   * 
   * No address or ordering information is required after issue
   * because XOR is associative and commutative.
   */
  val tlBusy =
    RegInit(
      VecInit(
        Seq.fill(2)(false.B)
      )
    )

  /* 
   * Number of requests that have fired on the A channel but have
   * not yet completed on D.
   * 
   * Maximum value is 2, so two bits are sufficient.
   */
  val tlInFlight =
    RegInit(0.U(2.W))

  /* 
   * TileLink output port and negotiated edge.
   */
  val (tlOut, tlEdge) =
    outer.atlNode.out(0)

  /*
   * ============================================================
   * RoCC command queue / decode
   * ============================================================
   */

  val cmd = 
    Queue(io.cmd)

  val funct = 
    cmd.bits.inst.funct

  val doReset            = funct === 0.U
  val doFold             = funct === 1.U

  // funct=2 = ordinary XOR accumulator read
  
  val doFoldMem          = funct === 3.U
  val doFoldMemN         = funct === 4.U
  val doWriteResult      = funct === 5.U 
  val doChecksumReset    = funct === 6.U 
  val doChecksumAddN     = funct === 7.U 
  val doChecksumFinalize = funct === 8.U 
  val doChecksumUpdate   = funct === 9.U
  val doWriteResultN     = funct === 10.U

  /* 
   * Single-word raw TileLink equivalent of funct=3.
   */
  val doFoldMemTL        = funct === 11.U

  /* 
   * N-word raw TileLink stream with up to two outstanding Gets.
   */
  val doFoldMemTLN       = funct === 12.U

  /* 
   * ============================================================
   * Helper: start HellaCache memory operation
   * ============================================================
   *  
   * Every operation entering sMemReq goes through this helper so
   * all persistent HellaCache transaction state is initialized
   * together.
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

    state := 
      sMemReq  
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
  def onesComplementAdd16(
      a: UInt, 
      b: UInt
  ): UInt = {
    
    val wideSum =
      a(15, 0) +& b(15, 0)
    
    val folded =
      wideSum(15, 0) +& wideSum(16)

    folded(15, 0)
  }

  /* 
   * ------------------------------------------------------------
   * Helper: little-endian RV64 beat -> network-order 16-bit lanes
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

  def networkOrder16Lanes(
      d: UInt
  ): UInt = {

    Cat(
      d(55, 48), d(63, 56),
      d(39, 32), d(47, 40),
      d(23, 16), d(31, 24),
      d(7, 0),   d(15, 8)
    )
  }

  /* 
   * ------------------------------------------------------------
   * Helper: four 16-bit words -> one RFC 1071 sum
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
   * checksumAccum is already maintained as a scalar 16-bit sum,
   * so finalization is simply the one's complement.
   */
  val checksumFinal =
    ~checksumAccum(15, 0)

  /*
   * ------------------------------------------------------------
   * Global accelerator-idle definition
   * ------------------------------------------------------------
   * 
   * HellaCache, single-request TileLink, and streaming TileLink
   * operations are deliberately serialized.
   * 
   * A new RoCC command can execute only when all three engines 
   * are idle.
   */

  val memIdle =
    state === sIdle 
  
  val tlIdleNow =
    tlState === tlIdle

  val idle = 
    memIdle && tlIdleNow && !tlStreamActive

  /* 
   * xd means the instruction expects a response through io.resp.
   */
  val doResp = 
    cmd.bits.inst.xd 

  val stallResp = 
    doResp && !io.resp.ready 

  /* 
   * No new RoCC command executes while the memory FSM is active.
   */
  cmd.ready := 
    idle && !stallResp

  /*
   * ------------------------------------------------------------
   * funct = 0
   * 
   * XOR accumulator reset.
   * ------------------------------------------------------------
   */
  when (cmd.fire && doReset) {
    accum := 0.U
  }

  /*  
   * ------------------------------------------------------------
   * funct = 1
   *  
   * Register/register XOR fold:
   *
   *   accum ^= rs1 + rs2
   * ------------------------------------------------------------
   */
  when (cmd.fire && doFold) {
    accum := 
      accum ^ (cmd.bits.rs1 + cmd.bits.rs2)
  }

  /* 
   * ------------------------------------------------------------
   * funct = 3
   *  
   * fold_mem(ptr)
   *  
   * Existing one-word HellaCache load.
   * ------------------------------------------------------------
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
   * ------------------------------------------------------------
   * funct = 4
   *  
   * fold_mem_n(ptr, n)
   *  
   * Existing sequential HellaCache N-word load.
   * 
   * rs1 = starting address
   * rs2 = number of 64-bit words
   *  
   * n == 0 is a no-op.
   * ------------------------------------------------------------
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
   * ------------------------------------------------------------
   * funct = 5
   *  
   * write_result(dst_ptr)
   * 
   * Single-word HellaCache write.
   * ------------------------------------------------------------
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
   * funct = 6
   * 
   * checksum_reset()
   * 
   * Reset only the checksum accumulator.
   * ------------------------------------------------------------
   */
  when (cmd.fire && doChecksumReset) {

    checksumAccum := 0.U
  }

  /* 
   * ------------------------------------------------------------
   * funct = 7
   * 
   * checksum_add_n(ptr, n)
   * 
   * rs1 = starting buffer address
   * rs2 = number of 8-byte beats
   * 
   * n == 0 is a no-op.
   * ------------------------------------------------------------
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
   * ------------------------------------------------------------
   * funct = 8
   * 
   * checksum_finalize()
   * 
   * No state-changing when-block is required.
   * 
   * The generic response path returns checksumFinal.
   * ------------------------------------------------------------
   */

  /* 
   * ------------------------------------------------------------
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
   * ------------------------------------------------------------
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
   * funct = 10
   * 
   * write_result_n(dst_ptr, n)
   * 
   * rs1 = starting destination address
   * rs2 = number of 64-bit words to write
   * 
   * The current accum value is captured once when the command is
   * accepted and broadcast to all n consecutive destination words.
   * 
   * Example:
   *
   *   accum = 0x77
   *   n     = 3
   * 
   * produces:
   * 
   *   [dst +  0] = 0x77
   *   [dst +  8] = 0x77
   *   [dst + 16] = 0x77
   * 
   * n == 0 is a no-op.
   * ------------------------------------------------------------
   */

  when (cmd.fire && doWriteResultN) {

    when (cmd.bits.rs2 =/= 0.U) {

      startMemOp(
        addr      = cmd.bits.rs1,
        count     = cmd.bits.rs2 ,
        op        = memWriteResult,
        writeData = accum,
        dprv      = cmd.bits.status.dprv,
        dv        = cmd.bits.status.dv 
      )
    }
  }

  /* 
   * ------------------------------------------------------------
   * funct = 11
   * 
   * fold_mem_tl(ptr)
   * 
   * Raw TileLink counterpart of funct=3.
   * 
   * rs1 = address of one 64-bit word
   * 
   * Semantics:
   *
   *   accum := accum ^ memory[rs1]
   * 
   * This first version allows exactly one outstanding TL request.
   * ------------------------------------------------------------
   */

  when (cmd.fire && doFoldMemTL) {

    /* 
     * Capture the address before the command leaves the queue.
     * 
     * The subsequent TileLink transaction depends only on tlAddr,
     * never on cmd.bits.
     */
    tlAddr :=
      cmd.bits.rs1

    /* 
     * Clear the TL debug error flag for the new operation.
     */
    tlError :=
      false.B

    tlState :=
      tlReq
  }

  /* 
   * ------------------------------------------------------------
   * funct=12
   * 
   * fold_mem_tl_n(ptr, n)
   * 
   * rs1 = address of first 64-bit word
   * rs2 = number of 64-bit words
   * 
   * At most two raw TileLink Gets may be outstanding 
   * simultaneously.
   * 
   * Response ordering does not matter:
   *
   *   A ^ B ^ C == C ^ A ^ B
   * 
   * n == 0 is a no-op.
   * ------------------------------------------------------------
   */
  when (cmd.fire && doFoldMemTLN) {

    /* 
     * Every new TileLink operation begins with a clean debug
     * error state.
     */
    tlError :=
      false.B 

    when (cmd.bits.rs2 =/= 0.U) {
      
      tlStreamPtr :=
        cmd.bits.rs1

      tlToIssue :=
        cmd.bits.rs2

      tlBusy(0) :=
        false.B 

      tlBusy(1) :=
        false.B 

      tlInFlight :=
        0.U 

      tlStreamActive :=
        true.B 
    }
  }

  /*
    * ------------------------------------------------------------
    * CPU response path
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
   * HellaCache path
   * ------------------------------------------------------------
   * 
   * Once the FSM enters sMemReq, the request no longer depends
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
   * memXorFold:
   *   load for funct=3 / funct=4
   * 
   * memChecksum:
   *   load for funct=7
   * 
   * memWriteResult:
   *   store for funct=5 / funct=10
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
   * Read operations do not use req.bits.data.
   * 
   * For memWriteResult, this is the latched accum value written by:
   *
   *   funct=5  -> one destination word
   *   funct=10 -> n consecutive destination words
   */
  io.mem.req.bits.data :=
    Mux(
      memOp === memWriteResult, 
      memWriteData, 
      0.U
    )
  
  /* 
   * Enable all bytes of the 64-bit memory operation.
   */
  io.mem.req.bits.mask := 
    Fill(
      xLen / 8, 
      1.U(1.W))

  /* 
   * HellaCache path uses virtual-address translation.
   * 
   * This differs from the raw TileLink path below.
   */
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
   * HellaCache request accepted
   * ------------------------------------------------------------
   * 
   * Stream bookkeeping is identical for reads and writes.
   * 
   * A request counts as issued only when io.mem.req.fire occurs.
   */

  when (state === sMemReq && io.mem.req.fire) {

    /* 
     * Advance to the next 64-bit word.
     */
    ptr :=
      ptr + (xLen / 8).U 

    /* 
     * remaining means:
     * 
     *   number of memory requests not yet issued
     * 
     * so decrement it when the request is actually accepted.
     */
    remaining :=
      remaining - 1.U

    /* 
     * Wait for this operation's response before issuing the next
     * request.
     */
    state :=
      sMemResp 
  }

  /*
   * ------------------------------------------------------------
   * HellaCache response / completion
   * ------------------------------------------------------------
   */

  when (state === sMemResp && io.mem.resp.valid) {

    /* 
     * Only reads consume resp.bits.data.
     * 
     * Stores simply use resp.valid as the completion indication.
     */

    when (memOp === memXorFold) {

      /* 
       * XOR memory fold.
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
       * Fold the four 16-bit words in this 64-bit beat into one
       * 16-bit one's-complement contribution.
       */
      val beatSum =
        fold64To16(
          checksumWord
        )

      /* 
       * Step 3:
       *
       * Add this beat's contribution onto the scalar checksum.
       */
      val nextChecksumSum =
        onesComplementAdd16(
          checksumAccum(15, 0),
          beatSum 
        )

      checksumAccum :=
        nextChecksumSum.pad(xLen)
    }

    /* 
     * memWriteResult intentionally has no data-path update here.
     * 
     * The store response only tells us that the current write has
     * completed.
     */

    /* 
     * Shared stream control:
     *
     * remaining was decremented when the corresponding request
     * fired.
     * 
     * Therefore remaining === 0 means the response arriving now
     * belongs to the final operation in the stream.
     */
    when (remaining === 0.U) {

      state :=
        sIdle

    } .otherwise {

      state :=
        sMemReq 
    }
  }

  /* 
   * ============================================================
   * Raw TileLink request construction
   * ============================================================
   */

  /* 
   * ------------------------------------------------------------
   * funct=11 request
   * ------------------------------------------------------------
   * 
   * Generate one 8-byte Get using source ID 0.
   * 
   * edge.Get is combinational, so tlGetLegal is known before an
   * A-channel handshake occurs.
   */
  val (tlGetLegal, tlGet) =
    tlEdge.Get(
      fromSource = 0.U,
      toAddress  = tlAddr,
      lgSize     = log2Ceil(xLen / 8).U
    )

  /* 
   * ------------------------------------------------------------
   * funct=12 source allocator
   * ------------------------------------------------------------
   * 
   * Fixed priority:
   *
   *   source 0 wins whenever both are free.
   * 
   * The two source IDs are functionally identical; this policy is
   * simply deterministic and easy to reason about.
   */
  val tlSource0Free =
    !tlBusy(0)

  val tlSource1Free =
    !tlBusy(1)

  val tlHasFreeSource =
    tlSource0Free || tlSource1Free

  val tlIssueSource =
    Mux(
      tlSource0Free,
      0.U,
      1.U
    )

  /* 
   * ------------------------------------------------------------
   * Same-cycle D error visibility
   * ------------------------------------------------------------
   * 
   * If a denied/corrupt D response is already visible this cycle,
   * do not launch one more request before tlError is registered.
   * 
   * Because the engines are globally serialized, any D response
   * while tlStreamActive belongs to funct=12.
   */
  val tlStreamDVisibleError =
    tlStreamActive && 
    tlOut.d.valid &&
    (
      tlOut.d.bits.denied || tlOut.d.bits.corrupt
    )

  /* 
   * ------------------------------------------------------------
   * funct=12 issue eligibility
   * ------------------------------------------------------------
   */
  val tlStreamCanIssue =
    tlStreamActive &&
    !tlError &&
    !tlStreamDVisibleError &&
    (tlToIssue =/= 0.U) &&
    tlHasFreeSource

  /* 
   * Candidate streaming Get.
   * 
   * edge.Get is combinational, so legality is known before the
   * A-channel request is allowed to fire.
   */
  val (tlStreamGetLegal, tlStreamGet) =
    tlEdge.Get(
      fromSource = tlIssueSource,
      toAddress  = tlStreamPtr,
      lgSize     = log2Ceil(xLen / 8).U 
    )

  /* 
   * ------------------------------------------------------------
   * Shared TileLink A channel
   * ------------------------------------------------------------
   * 
   * funct=11 and funct=12 share this physical A channel.
   * 
   * Global RoCC serialization guarantees they cannot be active
   * simultaneously.
   */
  val singleTLValid =
    (tlState === tlReq) &&
    tlGetLegal

  val streamTLValid =
    tlStreamCanIssue &&
    tlStreamGetLegal

  tlOut.a.valid :=
    singleTLValid || streamTLValid

  tlOut.a.bits :=
    Mux(
      tlStreamActive,
      tlStreamGet,
      tlGet
    )

  /* 
   * ------------------------------------------------------------
   * Illegal funct=11 request
   * ------------------------------------------------------------
   * 
   * Reject locally before A.valid can handshake.
   */
  when (
    tlState === tlReq &&
    !tlGetLegal
  ) {

    tlError :=
      true.B 

    tlState :=
      tlIdle
  }

  /* 
   * ------------------------------------------------------------
   * Illegal funct=12 request
   * ------------------------------------------------------------
   * 
   * Once an illegal request is detected:
   *
   *   - do not issue it
   *   - stop issuing future requests
   *   - allow already-outstanding requests to drain
   */
  val tlStreamIllegal =
    tlStreamCanIssue &&
    !tlStreamGetLegal

  when (tlStreamIllegal) {

    tlError :=
      true.B 
  }

  /* 
   * ============================================================
   * A-channel handshakes
   * ============================================================
   */
  val singleAFire =
    tlOut.a.fire &&
    (tlState === tlReq)

  val streamAFire =
    tlOut.a.fire &&
    tlStreamActive

  /* 
   * ------------------------------------------------------------
   * funct=11 A handshake
   * ------------------------------------------------------------
   */
  when (singleAFire) {

    tlState :=
      tlResp 
  }

  /* 
   * ------------------------------------------------------------
   * funct=12 A handshake
   * ------------------------------------------------------------
   */
  when (streamAFire) {

    /* 
     * Mark selected source ID occupied.
     */
    when (tlIssueSource === 0.U) {

      tlBusy(0) :=
        true.B 
    } .otherwise {

      tlBusy(1) :=
        true.B 
    }

    /* 
     * Advance to the next 64-bit word.
     */
    tlStreamPtr :=
      tlStreamPtr + (xLen / 8).U 

    /* 
     * One fewer request remains to be issued.
     */
    tlToIssue :=
      tlToIssue - 1.U 
  }

  /* 
   * ============================================================
   * Shared TileLink D channel
   * ============================================================
   *
   * funct=11:
   *   accept D only in tlResp
   * 
   * funct=12:
   *   accept D for the full duration of tlStreamActive
   */
  tlOut.d.ready :=
    (tlState === tlResp) || tlStreamActive

  val singleDFire =
    tlOut.d.fire && (tlState === tlResp)

  val streamDFire = 
    tlOut.d.fire && tlStreamActive

  /* 
   * ============================================================
   * funct=11 response
   * ============================================================
   */
  when (singleDFire) {

    /* 
     * Failed accesses leave accum untouched.
     */
    when (
      tlOut.d.bits.denied ||
      tlOut.d.bits.corrupt 
    ) {

      tlError :=
        true.B 
    } .otherwise {

      /* 
       * Plain XOR treats the loaded 64-bit word as opaque data.
       * 
       * No byte swapping is needed.
       */
      accum :=
        accum ^ tlOut.d.bits.data 
    }

    tlState :=
      tlIdle
  }

  /* 
   * ============================================================
   * funct=12 response error
   * ============================================================
   */

  val tlStreamResponseError =
    streamDFire &&
    (
      tlOut.d.bits.denied ||
      tlOut.d.bits.corrupt
    )

  /* 
   * ============================================================
   * funct=12 response processing
   * ============================================================
   */

  when (streamDFire) {

    when (tlStreamResponseError) {

      /* 
       * Stop future issue.
       * 
       * Requests already in flight are still drained.
       */
      tlError :=
        true.B 
    } .otherwise {

      /* 
       * Fold successful response immediately.
       * 
       * Return order does not matter:
       *
       *   A ^ B ^ C == C ^ A ^ B
       */
      accum := 
        accum ^ tlOut.d.bits.data 
    }

    /* 
     * Free whichever source ID generated this response.
     */
    when (tlOut.d.bits.source === 0.U) {

      tlBusy(0) :=
        false.B
    } .otherwise {

      tlBusy(1) :=
        false.B 
    }
  }

  /* 
   * ============================================================
   * funct=12 outstanding-request count
   * ============================================================
   *
   * A and D may fire during the same cycle.
   * 
   * Cases:
   * 
   *   A=0 D=0 -> hold
   *   A=1 D=0 -> +1
   *   A=0 D=1 -> -1
   *   A=1 D=1 -> hold
   * 
   * Chisel registers retain their previous value when no
   * conditional assignment fires, so no explicit self-assignment
   * is required for 00 or 11.
   */

  when (
    streamAFire &&
    !streamDFire
  ) {

    tlInFlight :=
      tlInFlight + 1.U

  } .elsewhen (
    !streamAFire &&
    streamDFire
  ) {

    tlInFlight :=
      tlInFlight - 1.U 
  }

  /* 
   * ============================================================
   * funct=12 next-state bookkeeping
   * ============================================================
   *
   * These combinational values describe what the issue and
   * outstanding counts will mean after this cycle's handshakes.
   */

  val tlToIssueNext =
    Mux(
      streamAFire,
      tlToIssue - 1.U,
      tlToIssue
    )

  val tlInFlightNext =
    Mux(
      streamAFire && !streamDFire,

      tlInFlight + 1.U,

      Mux(
        !streamAFire && streamDFire,

        tlInFlight - 1.U,

        tlInFlight
      )
    )

  /* 
   * Include both previously registered errors and errors first
   * discovered during the current cycle.
   */
  val tlStreamErrorNext =
    tlError ||
    tlStreamIllegal ||
    tlStreamResponseError

  /* 
   * ------------------------------------------------------------
   * Normal completion
   * ------------------------------------------------------------
   * 
   * Every request has been issued AND every response has returned.
   */

  val tlStreamNormalDone =
    tlStreamActive &&
    !tlStreamErrorNext &&
    (tlToIssueNext === 0.U) &&
    (tlInFlightNext === 0.U)

  /* 
   * ------------------------------------------------------------
   * Abort completon
   * ------------------------------------------------------------
   * 
   * Once an error occurs, remaining unissued words are abondoned.
   * 
   * Completion waits only for already-issued requests to drain.
   */

  val tlStreamAbortDone =
    tlStreamActive &&
    tlStreamErrorNext &&
    (tlInFlightNext === 0.U)

  /* 
   * ------------------------------------------------------------
   * End funct=12
   * ------------------------------------------------------------
   * 
   * tlStreamActive clears on the clock edge containing final
   * completion.
   * 
   * cmd.ready still sees the OLD tlStreamActive value during that
   * cycle, so the next RoCC command cannot fire until the following
   * cycle.
   * 
   * This deliberate one-cycle boundary prevents the final D
   * response and a newly accepted command from both attempting to
   * modify accum on the same edge.
   */

  when (
    tlStreamNormalDone || tlStreamAbortDone
  ) {

    tlStreamActive :=
      false.B 
  }

  /* 
   * ------------------------------------------------------------
   * Unused TileLink channels
   * ------------------------------------------------------------
   *
   * This client issues only ordinary Gets, so it does not use the
   * B, C, or E channels.
   * 
   * This matches the tie-offs used by Rocket Chip's current
   * CharacterCountExample.
   */
  tlOut.b.ready :=
    true.B 

  tlOut.c.valid :=
    false.B 

  tlOut.e.valid :=
    false.B

  /*
   * ------------------------------------------------------------
   * Accelerator status
   * ------------------------------------------------------------
   */

  io.busy :=
    cmd.valid ||
    state =/= sIdle ||
    tlState =/= tlIdle ||
    tlStreamActive

  io.interrupt := 
    false.B 
}
