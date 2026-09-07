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

  val accum = RegInit(0.U(xLen.W))

  val ptr       = RegInit(0.U(xLen.W))
  val remaining = RegInit(0.U(xLen.W))

  val memDprv = RegInit(0.U(2.W))
  val memDv   = RegInit(false.B)

  val sIdle :: sMemReq :: sMemResp :: Nil = Enum(3)
  val state = RegInit(sIdle)

  val cmd = Queue(io.cmd)

  val funct = cmd.bits.inst.funct

  val doReset    = funct === 0.U
  val doFold     = funct === 1.U
  val doRead     = funct === 2.U
  val doFoldMem  = funct === 3.U
  val doFoldMemN = funct === 4.U

  /*
   * ------------------------------------------------------------
   * RoCC command handling
   * ------------------------------------------------------------
   */

  val doResp = cmd.bits.inst.xd 

  val idle      = state === sIdle 
  val stallResp = doResp && !io.resp.ready 

  cmd.ready := idle && !stallResp

  /* 
   * Ordinary reset.
   */
  when (cmd.fire && doReset) {
    accum := 0.U
  }

  /* 
   * Ordinary register/register fold.
   */
  when (cmd.fire && doFold) {
    accum := accum ^ (cmd.bits.rs1 + cmd.bits.rs2)
  }

  when (cmd.fire && doFoldMem) {
    ptr       := cmd.bits.rs1
    remaining := 1.U 

    memDprv := cmd.bits.status.dprv 
    memDv   := cmd.bits.status.dv 

    state := sMemReq 
  }

  when (cmd.fire && doFoldMemN) {
    when (cmd.bits.rs2 =/= 0.U) {
      ptr       := cmd.bits.rs1
      remaining := cmd.bits.rs2

      memDprv := cmd.bits.status.dprv
      memDv   := cmd.bits.status.dv 

      state := sMemReq 
    }
  }

  /*
   * ------------------------------------------------------------
   * CPU response
   * ------------------------------------------------------------
   */

  io.resp.valid := 
    cmd.valid && 
    doResp && 
    idle  

  io.resp.bits.rd := cmd.bits.inst.rd
  io.resp.bits.data := accum

  /*
   * ------------------------------------------------------------
   * Memory request
   * ------------------------------------------------------------
   */

  io.mem.req.valid := state === sMemReq 

  io.mem.req.bits.addr := ptr
  io.mem.req.bits.tag := 0.U 
  io.mem.req.bits.cmd := M_XRD
  io.mem.req.bits.size := log2Ceil(xLen / 8).U 
  io.mem.req.bits.signed := false.B 
  io.mem.req.bits.data := 0.U 

  io.mem.req.bits.phys := false.B 
  io.mem.req.bits.dprv := memDprv
  io.mem.req.bits.dv := memDv 
  io.mem.req.bits.no_resp := false.B 

  when (state === sMemReq && io.mem.req.fire) {
    ptr       := ptr + (xLen / 8).U
    remaining := remaining - 1.U 

    state := sMemResp 
  }

  /*
   * ------------------------------------------------------------
   * Memory response / streaming loop
   * ------------------------------------------------------------
   */

  when (state === sMemResp && io.mem.resp.valid) {
    accum := accum ^ io.mem.resp.bits.data 

    when (remaining === 0.U) {
      state := sIdle 
    }.otherwise {
      state := sMemReq
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
