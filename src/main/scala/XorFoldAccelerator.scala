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

  val busy = RegInit(false.B)

  val cmd = Queue(io.cmd)

  val funct = cmd.bits.inst.funct

  val doReset   = funct === 0.U
  val doFold    = funct === 1.U
  val doRead    = funct === 2.U
  val doFoldMem = funct === 3.U

  when (cmd.fire && doReset) {
    accum := 0.U
  }
  
  when (cmd.fire && doFold) {
    accum := accum ^ (cmd.bits.rs1 + cmd.bits.rs2)
  }

  when (io.mem.resp.valid) {
    accum := accum ^ io.mem.resp.bits.data 
    busy := false.B
  }

  val doResp = cmd.bits.inst.xd

  val stallBusy = busy 
  val stallMem  = doFoldMem && !io.mem.req.ready 
  val stallResp = doResp && !io.resp.ready

  cmd.ready := !stallBusy && !stallMem && !stallResp

  io.resp.valid := cmd.valid && doResp && !stallBusy && !stallMem 
  io.resp.bits.rd := cmd.bits.inst.rd
  io.resp.bits.data := accum

  io.busy := cmd.valid
  io.interrupt := false.B

  io.mem.req.valid := cmd.valid && doFoldMem && !busy && !stallResp

  io.mem.req.bits.addr := cmd.bits.rs1
  io.mem.req.bits.tag := 0.U 
  io.mem.req.bits.cmd := M_XRD
  io.mem.req.bits.size := log2Ceil(xLen / 8).U 
  io.mem.req.bits.signed := false.B 
  io.mem.req.bits.data := 0.U 

  io.mem.req.bits.phys := false.B 
  io.mem.req.bits.dprv := cmd.bits.status.dprv 
  io.mem.req.bits.dv := cmd.bits.status.dv 
  io.mem.req.bits.no_resp := false.B 

  when (io.mem.req.fire) {
    busy := true.B
  }

}
