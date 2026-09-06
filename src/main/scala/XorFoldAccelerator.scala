package xorfold

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._

class XorFoldAccelerator(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  override lazy val module = new XorFoldAcceleratorModuleImp(this)
}

class XorFoldAcceleratorModuleImp(outer: XorFoldAccelerator)(implicit p: Parameters) extends LazyRoCCModuleImp(outer)
    with HasCoreParameters {

  val accum = RegInit(0.U(xLen.W))

  val cmd = Queue(io.cmd)
  val funct = cmd.bits.inst.funct

  val doReset = funct === 0.U
  val doFold  = funct === 1.U
  val doRead  = funct === 2.U

  when (cmd.fire && doReset) {
    accum := 0.U
  }
  when (cmd.fire && doFold) {
    accum := accum ^ (cmd.bits.rs1 + cmd.bits.rs2)
  }

  val doResp = cmd.bits.inst.xd
  cmd.ready := !doResp || io.resp.ready

  io.resp.valid := cmd.valid && doResp
  io.resp.bits.rd := cmd.bits.inst.rd
  io.resp.bits.data := accum

  io.busy := cmd.valid
  io.interrupt := false.B

  io.mem.req.valid := false.B
}
