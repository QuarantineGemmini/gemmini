//===========================================================================
// FlushController: executes tlb-flush commands
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._

class FlushController(config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends Module with HasCoreParameters
{
  import config._

  // interface
  val io = IO(new Bundle {
    val cmd         = Flipped(Decoupled(new GemminiCmd(rob_entries)))
    val completed   = Decoupled(UInt(log2Up(rob_entries).W))
    val flush_retry = Output(Bool())
    val flush_skip  = Output(Bool())
    val busy        = Output(Bool())
  })
  val cmd = Queue(io.cmd)

  // implementation
  val is_flush = cmd.bits.cmd.inst.funct === FLUSH_CMD
  val skip     = cmd.bits.cmd.rs1(0)

  cmd.ready          := completed.ready
  io.completed.valid := cmd.valid
  io.completed.bits  := cmd.bits.rob_id
  io.flush_retry     := cmd.fire() && !skip
  io.flush_skip      := cmd.fire() && skip
  io.busy            := cmd.valid
}

object FlushController {
  def apply(config: GemminiArrayConfig[T])(implicit p: Parameters) 
    = Module(new FlushController(config))
}
