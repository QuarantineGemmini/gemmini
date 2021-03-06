//===========================================================================
// TilerController Implementation
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._

class TilerController[T <: Data: Arithmetic]
  (config: GemminiArrayConfig[T])(implicit val p: Parameters) 
  extends Module with HasCoreParameters {
  import config._

  //=========================================================================
  // Interface
  //=========================================================================
  val io = IO(new Bundle {
    val cmd_in = Flipped(Decoupled(new TilerCmd(LOG2_OTYPE_BITS)))
    val issue = new Bundle {
      val exec  = Decoupled(new GemminiCmd(ROB_ENTRIES))
      val load  = Decoupled(new GemminiCmd(ROB_ENTRIES))
      val store = Decoupled(new GemminiCmd(ROB_ENTRIES))
    }
    val completed = new Bundle {
      val exec  = Flipped(Valid(UInt(LOG2_ROB_ENTRIES.W)))
      val load  = Flipped(Decoupled(UInt(LOG2_ROB_ENTRIES.W)))
      val store = Flipped(Decoupled(UInt(LOG2_ROB_ENTRIES.W)))
    }
    val busy = Output(Bool())
  })

  //=========================================================================
  // dispatch incoming commands
  //=========================================================================
  val fsm = TilerFSM(config)
  fsm.io.cmd_in <> io.cmd_in

  val sched = TilerScheduler(config)
  sched.io.cmd_in <> fsm.io.sched_out
  io.issue.exec   <> sched.io.issue.exec
  io.issue.load   <> sched.io.issue.load
  io.issue.store  <> sched.io.issue.store

  //=========================================================================
  // arbitrate incoming completions
  //=========================================================================
  val arb = Module(new Arbiter(UInt(LOG2_ROB_ENTRIES.W), 3))
  arb.io.in(0).valid := io.completed.exec.valid
  arb.io.in(0).bits  := io.completed.exec.bits
  arb.io.in(1)       <> io.completed.load
  arb.io.in(2)       <> io.completed.store
  sched.io.completed <> arb.io.out

  //=========================================================================
  // busy signal
  //=========================================================================
  io.busy := fsm.io.busy || sched.io.busy
}

object TilerController {
  def apply[T <: Data: Arithmetic]
    (config: GemminiArrayConfig[T])(implicit p: Parameters)
      = Module(new TilerController(config))
}
