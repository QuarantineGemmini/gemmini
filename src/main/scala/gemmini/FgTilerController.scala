//===========================================================================
// TilerController Implementation
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._

class FgTilerController[T <: Data: Arithmetic]
  (config: FgGemminiArrayConfig[T])(implicit val p: Parameters) 
  extends Module with HasCoreParameters {
  import config._

  //=========================================================================
  // Interface
  //=========================================================================
  val io = IO(new Bundle {
    val cmd_in = Flipped(Decoupled(new TilerCmd(OTYPE_BITS_IDX)))
    val issue = new Bundle {
      val exec   = Decoupled(new GemminiCmd(ROB_ENTRIES_IDX))
      val loadA  = Decoupled(new GemminiCmd(ROB_ENTRIES_IDX))
      val loadB  = Decoupled(new GemminiCmd(ROB_ENTRIES_IDX))
      val loadD  = Decoupled(new GemminiCmd(ROB_ENTRIES_IDX))
      val storeC = Decoupled(new GemminiCmd(ROB_ENTRIES_IDX))
    }
    val completed = new Bundle {
      val exec   = Flipped(Valid(UInt(ROB_ENTRIES_IDX.W)))
      val loadA  = Flipped(Decoupled(UInt(ROB_ENTRIES_IDX.W)))
      val loadB  = Flipped(Decoupled(UInt(ROB_ENTRIES_IDX.W)))
      val loadD  = Flipped(Decoupled(UInt(ROB_ENTRIES_IDX.W)))
      val storeC = Flipped(Decoupled(UInt(ROB_ENTRIES_IDX.W)))
    }
    val busy = Output(Bool())
    // to exec-unit
    val a_fg_mux_sel = Output(UInt(FG_NUM_CTR_CTR.W))
    val b_fg_mux_sel = Output(UInt(FG_NUM_CTR_CTR.W))
  })

  //=========================================================================
  // dispatch incoming commands
  //=========================================================================
  val fsm = FgTilerFSM(config)
  fsm.io.cmd_in   <> io.cmd_in
  io.a_fg_mux_sel := fsm.io.a_fg_mux_sel
  io.b_fg_mux_sel := fsm.io.b_fg_mux_sel

  val sched = FgTilerScheduler(config)
  sched.io.cmd_in <> fsm.io.sched_out
  io.issue.exec   <> sched.io.issue.exec
  io.issue.loadA  <> sched.io.issue.loadA
  io.issue.loadB  <> sched.io.issue.loadB
  io.issue.loadD  <> sched.io.issue.loadD
  io.issue.storeC <> sched.io.issue.storeC

  //=========================================================================
  // arbitrate incoming completions
  //=========================================================================
  val arb = Module(new Arbiter(UInt(ROB_ENTRIES_IDX.W), 5))
  arb.io.in(0).valid := io.completed.exec.valid
  arb.io.in(0).bits  := io.completed.exec.bits
  arb.io.in(1)       <> io.completed.loadA
  arb.io.in(2)       <> io.completed.loadB
  arb.io.in(3)       <> io.completed.loadD
  arb.io.in(4)       <> io.completed.storeC
  sched.io.completed <> arb.io.out

  //=========================================================================
  // busy signal
  //=========================================================================
  io.busy := fsm.io.busy || sched.io.busy
}

object FgTilerController {
  def apply[T <: Data: Arithmetic]
    (config: FgGemminiArrayConfig[T])(implicit p: Parameters)
      = Module(new FgTilerController(config))
}
