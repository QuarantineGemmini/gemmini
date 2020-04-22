//===========================================================================
// DMACmdTracker tracks outstanding mem-ops from Load/Store Controller
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._

class DMACmdTracker[T <: Data](config: GemminiArrayConfig[T])
  (implicit val p: Parameters) extends CoreModule {
  import config._
  //-------------------------------------------------------------------------
  // I/O interface
  //-------------------------------------------------------------------------
  val io = IO(new Bundle {
    val alloc = Flipped(Decoupled(new Bundle {
      val rob_id = UInt(LOG2_ROB_ENTRIES.W)
      val rows   = UInt(LOG2_MAX_MEM_OP_BYTES.W)
    }))
    val progress  = Flipped(Valid(UInt(LOG2_ROB_ENTRIES.W)))
    val completed = Decoupled(UInt(LOG2_ROB_ENTRIES.W))
    val busy      = Output(Bool())
  })

  // slots for outstanding commands
  val cmds = Reg(Vec(ROB_ENTRIES, new Bundle {
    val valid     = Bool()
    val rows_left = UInt(LOG2_MAX_MEM_OP_BYTES.W)
  }))

  // when a new mem-op is allocated
  val cmd_valids = cmds.map(_.valid)
  io.alloc.ready := cmd_valids(io.alloc.bits.rob_id)
  when (io.alloc.fire()) {
    val rob_id = io.alloc.bits.rob_id
    cmds(rob_id).valid     := true.B
    cmds(rob_id).rows_left := io.alloc.bits.rows
  }

  // when a new read/write progress of the mem-op is finished
  when (io.progress.fire()) {
    val rob_id = io.progress.bits.rob_id
    cmds(rob_id).rows_left := cmds(rob_id).rows_left - 1.U
    assert(cmds(rob_id).rows_left > 1.U)
  }

  // complete outstanding command logic
  val cmd_completed_id = MuxCase(0.U, cmds.zipWithIndex.map { 
    case (cmd, i) => (cmd.valid && cmd.rows_left === 0.U) -> i.U
  })
  io.cmd_completed.valid       := cmds(cmd_completed_id).valid
  io.cmd_completed.bits.rob_id := cmd_completed_id
  when (io.complete.fire()) {
    cmds(io.complete.bits.rob_id).valid := false.B
  }

  // busy logic 
  io.busy := cmd_valids.reduce(_ || _)

  // reset logic
  when (reset.toBool()) {
    cmds.foreach(_.valid := false.B)
  }
}

