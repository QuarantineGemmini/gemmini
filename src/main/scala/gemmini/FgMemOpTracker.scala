//===========================================================================
// tracks outstanding mem-ops from FgMemOpController
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._

class FgMemOpTracker[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreModule {
  import config._
  //-------------------------------------------------------------------------
  // I/O interface
  //-------------------------------------------------------------------------
  val io = IO(new Bundle {
    val alloc = Flipped(Decoupled(new Bundle {
      val rob_id = UInt(ROB_ENTRIES_IDX.W)
      val rows   = UInt(MEM_OP_ROWS_CTR.W)
    }))
    val progress  = Flipped(Decoupled(new FgMemUnitMemOpResp(config)))
    val completed = Decoupled(UInt(ROB_ENTRIES_IDX.W))
    val busy      = Output(Bool())
  })

  // slots for outstanding commands
  val cmds = Reg(Vec(ROB_ENTRIES, new Bundle {
    val valid     = Bool()
    val rows_left = UInt(MEM_OP_ROWS_CTR.W)
  }))

  // when a new mem-op is allocated
  val cmd_valids = VecInit(cmds.map(_.valid))
  io.alloc.ready := !cmd_valids(io.alloc.bits.rob_id)
  when (io.alloc.fire()) {
    val rob_id = io.alloc.bits.rob_id
    cmds(rob_id).valid     := true.B
    cmds(rob_id).rows_left := io.alloc.bits.rows
  }

  // when a new read/write progress of the mem-op is finished
  io.progress.ready := true.B
  when (io.progress.fire()) {
    val rob_id = io.progress.bits.rob_id
    cmds(rob_id).rows_left := cmds(rob_id).rows_left - 1.U
    assert(cmds(rob_id).rows_left >= 1.U)
  }

  // complete outstanding command logic
  val cmd_completed_id = MuxCase(0.U, cmds.zipWithIndex.map { 
    case (cmd, i) => (cmd.valid && cmd.rows_left === 0.U) -> i.U
  })
  io.completed.valid := cmds(cmd_completed_id).valid &&
                        cmds(cmd_completed_id).rows_left === 0.U
  io.completed.bits := cmd_completed_id
  when (io.completed.fire()) {
    cmds(io.completed.bits).valid := false.B
  }

  // busy logic 
  io.busy := cmd_valids.reduce(_ || _)

  // reset logic
  when (reset.toBool()) {
    cmds.foreach(_.valid := false.B)
  }
}

