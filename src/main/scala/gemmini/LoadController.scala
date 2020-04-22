package gemmini

import chisel3._
import chisel3.util._
import GemminiISA._
import Util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._

class FgMemTransferController[T <: Data](config: GemminiArrayConfig[T])
  (implicit val p: Parameters) extends CoreModule {
  import config._
  //=========================================================================
  // I/O interface
  //=========================================================================
  val io = IO(new Bundle {
    val cmd       = Flipped(Decoupled(new GemminiCmd(ROB_ENTRIES)))
    val dma       = new FgScratchpadMemIO(config)
    val completed = Decoupled(UInt(log2Up(ROB_ENTRIES).W))
    val busy      = Output(Bool())
  })

  //=========================================================================
  // queue up incoming commands
  //=========================================================================
  val cmd = Queue(io.cmd, ld_queue_length)
  val is_config_cmd = cmd.valid && cmd.bits.cmd.inst.funct === CONFIG_CMD
  val is_load_cmd   = cmd.valid && cmd.bits.cmd.inst.funct === CONFIG_LOAD
  val vaddr         = cmd.bits.cmd.rs1
  val config_stride = cmd.bits.cmd.rs2
  val item_rows     = cmd.bits.cmd.rs2(63, 48)
  val item_cols     = cmd.bits.cmd.rs2(47, 32)
  val is_acc        = cmd.bits.cmd.rs2(31)
  val is_accum      = cmd.bits.cmd.rs2(30)
  val sq_col_start  = cmd.bits.cmd.rs2(29,16)
  val row_start     = cmd.bits.cmd.rs2(15,0)
  val mstatus       = cmd.bits.cmd.status
  val rob_id        = cmd.bits.rob_id

  cmd.ready := false.B
  io.busy := cmd.valid

  //=========================================================================
  // Track Outstanding Requests
  //=========================================================================
  val cmd_tracker = Module(new DMACmdTracker(config))

  cmd_tracker.io.alloc.valid       := false.B
  cmd_tracker.io.alloc.bits.rob_id := cmd.bits.rob_id
  cmd_tracker.io.alloc.bits.rows   := item_rows
  cmd_tracker.io.progress          <> io.dma.resp
  cmd_tracker.io.completed         <> io.completed

  //=========================================================================
  // DMA request
  //=========================================================================
  val stride      = RegInit(0.U(coreMaxAddrBits.W))
  val row_counter = RegInit(0.U(LOG2_MAX_TRANSFER_ROWS.W))
  val lrange      = cmd.bits.cmd.rs2.asTypeOf[FgLocalRange]

  // only request 1 row at a time
  lrange.rows      := 1.U
  lrange.row_start := row_start + row_counter

  io.dma.req.valid       := false.B
  io.dma.req.bits.vaddr  := vaddr + row_counter * stride
  io.dma.req.bits.laddr  := lrange
  io.dma.req.bits.status := mstatus
  io.dma.req.bits.rob_id := rob_id

  //==========================================================================
  // Request FSM (only sends requests to the DMA engine)
  //==========================================================================
  val (s_IDLE :: s_TRANSFER_ROWS :: Nil) = Enum(2)
  val state = RegInit(s_IDLE)

  switch (state) {
    is (s_IDLE) {
      row_counter := 0.U
      when (is_config_cmd) {
        cmd.ready := true.B
        stride    := config_stride
      }
      .elsewhen (is_load_cmd) {
        cmd_tracker.io.alloc.valid := true.B
        when (cmd_tracker.io.alloc.fire()) {
          state := s_TRANSFER_ROWS
        }
        assert(item_rows =/= 0.U && item_cols =/= 0.U)
      }
    }
    is (s_TRANSFER_ROWS) {
      io.dma.req.valid := true.B
      when (io.dma.req.fire()) {
        when (row_counter === (item_rows - 1.U)) {
          cmd.ready := true.B
          state := s_IDLE
        } .otherwise {
          row_counter := row_counter + 1.U
        }
      }
    }
  }
}

object FgMemTransferController {
  def apply[T <: Data: Arithmetic]
    (config: GemminiArrayConfig[T])(implicit p: Parameters)
      = Module(new FgMemTransferController(config))
}
