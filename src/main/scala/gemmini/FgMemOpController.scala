package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import GemminiISA._
import Util._

class FgMemOpController[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit val p: Parameters) extends CoreModule {
  import config._
  //=========================================================================
  // I/O interface
  //=========================================================================
  val io = IO(new Bundle {
    val cmd       = Flipped(Decoupled(new GemminiCmd(ROB_ENTRIES_IDX)))
    val dma       = new FgMemUnitMemIO(config)
    val completed = Decoupled(UInt(ROB_ENTRIES_IDX.W))
    val busy      = Output(Bool())
  })

  //=========================================================================
  // queue up incoming commands
  //=========================================================================
  val cmd = Queue(io.cmd, MEM_OP_QUEUE_LENGTH)
  val is_config_cmd = cmd.valid && (cmd.bits.cmd.inst.funct === CONFIG_CMD)
  val is_load_cmd   = cmd.valid && (cmd.bits.cmd.inst.funct === CONFIG_LOAD)
  val is_store_cmd  = cmd.valid && (cmd.bits.cmd.inst.funct === CONFIG_STORE)
  val rob_id        = cmd.bits.rob_id
  val mstatus       = cmd.bits.cmd.status
  val vaddr         = cmd.bits.cmd.rs1
  val config_stride = cmd.bits.cmd.rs2
  val lrange        = cmd.bits.cmd.rs2.asTypeOf(new FgLocalRange(config))
  val item_rows     = lrange.rows
  val item_cols     = lrange.cols
  val row_start     = lrange.row_start

  cmd.ready := false.B
  io.busy := cmd.valid

  //=========================================================================
  // Track Outstanding Requests
  //=========================================================================
  val cmd_tracker = Module(new FgMemOpTracker(config))

  cmd_tracker.io.alloc.valid       := false.B
  cmd_tracker.io.alloc.bits.rob_id := cmd.bits.rob_id
  cmd_tracker.io.alloc.bits.rows   := item_rows
  cmd_tracker.io.progress          <> io.dma.resp
  cmd_tracker.io.completed         <> io.completed

  //=========================================================================
  // DMA request
  //=========================================================================
  val stride      = RegInit(0.U(coreMaxAddrBits.W))
  val row_counter = RegInit(0.U(MEM_OP_ROWS_CTR.W))
  val cur_lrange  = WireDefault(lrange)

  // only request 1 row at a time
  cur_lrange.rows      := 1.U
  cur_lrange.row_start := row_start + row_counter

  io.dma.req.valid       := false.B
  io.dma.req.bits.vaddr  := vaddr + (row_counter * stride)
  io.dma.req.bits.lrange := cur_lrange
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
      .elsewhen (is_load_cmd || is_store_cmd) {
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
          assert(cmd.fire())
        } .otherwise {
          row_counter := row_counter + 1.U
        }
      }
    }
  }
}

object FgMemOpController {
  def apply[T <: Data: Arithmetic]
    (config: FgGemminiArrayConfig[T])(implicit p: Parameters)
      = Module(new FgMemOpController(config))
}
