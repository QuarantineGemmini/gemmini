package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import GemminiISA._
import Util._

class MemOpController[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreModule {
  import config._
  //=========================================================================
  // I/O interface
  //=========================================================================
  val io = IO(new Bundle {
    val cmd        = Flipped(Decoupled(new GemminiCmd(ROB_ENTRIES_IDX)))
    val dma        = new MemUnitMemOpIO(config)
    val completed  = Decoupled(UInt(ROB_ENTRIES_IDX.W))
    val busy       = Output(Bool())
    val csr        = Input(new GemminiCSR(OTYPE_BITS_IDX))
  })

  //=========================================================================
  // queue up incoming commands
  //=========================================================================
  val cmd = Queue(io.cmd, MEM_OP_QUEUE_LENGTH)
  val is_load_cmd    = cmd.valid && (cmd.bits.funct === LOAD_CMD)
  val is_store_cmd   = cmd.valid && (cmd.bits.funct === STORE_CMD)
  val is_invalid_cmd = cmd.valid && !(is_load_cmd || is_store_cmd)
  assert(!is_invalid_cmd, "only LOAD or STORE commands are valid")

  val rob_id   = cmd.bits.rob_id
  val mstatus  = csr.status
  val vaddr    = cmd.bits.rs1
  val operand  = cmd.bits.rs2(1,0)
  val addr_cfg = Mux1H(
    (operand === MATRIX_ID_A) -> io.csr.addr_a_cfg,
    (operand === MATRIX_ID_B) -> io.csr.addr_b_cfg,
    (operand === MATRIX_ID_C) -> io.csr.addr_c_cfg,
    (operand === MATRIX_ID_D) -> io.csr.addr_d_cfg,
  )

  val lrange    = cmd.bits.cmd.rs2.asTypeOf(new FgLocalRange(config))
  val matrix_id = lrange.matrix_id
  val is_acc    = lrange.is_acc
  val row_start = lrange.row_start

  val elem_bytes = Mux(is_store_cmd || !is_acc, ITYPE_BYTES.U, OTYPE_BYTES.U)

  cmd.ready := false.B
  io.busy := cmd.valid

  //=========================================================================
  // Track Outstanding Requests (TODO, fix this!)
  //=========================================================================
  val cmd_tracker = Module(new MemOpTracker(config))
  val is_normal_xfer = WireDefault(true.B)

  // set cmd_tracker.io.alloc.bits.reqs below!
  cmd_tracker.io.alloc.valid       := false.B
  cmd_tracker.io.alloc.bits.rob_id := rob_id
  cmd_tracker.io.progress          <> io.dma.resp
  cmd_tracker.io.completed         <> io.completed

  //=========================================================================
  // DMA request
  //=========================================================================
  val row_counter = RegInit(0.U(MEM_OP_ROWS_CTR.W))
  val col_counter = RegInit(0.U(DIM.W))

  io.dma.req.valid       := false.B
  io.dma.req.bits.is_acc := lrange.is_acc
  io.dma.req.bits.row    := lrange.row + row_counter
  io.dma.req.bits.status := mstatus
  io.dma.req.bits.rob_id := rob_id


  when (is_normal_xfer) {
    cmd_tracker.io.alloc.bits.reqs := item_rows

    io.dma.req.bits.col_start     := 0.U
    io.dma.req.bits.cols          := lrange.cols
    io.dma.req.bits.pad_col_start := 0.U
    io.dma.req.bits.pad_cols      := DIM.U
    io.dma.req.bits.vaddr   := vaddr + (row_counter * addr_cfg.normal_stride)
    io.dma.req.bits.vstride := elem_bytes
    io.dma.req.bits.vchunk  := elem_bytes
  } 
  .otherwise {
    val g_BASE_ADDR = addr_cfg.vaddr
    val g_IN_ROWS   = addr_cfg.in_rows
    val g_IN_COLS   = addr_cfg.in_cols
    val g_IN_CHANS  = addr_cfg.in_channels
    val g_PADDING   = addr_cfg.padding
    val g_KSTRIDE   = addr_cfg.stride
    val g_KROWS     = addr_cfg.kernel_size
    val g_KCOLS     = addr_cfg.kernel_size
    val g_OUT_IMG_ROWS = (g_IN_ROWS + 2.U*g_PADDING -g_KROWS+1.U) / g_KSTRIDE
    val g_OUT_IMG_COLS = (g_IN_COLS + 2.U*g_PADDING -g_KCOLS+1.U) / g_KSTRIDE
    val g_DOT_PROD_DIM   = g_KROWS * g_KCOLS * g_IN_CHANS
    val g_DOT_PROD_BYTES = DOT_PROD_DIM * 
                           Mux(ITYPE_BYTES.U 
    //-------------------------------------------------------------
    val out_row  = ((vaddr - g_BASE_ADDR) / g_DOT_PROD_BYTES) + row_counter
    val out_col  = ((vaddr - g_BASE_ADDR) % g_DOT_PROD_BYTES) + col_counter

    val ichan    = out_col / (g_KCOLS * g_KROWS)
    val krow     = (out_col / g_KCOLS) % g_KROWS
    val kcol     = out_col % g_KCOLS
    val cur_cols = g_KCOLS - kcol

    val batch       = out_row / (g_OUT_IMG_ROWS * g_OUT_IMG_COLS)
    val out_img_row = (out_row / g_OUT_IMG_COLS) % g_OUT_IMG_ROWS
    val out_img_col = out_row % g_OUT_IMG_COLS
    val in_img_row  = (out_img_row *g_STRIDE - g_PADDING + krow).asSInt()
    val in_img_col_start = (out_img_col *g_STRIDE - g_PADDING + kcol).asSInt()
    val in_img_col_end   = in_img_col_start + (cur_cols - 1.U).asSInt()
 
    val is_row_zeros = (in_img_row < g_PADDING) || (in_img_row > g_IN_ROWS)
    val start_zeros  = Mux(in_img_col_start >= 0.S, 0.U,
                           (-in_img_col_start).asUInt())
    val end_zeros    = Mux(in_img_col_end < g_IN_COLS.asSInt(), 0.U,
                           (in_img_col_end-g_IN_COLS.asSInt()).asUInt()+1.U)

    val req_col_start     = col_counter + start_zeros
    val req_cols          = Mux(is_row_zeros, 0.U, 
                                cur_cols - start_zeros - end_zeros)
    val req_pad_col_start = col_counter
    val req_pad_cols      = cur_cols

    cmd_tracker.io.alloc.bits.reqs := item_rows * jk:

    io.dma.req.bits.col_start     := req_col_start
    io.dma.req.bits.cols          := req_cols
    io.dma.req.bits.pad_col_start := req_pad_col_start
    io.dma.req.bits.pad_cols      := req_pad_cols
    io.dma.req.bits.vaddr := g_BASE_ADDR + 
                             (((batch * g_IN_ROWS + in_img_row) * g_IN_COLS +
                              (in_img_col_start + start_zeros)) * g_IN_CHANS +
                              ichan) * elem_bytes
    io.dma.req.bits.vstride := g_IN_CHANS * elem_bytes
    io.dma.req.bits.vchunk  := elem_bytes
  }

  //==========================================================================
  // Request FSM (only sends requests to the DMA engine)
  //==========================================================================
  val (s_IDLE :: s_NORMAL_TRANSFER :: s_IM2COL_TRANSFER :: Nil) = Enum(3)
  val state = RegInit(s_IDLE)

  switch (state) {
    is (s_IDLE) {
      col_counter := 0.U
      row_counter := 0.U
      when (is_load_cmd || is_store_cmd) {
        cmd_tracker.io.alloc.valid := true.B
        when (cmd_tracker.io.alloc.fire()) {
          state := Mux(addr_cfg.mode === ADDR_MODE_NORMAL, 
                    s_NORMAL_TRANSFER, s_IM2COL_TRANSFER)
        }
        assert(lrange.total_bytes() > 0)
      }
    }
    is (s_NORMAL_TRANSFER) {
      io.dma.req.valid := true.B
      when (io.dma.req.fire()) {
        val row_counter_next = row_counter + 1.U
        row_counter := row_counter_next
        when (row_counter_next === lrange.rows) {
          state := s_IDLE
          row_counter := 0.U
          cmd.ready := true.B
          assert(cmd.fire())
        }
      }
    }
    is (s_IM2COL_TRANSFER) {
      io.dma.req.valid := true.B
      when (io.dma.req.fire()) {
        val col_counter_next = col_counter + req_pad_cols
        col_counter := col_counter_next 
        when (col_counter_next === lrange.cols) {
          col_counter := 0.U
          val row_counter_next = row_counter + 1.U
          row_counter := row_counter_next
          when (row_counter_next === lrange.rows) {
            state := s_IDLE
            row_counter := 0.U
            cmd.ready := true.B
            assert(cmd.fire())
          }
        }
      }
    }
  }
}

object MemOpController {
  def apply[T <: Data: Arithmetic]
    (config: GemminiArrayConfig[T])(implicit p: Parameters)
      = Module(new MemOpController(config))
}
