//============================================================================
// Fine-grained-systolic-array exec-controller
// (new parameter: fg_sa_div)
//
// WARNING!!! the 'b-addr' and 'd-addr' within this ExecuteController and
// all submodules are swapped with the meaning of 'b-addr' and 'd-addr'
// in all external modules! this was due to originally supporting WS and OS
// modes simultaneously. Just remember that d-addr within this module means
// the weights that is preloaded into the matrix, and the 'b-addr' is the
// address that is preloaded into the accumulator (so it isn't used here!)
//============================================================================
package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import GemminiISA._
import Util._

class FgExecuteController[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit val p: Parameters, ev: Arithmetic[T])
  extends Module with HasCoreParameters {
  import config._
  import ev._
  val A_TYPE = Vec(FG_NUM, Vec(FG_DIM, inputType))
  val B_TYPE = Vec(FG_NUM, Vec(FG_DIM, inputType))
  //=========================================================================
  // I/O interface
  //=========================================================================
  val io = IO(new Bundle {
    val cmd        = Flipped(Decoupled(new GemminiCmd(ROB_ENTRIES_IDX)))
    val readA      = new FgMemUnitExecReadIO(config, A_SP_FG_COLS)
    val readB      = new FgMemUnitExecReadIO(config, B_SP_FG_COLS)
    val writeC     = new FgMemUnitExecWriteReq(config)
    val acc_config = new FgAccumulatorBankConfigIO(config)
    val completed  = Valid(UInt(ROB_ENTRIES_IDX.W))
    val busy       = Output(Bool())
    val prof       = Input(new Profiling)
  })

  io.completed.valid := false.B
  io.completed.bits  := DontCare

  //=========================================================================
  // input cmd parsing and config-state
  //=========================================================================
  val cmd_q_heads = 2
  assert(ex_queue_length >= cmd_q_heads)
  val (cmd, _) = MultiHeadedQueue(io.cmd, ex_queue_length, cmd_q_heads)
  cmd.pop := 0.U

  val functs = cmd.bits.map(_.cmd.inst.funct)
  val rs1s   = VecInit(cmd.bits.map(_.cmd.rs1))
  val rs2s   = VecInit(cmd.bits.map(_.cmd.rs2))

  val is_config_cmd  = cmd.valid(0) && (functs(0) === CONFIG_CMD)
  val is_compute_cmd = cmd.valid(0) && (functs(0) === COMPUTE_AND_FLIP_CMD ||
                                        functs(0) === COMPUTE_AND_STAY_CMD)
  val preload_idx    = Mux(is_compute_cmd, 1.U, 0.U)
  val is_preload_cmd = cmd.valid(preload_idx) &&
                       (functs(preload_idx) === PRELOAD_CMD)

  val preload_rs1 = rs1s(preload_idx).asTypeOf(new FgLocalRange(config))
  val preload_rs2 = rs2s(preload_idx).asTypeOf(new FgLocalRange(config))
  val compute_rs1 = rs1s(0).asTypeOf(new FgLocalRange(config))
  val compute_rs2 = rs2s(0).asTypePf(new FgLocalRange(config))

  val in_prop = functs(0) === COMPUTE_AND_FLIP_CMD

  val a_lrange = compute_rs1
  val b_lrange = preload_rs1
  val c_lrange = preload_rs2

  val multiply_garbage = compute_rs1.garbage // A bits garbage
  val preload_zeros    = preload_rs1.garbage // B bits garbage
  val c_garbage        = preload_rs2.garbage

  //=========================================================================
  // configuration state
  // - NOTE: these configs should not be serviced by the ExController...
  //=========================================================================
  val acc_shift   = Reg(UInt(OTYPE_BITS_IDX.W))
  val relu6_shift = Reg(UInt(OTYPE_BITS_IDX.W))
  val activation  = Reg(UInt(2.W))

  io.acc_config.acc_shift    := acc_shift
  io.acc_config.relu6_lshift := relu6_shift
  io.acc_config.act          := activation

  //=========================================================================
  // FG Mesh Muxing Control
  //=========================================================================
  val mesh_partition_list = (0 to log2Up(FG_NUM)).map { e=>pow(2,e).toInt }

  val b_fg_mux_cntl = Vec(FG_NUM, Bool())
  val a_fg_mux_cntl = Vec(FG_NUM, Bool())

  //TODO this needs to not change when computing in the mesh
  // Bucket the computation for assigning to FG arrays
  for (i <- 0 until mesh_partition_list.length) {
    b_fg_mux_cntl(i) := (b_cols > (mesh_partition_list(i) * FG_DIM).U)
  }

  for (i <- 0 until mesh_partition_list.length) {
    a_fg_mux_cntl(i) := (a_rows > mesh_partition_list(i).U)
  }

  //=========================================================================
  // fix-latency scratchpad-inputs read
  //=========================================================================
  val sp_read_counter = RegInit(0.U(FG_DIM_IDX.W))

  //----------------------------------------
  // a_data input
  //----------------------------------------
  val is_reading_a     = WireInit(false.B)
  val a_nonzero_cycles = Mux(a_lrange.rows > FG_DIM.U,FG_DIM.U,a_lrange.rows)
  val a_maxbank_rows   = a_lrange.rows(FG_DIM_IDX-1,0)
  val a_nonzero_cols   = a_lrange.cols
  val a_banks          = Mux(sp_read_counter < a_maxbank_rows,
                             a_lrange.total_banks(),
                             a_lrange.total_banks() - 1.U)
  val a_read_en        = (sp_read_counter < a_nonzero_cycles) && is_reading_a

  io.readA.req.bits.en           := a_read_en
  io.readA.req.bits.row          := sp_read_counter
  io.readA.req.bits.fg_col_start := a_lrange.fg_col_start
  io.readA.req.bits.bank_start   := a_lrange.bank_start()
  io.readA.req.bits.banks        := a_banks

  // get data back 2 cycles later
  val a_read_en_buf      = ShiftRegister(a_read_en, SP_RD_CYCLES)
  val a_nonzero_cols_buf = ShiftRegister(a_nonzero_cols, SP_RD_CYCLES)
  val a_data_prepad      = io.readA.bits.resp.data.asTypeOf(A_TYPE)
  val a_data             = WireDefault(0.U.asTypeOf(A_TYPE))

  for (i <- 0 until FG_NUM; j <- 0 until FG_DIM) {
    when(a_read_en_buf && (j.U < a_nonzero_cols_buf)) {
      a_data(i)(j) := a_data_prepad(i)(j)
    }
  }

  //----------------------------------------
  // b_data input
  //----------------------------------------
  val is_reading_b    = WireInit(false.B)
  val b_nonzero_rows  = b_lrange.rows
  val b_nonzero_cols  = b_lrange.cols
  val b_read_en       = (sp_read_counter < b_nonzero_rows) && is_reading_b

  io.readB.req.bits.en           := b_read_en
  io.readB.req.bits.row          := sp_read_counter
  io.readB.req.bits.fg_col_start := b_lrange.fg_col_start
  io.readB.req.bits.bank_start   := b_lrange.bank_start()
  io.readB.req.bits.banks        := b_banks

  // get data back 2 cycles later
  val b_read_en_buf      = ShiftRegister(b_read_en, SP_RD_CYCLES)
  val b_nonzero_cols_buf = ShiftRegister(b_nonzero_cols, SP_RD_CYCLES)
  val b_data_prepad      = io.readB.bits.resp.data.asTypeOf(B_TYPE)
  val b_data             = WireDefault(0.U.asTypeOf(B_TYPE))

  for (i <- 0 until FG_NUM; j <- 0 until FG_DIM) {
    when(b_read_en_buf && ((i*j).U < b_nonzero_cols_buf)) {
      b_data(i)(j) := b_data_prepad(i)(j)
    }
  }

  //=========================================================================
  // global FSM
  //=========================================================================
  val (s_IDLE :: s_PRELOAD :: s_MUL :: s_MUL_PRE) :: Nil = Enum(4)
  val state = RegInit(s_IDLE)

  // if we are committing the mul tag back to the rob. the backend datapath
  // needs to stall its write back to the rob if this is happening on the
  // same cycle
  val is_mul_tag_finished = WireInit(false.B)

  switch (state) {
    is (s_IDLE) {
      when(is_config_cmd && !io.busy) {
        activation  := rs1s(0)(4, 3)
        acc_shift   := cmd.bits(0).cmd.rs1(63, 32)
        relu6_shift := cmd.bits(0).cmd.rs2(63, 32)

        io.completed.valid := true.B
        io.completed.bits := cmd.bits(0).rob_id

        cmd.pop := 1.U
      }
      .elsewhen (is_compute_cmd && is_preload_cmd) {
        state := s_MUL_PRE
      }
      .elsewhen (is_compute_cmd) {
        state := s_MUL
      }
      .elsewhen (is_preload_cmd) {
        state := s_PRELOAD
      }
    }
    is (s_PRELOAD) {
      is_reading_b    := !b_lrange.garbage
      sp_read_counter := sp_read_counter + 1.U

      when (sp_read_counter === (FG_DIM-1.U)) {
        cmd.pop         := 1.U
        sp_read_counter := 0.U
        state           := s_IDLE
      }
    }
    is (s_MUL_PRE) {
      assert(!a_lrange.garbage, "a cannot be garbage")
      is_reading_a    := true.B
      is_reading_b    := !b_lrange.garbage
      sp_read_counter := sp_read_counter + 1.U

      when (sp_read_counter === (FG_DIM-1.U)) {
        cmd.pop := 2.U
        is_mul_tag_finished := true.B
        io.completed.valid  := true.B
        io.completed.bits   := cmd.bits(0).rob_id
        sp_read_counter     := 0.U
        state               := s_IDLE
      }
    }
    is (s_MUL) {
      assert(!a_lrange.garbage, "a cannot be garbage")
      is_reading_a    := true.B
      sp_read_counter := sp_read_counter + 1.U

      when (sp_read_counter === (FG_DIM-1.U)) {
        cmd.pop := 1.U
        is_mul_tag_finished := true.B
        io.completed.valid  := true.B
        io.completed.bits   := cmd.bits(0).rob_id
        sp_read_counter     := 0.U
        state               := s_IDLE
      }
    }
  }

  //=========================================================================
  // mesh-ctrl input signals 
  // - buffer all mesh-ctrl signals for 2 cycles (sp delay is fixed 2-cycles)
  //=========================================================================
  class ComputeCntrlSignals extends Bundle {
    val valid    = Bool()
    val rob_id   = UDValid(UInt(ROB_ENTRIES_IDX.W))
    val c_lrange = new FgLocalRange(config)
    val row_idx  = UInt(FG_DIM_CTR.W)
    val prop     = UInt(1.W)
    val last_row = Bool()
  }
  val mesh_ctrl = new ComputeCntrlSignals
  val mesh_ctrl_buf = ShiftRegister(mesh_ctrl, SP_RD_LATENCY)

  //------------------------------------------------------------------------
  // write next command for meshq input datapath to handle
  //------------------------------------------------------------------------
  mesh_ctrl.valid         := state === s_PRELOAD ||
                             state === s_MUL_PRE ||
                             state === s_MUL
  mesh_ctrl.rob_id        := cmd.bits(preload_idx).rob_id
  mesh_ctrl.c_lrange      := c_lrange 
  mesh_ctrl.row_idx       := sp_read_counter
  mesh_ctrl.prop          := in_prop
  mesh_ctrl.bits.last_row := sp_read_counter === (FG_DIM-1.U)

  //========================================================================
  // Instantiate the actual mesh and connect non-blocking inputs
  //========================================================================
  val mesh = Module(new FgMesh(config))
  val mesh_in_c_lrange = mesh_ctrl_buf.c_lrange
  mesh_in_c_lrange.row := mesh_ctrl_buf.row_idx

  mesh.io.a.valid           := a_read_en_buf
  mesh.io.a.bits            := a_data
  mesh.io.a.valid           := b_read_en_buf
  mesh.io.a.bits            := b_data
  mesh.io.a_mux_ctrl        := mesh_ctrl_buf.a_fg_mux_ctrl
  mesh.io.b_mux_ctrl        := mesh_ctrl_buf.b_fg_mux_ctrl
  mesh.io.tag_in.rob_id     := mesh_ctrl_buf.rob_id
  mesh.io.tag_in.c_lrange   := mesh_in_c_lrange
  mesh.io.pe_ctrl.propagate := mesh_ctrl_buf.prop
  mesh.io.prof              := io.prof

  //=========================================================================
  // Mesh->Scratchpad/Accumulator write datapath
  //=========================================================================
  val wb_valid        = mesh.io.out.fire()
  val wb_data         = mesh.io.out.bits
  val wb_rob_id       = mesh.io.tag_out.rob_id
  val wb_lrange       = mesh.io.tag_out.c_lrange
  val wb_cols         = wb_lrange.cols
  val wb_fg_col_start = wb_lrange.fg_col_start
  val wb_bank_start   = wb_lrange.bank_start
  val wb_banks        = wb_lrange.total_banks()
  val wb_accum        = wb_lrange.is_accum
  val wb_garbage      = wb_lrange.garbage
  val wb_rows         = Mux(wb_lrange.rows > FG_DIM.U, 
                            FG_DIM.U, wb_lrange.rows)
  val wb_maxbank_rows = wb_lrange.rows(FG_DIM_IDX-1,0)
  val wb_row          = wb_lrange.row
  val wb_banks        = Mux(wb_row < wb_maxbank_rows,
                            wb_lrange.total_banks(),
                            wb_lrange.total_banks() - 1.U)

  val is_outputting      = wb_valid && !wb_garbage
  val is_outputting_last = is_outputting && (wb_row === (wb_rows-1.U))

  io.writeC.en           := is_outputting
  io.writeC.row          := wb_row
  io.writeC.cols         := wb_cols
  io.writeC.fg_col_start := wb_fg_col_start
  io.writeC.bank_start   := wb_bank_start
  io.writeC.banks        := wb_banks
  io.writeC.accum        := wb_accum
  io.writeC.data         := wb_data

  //-------------------------------------------------------------------------
  // commit the pending preload tag on the last output row
  //-------------------------------------------------------------------------
  val pending_preload_tag_complete
    = RegInit(0.U.asTypeOf(UDValid(UInt(ROB_ENTRIES_IDX.W))))

  when(is_outputing_last) {
    assert(!pending_preload_tag_complete.valid,
      "can't output last row when we have an existing pending tag!")
    when (is_mul_tag_finished) {
      pending_preload_tag_complete.valid := true.B
      pending_preload_tag_complete.bits := mesh.io.tag_out.rob_id.bits
    } .otherwise {
      io.completed.valid := true.B
      io.completed.bits := mesh.io.tag_out.rob_id.bits
    }
  }

  when (pending_preload_tag_complete.valid && ~is_mul_tag_finished) {
    io.completed.valid := true.B
    io.completed.bits := pending_preload_tag_complete.bits
    pending_preload_tag_complete.valid := false.B
  }

  //=========================================================================
  // busy calculation
  //=========================================================================
  io.busy := (state =/= s_IDLE) || mesh.io.busy

  //=========================================================================
  // hardware profiling counters (non architecturally visible!)
  //=========================================================================
  //withReset(io.prof.start) {
  //  val bank_stall_a_cycles = RegInit(0.U(16.W))
  //  val bank_stall_b_cycles = RegInit(0.U(16.W))
  //  val bank_stall_d_cycles = RegInit(0.U(16.W))
  //  val col_usage   = RegInit(VecInit(Seq.fill(DIM)(0.U(32.W))))
  //  val row_usage   = RegInit(VecInit(Seq.fill(DIM)(0.U(32.W))))
  //  val total_usage = RegInit(0.U(32.W))
  //
  //  bank_stall_a_cycles := bank_stall_a_cycles +(is_inputting_a && !a_valid)
  //  bank_stall_b_cycles := bank_stall_b_cycles +(is_inputting_b && !b_valid)
  //  bank_stall_d_cycles := bank_stall_d_cycles +(is_inputting_d && !d_valid)

  //  for(c <- 0 to DIM) {
  //    col_usage(c) := col_usage(c) && (c < cntlq.c_cols) &&

  //  when(io.prof.end) {
  //    printf(s"G2-PERF[%d]: bank-stall-a-cycles: %d\n",
  //            io.prof.debug_cycle, bank_stall_a_cycles)
  //    printf(s"G2-PERF[%d]: bank-stall-b-cycles: %d\n",
  //            io.prof.debug_cycle, bank_stall_b_cycles)
  //    printf(s"G2-PERF[%d]: bank-stall-d-cycles: %d\n",
  //            io.prof.debug_cycle, bank_stall_d_cycles)

  //    printf(s"G2-PERF[%d]: bank-stall-d-cycles: %d\n",
  //            io.prof.debug_cycle, bank_stall_d_cycles)
  //  }
  //}
}

object FgExecuteController {
  def apply[T <: Data: Arithmetic]
    (config: FgGemminiArrayConfig[T])(implicit p: Parameters)
      = Module(new FgExecuteController(config))
}
