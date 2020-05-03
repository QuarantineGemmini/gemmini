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

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths}

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import GemminiISA._
import Util._

class FgExecuteController[T <: Data](config: GemminiArrayConfig[T])
  (implicit val p: Parameters, ev: Arithmetic[T])
  extends Module with HasCoreParameters {
  import config._
  import ev._
  //=========================================================================
  // module interface
  //=========================================================================

  val A_TYPE = Vec(FG_NUM, Vec(FG_DIM, inputType))
  val B_TYPE = Vec(FG_NUM, Vec(FG_DIM, inputType))

  val io = IO(new Bundle {
    val cmd = Flipped(Decoupled(new GemminiCmd(rob_entries)))

    val memReadA = FgMemUnitExecReadIO(config)
    val memReadB = FgMemUnitExecReadIO(config)
    val memWriteC = FgMemUnitExecWriteReq(config)
    val accConfig = FgAccumulatorBankConfigIO(config)
    val completed = Valid(UInt(log2Up(rob_entries).W))
    val busy = Output(Bool())
    val prof = Input(new Profiling)
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
  val rs1s = VecInit(cmd.bits.map(_.cmd.rs1))
  val rs2s = VecInit(cmd.bits.map(_.cmd.rs2))

  val DoPreloads = functs.map(_ === PRELOAD_CMD)
  val DoComputes = functs.map(_ === COMPUTE_ALL)


  val DoConfig = functs(0) === CONFIG_CMD
  val DoComputes = functs.map(f => f === COMPUTE_AND_FLIP_CMD ||
                                   f === COMPUTE_AND_STAY_CMD)
  val DoPreloads = functs.map(_ === PRELOAD_CMD)

  val preload_cmd_place = Mux(DoPreloads(0), 0.U, 1.U)

  val preload_rs1 = rs1s(preload_cmd_place).asTypeOf(new FgLocalRange)
  val preload_rs2 = rs2s(preload_cmd_place).asTypeOf(new FgLocalRange)

  val compute_rs1 = rs1s(0).asTypeOf(new FgLocalRange)
  val compute_rs2 = rs2s(0).asTypePf(new FgLocalRange)


  val in_prop = functs(0) === COMPUTE_AND_FLIP_CMD

  // configuration state
  val acc_shift   = Reg(UInt(log2Up(accType.getWidth).W))
  val relu6_shift = Reg(UInt(log2Up(accType.getWidth).W))
  val activation  = Reg(UInt(2.W))

  val multiply_garbage = compute_rs1.garbage // A bits garbage
  val preload_zeros = preload_rs1.garbage // B bits garbage
  val c_garbage = preload_rs2.garbage

  val a_cols = compute_rs1.cols
  val a_rows = compute_rs1.rows
  val a_bank_start = compute_rs1.bank_start()
  val a_bank_end = compute_rs1.bank_end()
  val a_total_banks = compute_rs1.total_banks()
  val a_row_end = compute_rs1.row_end()
  val a_col_start = compute_rs1.col_start()
  val a_col_end = compute_rs1.col_end()
  val a_fg_col_start = compute_rs1.fg_col_start
  val a_row_start = compute_rs1.row_start

  val b_cols = preload_rs1.bits.cols
  val b_rows = preload_rs1.bits.rows
  val b_bank_start = preload_rs1.bank_start()
  val b_bank_end = preload_rs1.bank_end()
  val b_total_banks = preload_rs1.total_banks()
  val b_row_end = preload_rs1.row_end()
  val b_col_start = preload_rs1.col_start()
  val b_col_end = preload_rs1.col_end()
  val b_fg_col_start = preload_rs1.fg_col_start
  val b_row_start = preload_rs1.row_start

  val c_cols = preload_rs2.cols
  val c_rows = preload_rs2.rows
  val c_bank_start = preload_rs2.bank_start()
  val c_bank_end = preload_rs2.bank_end()
  val c_total_banks = preload_rs2.total_banks()
  val c_row_end = preload_rs2.row_end()
  val c_col_start = preload_rs2.col_start()
  val c_col_end = preload_rs2.col_end()
  val c_fg_col_start = preload_rs2.fg_col_start
  val c_row_start = preload_rs2.row_start

  val is_accum = preload_rs2.is_accum

  //=========================================================================
  // Queue between the frontend FSM and the backend datapath
  // - the frontend FSM schedules input reads to scratchpad/accumulator and
  //   puts the corresponding mesh command in the queue.
  // - then, the backend datapath reads from the queue after a fixed number of
  //   cycles, when the scratchpad/accumulator inputs are valid
  //=========================================================================

  class ComputeCntrlSignals extends Bundle {
    val do_mul_pre        = Bool()
    val do_single_mul     = Bool()
    val do_single_preload = Bool()

    val a_cols = UInt()
    val a_rows = UInt()
    val a_bank_start = UInt()
    val a_bank_end = UInt()
    val a_total_banks = UInt()
    val a_row_end = UInt()
    val a_col_start = UInt()
    val a_col_end = UInt()
    val a_fg_col_start = UInt()
    val a_row_start = UInt()
    val a_row_end = UInt()

    val b_cols = UInt()
    val b_rows = UInt()
    val b_bank_start = UInt()
    val b_bank_end = UInt()
    val b_total_banks = UInt()
    val b_row_end = UInt()
    val b_col_start = UInt()
    val b_col_end = UInt()
    val b_fg_col_start = UInt()
    val b_row_start = UInt()
    val b_row_end = UInt()

    val c_cols = UInt()
    val c_rows = UInt()
    val c_bank_start = UInt()
    val c_bank_end = UInt()
    val c_total_banks = UInt()
    val c_row_end = UInt()
    val c_col_start = UInt()
    val c_col_end = UInt()
    val c_fg_col_start = UInt()
    val c_row_start = UInt()
    val c_row_end = UInt()

    val accum = Bool()

    val preload_zeros = Bool()
    val a_garbage = Bool()
    val b_garbage = Bool()
    val c_garbage = Bool()

    val rob_id = UDValid(UInt(log2Up(rob_entries).W))
    val prop = UInt(1.W)
    val last_row = Bool()

  }


  val mesh_cntl_signals_q = Module(new Queue(
                          new ComputeCntlSignals, mem_pipeline+1, pipe=true))
  val cntl_ready = mesh_cntl_signals_q.io.enq.ready



  //=========================================================================
  // FG Mesh Muxing Control
  //=========================================================================

  // TODO: generate list of powers-of-2 up to FG_NUM
  val mesh_partition_list = Seq(1, 2, 4, 8, 16)

  val b_fg_arrange = Vec(FG_NUM, Bool())
  val a_fg_arrange = Vec(FG_NUM, Bool())

  // Bucket the computation for assigning to FG arrays
  for (i <- 0 until mesh_partition_list.length) {
    when (b_cols < (mesh_partition_list(i) * FG_DIM).U) {
      b_fg_arrange(i) := true.B
    } .otherwise {
      b_fg_arrange(i) := false.B
    }
  }

  for (i <- 0 until mesh_partition_list.length) {
    when (a_rows < mesh_partition_list(i).U) {
      a_fg_arrange(i) := true.B
    } .otherwise {
      a_fg_arrange(i) := false.B
    }
  }


  //=========================================================================
  // Preload Control
  //=========================================================================

  // number of cycles to load data into mesh after reading from spad and padding
  val READ_B_LATENCY = FG_DIM + 2 // Two cycle latency for reads

  val is_reading_b = WireInit(false.B)
  val b_read_counter = Reg(UInt(FG_DIM_IDX.W))  //Required width

  val b_padded_rows = FG_DIM.U - b_rows
  val b_padded_cols = b_total_cols -

  //TODO stop counting after rows rather than FG_DIM
  when (!is_reading_b) {
    b_read_counter := 0.U
  } .elsewhen (is_reading_b) //other signals? if not, can switch to .otherwise
    .when (b_read_counter === READ_B_LATENCY.U) {
      b_read_counter := READ_B_LATENCY.U
    } .otherwise {
      b_read_counter := b_read_counter + 1.U
    }
  }

  //TODO include garbage? account for preload_zeros
  val b_read_en = (b_read_counter < b_rows) && is_reading_b

  io.memReadB.req.bits.en := b_read_en
  io.memReadB.req.bits.row := b_read_counter
  io.memReadB.req.bits.fg_col_start := b_fg_col_start
  io.memReadB.req.bits.bank_start := b_bank_start
  io.memReadB.req.bits.banks := b_total_banks

  val b_read_data = io.memReadB.bits.resp.data

  //TODO input type?
  val b_mesh_input_prepad = Vec(FG_NUM, Vec(FG_DIM, inputType))
  val b_mesh_input = Vec(FG_NUM, Vec(FG_DIM, inputType))

  // TODO: fix bit slicing?
  for (i <- 0 until FG_NUM) {
    for j <- 0 until FG_DIM) {
      b_mesh_input_prepad(i,j) := b_read_data((FG_NUM-i)*(FG_DIM-j)*inputType,(FG_NUM-i)*(FG_DIM-j-1)*inputType)
    }
  }

  //TODO check if off by one
  // Pad unused rows and columns with zero
  for (i <- 0 until FG_NUM) {
    for (j <- 0 until FG_DIM) {
      when (b_read_counter > b_rows + 2.U) { // +2 accounts for b_read_counter incrementing for 2 cycles already
        b_mesh_input(i,j) := inputType.zero
      } .elsewhen (b_cols < (FG_NUM*i + j).U) { //zero out all columns above the desired number
        b_mesh_input(i,j) := inputType.zero
      } .otherwise {
        b_mesh_input(i,j) := b_mesh_input_prepad(i,j)
      }
    }
  }


  //=========================================================================
  // Compute Control
  //=========================================================================

  val READ_A_LATENCY = FG_DIM + 2

  val is_reading_a = WireInit(false.B)
  val a_read_counter = Reg(UInt(FG_DIM_IDX.W))


  // TODO: terminate read early if fewer rows that one FG_DIM
  when (!is_reading_a) {
    a_read_counter := 0.U
  } .elsewhen (is_reading_a) {
      .when (a_read_counter === READ_A_LATENCY.U) {
        a_read_counter := READ_A_LATENCY.U
      } .otherwise {
        a_read_counter := a_read_counter + 1.U
      }
  }

  // catches case when a_rows < FG_DIM
  val a_read_en = a_read_counter < a_rows && a_read_counter < FG_DIM.U && is_reading_a

  io.memReadA.req.bits.en := a_read_en
  io.memReadA.req.bits.row := a_read_counter
  io.memReadA.req.bits.fg_col_start := a_fg_col_start
  io.memReadA.req.bits.bank_start := a_bank_start
  io.memReadA.req.bits.banks := a_total_banks

  val a_read_data = io.memReadA.bits.resp.data

  val a_mesh_input_prepad = Vec(FG_NUM, Vec(FG_DIM, inputType))

  // TODO: fix bit slicing?
  for (i <- 0 until FG_NUM) {
    for j <- 0 until FG_DIM) {
      a_mesh_input(i,j) := a_read_data((FG_NUM-i)*(FG_DIM-j)*inputType,(FG_NUM-i)*(FG_DIM-j-1)*inputType)
    }
  }

  //TODO check off by one?
  // Pad the unused rows and columns with zero
  for (i <- 0 until FG_NUM) {
    for (j <- 0 until FG_DIM) {
      when (a_read_counter + (i*FG_DIM).U) > a_rows + 2.U) { // +2 accounts for a_read_counter incrememnting for 2 cycles already
        a_mesh_input(i,j) := inputType.zero
      } .elsewhen (a_cols < j.U) { // zero out all columns above a_cols
        b_mesh_input(i,j) := inputType.zero
      } .otherwise {
        b_mesh_input(i,j) := b_mesh_input_prepad(i,j)
      }
    }
  }

  //=========================================================================
  // global FSM
  //=========================================================================
  val s_IDLE :: s_PRELOAD :: s_MUL :: s_MUL_PRE :: Nil = Enum(4)
  val state = RegInit(s_IDLE)
  val state_n = WireInit(state)
  state := state_n

  // if we are committing the mul tag back to the rob. the backend datapath
  // needs to stall its write back to the rob if this is happening on the
  // same cycle
  val is_mul_tag_finished = WireInit(false.B)

  switch (state) {
    is (s_IDLE) {
      when(cmd.valid(0)) {
        when(DoConfig && !io.busy) {
          activation  := rs1s(0)(4, 3)
          acc_shift   := cmd.bits(0).cmd.rs1(63, 32)
          relu6_shift := cmd.bits(0).cmd.rs2(63, 32)

          io.completed.valid := true.B
          io.completed.bits := cmd.bits(0).rob_id

          cmd.pop := 1.U
        }
        .elsewhen (DoPreloads(0) && cmd.valid(1)) {
          state_n := s_PRELOAD
        }
        .elsewhen (DoComputes(0) && cmd.valid(1) && DoPreloads(1)) {
          state := s_MUL_PRE
        }
        .elsewhen (DoComputes(0)) {
          state := s_MUL
        }
      }
    }
    is (s_PRELOAD) {
      is_reading_b := true.B
      when (b_read_counter === READ_B_LATENCY) {
        cmd.pop := 1.U
        state_n := s_IDLE
      }
    }
    is (s_MUL_PRE) {
      is_reading_a := true.B
      is_reading_b := true.B
      when (b_read_counter === READ_B_LATENCY && a_read_counter === READ_A_LATENCY) {
        cmd.pop := 2.U

        is_mul_tag_finished := true.B
        io.completed.valid := true.B
        io.completed.bits  := cmd.bits(0).rob_id

        state_n := s_IDLE
      }
    }
    is (s_MUL) {
      is_reading_a := true.B
      when (a_read_counter === READ_A_LATENCY) {
        cmd.pop := 1.U

        is_mul_tag_finished := true.B
        io.completed.valid := true.B
        io.completed.bits  := cmd.bits(0).rob_id

        state_n := s_IDLE
      }
    }
  }

  //------------------------------------------------------------------------
  // write next command for meshq input datapath to handle
  // - the meshq input datapath reads from this queue 1+ cycle after it
  //   is written to, dependening on how long the scratchpad latency is
  //------------------------------------------------------------------------
  val cntlq_in = mesh_cntl_signals_q.io.enq

  cntlq_in.valid := state === s_PRELOAD ||
                    state === s_MUL_PRE ||
                    state === s_MUL

  cntlq_in.bits.do_mul_pre        := (state === s_MUL_PRE)
  cntlq_in.bits.do_single_mul     := (state === s_MUL)
  cntlq_in.bits.do_single_preload := (state === s_PRELOAD)

  cntlq_in.bits.a_cols          := a_cols
  cntlq_in.bits.a_rows          := a_rows
  cntlq_in.bits.a_bank_start    := a_bank_start
  cntlq_in.bits.a_bank_end      := a_bank_end
  cntlq_in.bits.a_total_banks   := a_total_banks
  cntlq_in.bits.a_row_end       := a_row_end
  cntlq_in.bits.a_col_start     := a_col_start
  cntlq_in.bits.a_fg_col_start  := a_fg_col_start
  cntlq_in.bits.a_row_start     := a_row_start
  cntlq_in.bits.a_row_end       := a_row_end

  cntlq_in.bits.b_cols          := b_cols
  cntlq_in.bits.b_rows          := b_rows
  cntlq_in.bits.b_bank_start    := b_bank_start
  cntlq_in.bits.b_bank_end      := b_bank_end
  cntlq_in.bits.b_total_banks   := b_total_banks
  cntlq_in.bits.b_row_end       := b_row_end
  cntlq_in.bits.b_col_start     := b_col_start
  cntlq_in.bits.b_fg_col_start  := b_fg_col_start
  cntlq_in.bits.b_row_start     := b_row_start
  cntlq_in.bits.b_row_end       := b_row_end

  cntlq_in.bits.c_cols          := c_cols
  cntlq_in.bits.c_rows          := c_rows
  cntlq_in.bits.c_bank_start    := c_bank_start
  cntlq_in.bits.c_bank_end      := c_bank_end
  cntlq_in.bits.c_total_banks   := c_total_banks
  cntlq_in.bits.c_row_end       := c_row_end
  cntlq_in.bits.c_col_start     := c_col_start
  cntlq_in.bits.c_fg_col_start  := c_fg_col_start
  cntlq_in.bits.c_row_start     := c_row_start
  cntlq_in.bits.c_row_end       := c_row_end

  cntlq_in.bits.c_garbage       := c_garbage

  cntlq_in.bits.accum := is_accum

  //TODO fix this
  cntlq_in.bits.a_garbage := multiply_garbage || !is_inputting_a
  cntlq_in.bits.b_garbage := preload_zeros || !is_inputting_d

  cntlq_in.bits.preload_zeros := preload_zeros


  cntlq_in.bits.rob_id.valid := !c_garbage &&
                                (state === s_MUL_PRE || state === s_PRELOAD)
  cntlq_in.bits.rob_id.bits  := cmd.bits(preload_cmd_place).rob_id
  cntlq_in.bits.prop         := in_prop

  //TODO should occur at the same time so one is sufficient?
  cntlq_in.bits.last_row     := b_read_counter === READ_B_LATENCY || a_read_counter === READ_A_LATENCY

  //========================================================================
  // Instantiate the actual mesh
  //========================================================================
  //val mesh = Module(new MeshWithDelays2[T](config))

  val mesh = Module(new FgMesh[T](config))

  mesh.io.tag_in.valid := false.B
  mesh.io.tag_in.bits  := DontCare
  mesh.io.a.valid      := false.B
  mesh.io.b.valid      := false.B
  mesh.io.a.bits       := DontCare
  mesh.io.b.bits       := DontCare
  mesh.io.prof         := io.prof

  // busy if the mesh has pending tags
  io.busy := mesh.io.busy

  //=========================================================================
  // write inputs into mesh datapath (read from scratchpad/accumulator)
  //=========================================================================
  val cntlq = mesh_cntl_signals_q.io.deq.bits
  val cntlq_out_ready = mesh_cntl_signals_q.io.deq.ready
  val cntlq_out_valid = mesh_cntl_signals_q.io.deq.valid

  mesh.io.a.valid := ShiftRegister(a_read_en, 2) // 2 cycle read latency
  mesh.io.a.bits := a_mesh_input

  mesh.io.b.valid := ShiftRegister(b_read_en, 2) // 2 cycle read latency
  mesh.io.b.bits := b_mesh_input

  // write the PE-control signals
  mesh.io.pe_ctrl.propagate  := cntlq.prop

  //TODO check logic here
  // write tag_in into mesh only on last row written. ALL compute rob_ids are
  // written to the mesh, even if they have garbage output addrs
  mesh.io.tag_in.valid       := cntlq.last_row && cntlq_out_ready
  mesh.io.tag_in.bits.rob_id := cntlq.rob_id
  //TODO garbage addr
  //mesh.io.tag_in.bits.addr   := Mux(cntlq.do_single_mul,
  //                                  GARBAGE_ADDR.asTypeOf(local_addr_t),
  //                                  cntlq.c_addr)
  mesh.io.tag_in.bits.cols          := cntlq.c_cols
  mesh.io.tag_in.bits.rows          := cntlq.c_rows
  mesh.io.tag_in.bits.fg_col_start  := cntlq.fg_col_start
  mesh.io.tag_in.bits.bank_start    := cntlq.bank_start
  mesh.io.tag_in.bits.banks         := cntlq.banks
  mesh.io.tag_in.bits.accum         := cntlq.accum
  mesh.io.tag_in.bits.garbage       := cntlq.c_garbage


  when (mesh.io.tag_in.valid) {
    assert(mesh.io.tag_in.fire(), "could not write tag_in to mesh!")
  }

  mesh.io.a_mux_ctrl := a_fg_arrange
  mesh.io.b_mux_ctrl := b_fg_arrange

  //TODO check this operation (just write every cycle in this case)
  cntlq_out.ready := state === s_PRELOAD ||
                    state === s_MUL_PRE ||
                    state === s_MUL
 // cntlq_out_ready :=
 //   (!cntlq.a_fire || mesh.io.a.fire() || !mesh.io.a.ready) &&
 //   (!cntlq.d_fire || mesh.io.d.fire() || !mesh.io.d.ready)

  //=========================================================================
  // Mesh->Scratchpad/Accumulator write datapath
  //=========================================================================
  val w_matrix_rows     = mesh.io.tag_out.rows
  val w_matrix_cols     = mesh.io.tag_out.cols
  val w_fg_col_start    = mesh.io.tag_out.fg_col_start
  val w_bank_start      = mesh.io.tag_out.bank_start
  val w_banks           = mesh.io.tag_out.banks
  val w_accum           = mesh.io.tag_out.accum
  val w_garbage         = mesh.io.tag_out.garbage

  val is_array_outputting = mesh.io.out.fire() &&
                            !w_garbage &&
                            mesh.io.tag_out.rob_id.valid

  // TODO: make this finish after w_matrix_rows
  // TODO off by 1?
  val output_row = RegInit(0.U(log2Up(FG_DIM+1).W))
  output_row := wrappingAdd(output_row, is_array_outputting, FG_DIM)

  //TODO fix this
  val is_array_outputting_valid_row = is_array_outputting &&
                                      (output_row < FG_DIM)
  val is_array_outputting_last_row = is_array_outputting &&
                                     (output_row === (FG_DIM-1.U))

  //TODO check valid row
  io.memWriteC.en := is_array_outputting_valid_row
  io.memWriteC.row := output_row
  io.memWriteC.cols := w_matrix_cols
  io.memWriteC.fg_col_start := w_fg_col_start
  io.memWriteC.bank_start := w_bank_start
  io.memWriteC.banks := w_banks
  io.memWriteC.accum := w_accum

  //-------------------------------------------------------------------------
  // commit the pending preload tag on the last output row
  //-------------------------------------------------------------------------
  val pending_preload_tag_complete
    = RegInit(0.U.asTypeOf(UDValid(UInt(log2Up(rob_entries).W))))

  when(is_array_outputting_last_row) {
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

  //-------------------------------------------------------------------------
  // Write to normal scratchpad
  //-------------------------------------------------------------------------
  //val activated_wdata = VecInit(mesh.io.out.bits.map(v => VecInit(v.map {e =>
  //  val e_clipped = e.clippedToWidthOf(inputType)
  //  val e_act = MuxCase(e_clipped, Seq(
  //    (activation === Activation.RELU) -> e_clipped.relu,
  //    (activation === Activation.RELU6) -> e_clipped.relu6(relu6_shift)))
  //  e_act
  //})))

  //TODO add ACC_CONFIG connections

  //TODO fix type and deal with activation
  io.memWriteC.data := mesh.io.out.bits

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
    (config: GemminiArrayConfig[T])(implicit p: Parameters)
      = Module(new FgExecuteController(config))
}
