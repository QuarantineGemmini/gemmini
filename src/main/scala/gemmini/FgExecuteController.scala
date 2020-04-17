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
  val DIM = meshRows*tileRows
  // TODO for dgrubb: use this parameter
  val TOTAL_MESHES = fs_sa_div * fa_sa_div

  val A_TYPE = Vec(meshRows, Vec(tileRows, inputType))
  val D_TYPE = Vec(meshCols, Vec(tileCols, inputType))

  val io = IO(new Bundle {
    val cmd = Flipped(Decoupled(new GemminiCmd(rob_entries)))
    val srams = new Bundle {
      val read = Vec(sp_banks, new ScratchpadReadIO(sp_bank_entries, 
                                                    sp_width))
      val write = Vec(sp_banks, new ScratchpadWriteIO(sp_bank_entries, 
                              sp_width, (sp_width / (aligned_to * 8)) max 1))
    }

    val acc = new Bundle {
      val read = Vec(acc_banks, 
        new AccumulatorReadIO(acc_bank_entries, log2Up(accType.getWidth), 
                              Vec(meshColumns, Vec(tileColumns, inputType))))
      val write = Vec(acc_banks, 
        new AccumulatorWriteIO(acc_bank_entries, 
                               Vec(meshColumns, Vec(tileColumns, accType))))
    }
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

  val DoConfig = functs(0) === CONFIG_CMD
  val DoComputes = functs.map(f => f === COMPUTE_AND_FLIP_CMD || 
                                   f === COMPUTE_AND_STAY_CMD)
  val DoPreloads = functs.map(_ === PRELOAD_CMD)

  val preload_cmd_place = Mux(DoPreloads(0), 0.U, 1.U)

  val in_prop = functs(0) === COMPUTE_AND_FLIP_CMD

  // configuration state
  val acc_shift   = Reg(UInt(log2Up(accType.getWidth).W))
  val relu6_shift = Reg(UInt(log2Up(accType.getWidth).W))
  val activation  = Reg(UInt(2.W))

  // SRAM addresses of matmul operands
  val a_address_rs1 = rs1s(0).asTypeOf(local_addr_t)
  val d_address_rs1 = rs1s(preload_cmd_place).asTypeOf(local_addr_t)
  val c_address_rs2 = rs2s(preload_cmd_place).asTypeOf(local_addr_t)

  val multiply_garbage = a_address_rs1.is_garbage()
  val preload_zeros    = d_address_rs1.is_garbage()

  val a_cols = rs1s(0)(47, 32)
  val a_rows = rs1s(0)(63, 48)
  val d_cols = rs1s(preload_cmd_place)(47, 32)
  val d_rows = rs1s(preload_cmd_place)(63, 48)
  val c_cols = rs2s(preload_cmd_place)(47, 32)
  val c_rows = rs2s(preload_cmd_place)(63, 48)

  //=========================================================================
  // Queue between the frontend FSM and the backend datapath
  // - the frontend FSM schedules input reads to scratchpad/accumulator and
  //   puts the corresponding mesh command in the queue.
  // - then, the backend datapath reads from the queue after a fixed number of
  //   cycles, when the scratchpad/accumulator inputs are valid
  //=========================================================================
  class ComputeCntlSignals extends Bundle {
    val do_mul_pre        = Bool()
    val do_single_mul     = Bool()
    val do_single_preload = Bool()

    val a_bank = UInt(log2Up(sp_banks).W)
    val d_bank = UInt(log2Up(sp_banks).W)

    val a_bank_acc = UInt(log2Up(acc_banks).W)
    val d_bank_acc = UInt(log2Up(acc_banks).W)

    val a_read_from_acc = Bool()
    val d_read_from_acc = Bool()

    val a_garbage = Bool()
    val d_garbage = Bool()

    val preload_zeros = Bool()

    val a_fire = Bool()
    val d_fire = Bool()

    val a_unpadded_cols = UInt(log2Up(DIM + 1).W)
    val d_unpadded_cols = UInt(log2Up(DIM + 1).W)

    val c_addr = local_addr_t.cloneType
    val c_rows = UInt(log2Up(DIM + 1).W)
    val c_cols = UInt(log2Up(DIM + 1).W)

    val rob_id = UDValid(UInt(log2Up(rob_entries).W))
    val prop = UInt(1.W)
    val last_row = Bool()
  }

  val mesh_cntl_signals_q = Module(new Queue(
                          new ComputeCntlSignals, mem_pipeline+1, pipe=true))
  val cntl_ready = mesh_cntl_signals_q.io.enq.ready

  //========================================================================
  // SRAM scratchpad read dependency/hazards
  //========================================================================
  val dataAbank = a_address_rs1.sp_bank()
  val dataDbank = d_address_rs1.sp_bank()

  val dataABankAcc = a_address_rs1.acc_bank()
  val dataDBankAcc = d_address_rs1.acc_bank()

  val a_read_from_acc = a_address_rs1.is_acc_addr
  val d_read_from_acc = d_address_rs1.is_acc_addr

  //wires determined by state
  val is_inputting_a = WireInit(false.B)
  val is_inputting_d = WireInit(false.B)

  // Fire counters which resolve same-bank accesses
  val a_fire_counter = Reg(UInt(log2Up(DIM).W))
  val d_fire_counter = Reg(UInt(log2Up(DIM).W))

  // These variables determine whether or not the row that is 
  // currently being read should be completely padded with 0
  val a_row_is_not_all_zeros = a_fire_counter < a_rows
  val d_row_is_not_all_zeros = DIM.U - 1.U - d_fire_counter < d_rows

  def same_bank(addr1: LocalAddr, addr2: LocalAddr, 
                is_inputting1: Bool, is_inputting2: Bool): Bool = {
    val addr1_read_from_acc = addr1.is_acc_addr
    val addr2_read_from_acc = addr2.is_acc_addr

    val is_garbage = addr1.is_garbage() || addr2.is_garbage() ||
                     !is_inputting1     || !is_inputting2

    !is_garbage && (
      (addr1_read_from_acc && addr2_read_from_acc) ||
      (!addr1_read_from_acc && !addr2_read_from_acc && 
        addr1.sp_bank() === addr2.sp_bank()))
  }

  val a_ready = WireInit(true.B)
  val d_ready = WireInit(true.B)

  case class Operand(addr: LocalAddr, is_inputting: Bool, 
                     counter: UInt, priority: Int)
  val a_operand = Operand(a_address_rs1, is_inputting_a, a_fire_counter, 0)
  val d_operand = Operand(d_address_rs1, is_inputting_d, d_fire_counter, 1)
  val operands  = Seq(a_operand, d_operand)

  // these are 'valid' if they are garbage addrs
  val Seq(a_valid, d_valid) = operands.map { 
    case Operand(addr, is_inputting, counter, priority) =>
      val others = operands.filter(_.priority != priority)

      val same_banks = others.map(o => same_bank(addr, o.addr, 
                                        is_inputting, o.is_inputting))
      val same_counter = others.map(o => counter === o.counter)
      val one_ahead = others.map(
                        o => counter === wrappingAdd(o.counter, 1.U, DIM))

      val higher_priorities = others.map(o => (o.priority < priority).B)

      val must_wait_for = ((same_banks zip same_counter) zip 
                           (one_ahead zip higher_priorities)).map {
        case ((sb, sc), (oa, hp)) => (sb && hp && sc) || oa
      }
      !must_wait_for.reduce(_ || _)
  }

  val a_fire = a_valid && a_ready
  val d_fire = d_valid && d_ready

  val firing = is_inputting_a || is_inputting_d

  when (!firing) {
    a_fire_counter := 0.U
  }.elsewhen (firing && a_fire && cntl_ready) {
    a_fire_counter := wrappingAdd(a_fire_counter, 1.U, DIM)
  }

  when (!firing) {
    d_fire_counter := 0.U
  }.elsewhen (firing && d_fire && cntl_ready) {
    d_fire_counter := wrappingAdd(d_fire_counter, 1.U, DIM)
  }

  val about_to_fire_all_rows = 
    ((a_fire_counter === (DIM-1).U && a_valid) || a_fire_counter === 0.U) &&
    ((d_fire_counter === (DIM-1).U && d_valid) || d_fire_counter === 0.U) &&
    ((a_fire_counter | d_fire_counter) =/= 0.U) &&
    cntl_ready

  //========================================================================
  // Scratchpad reads
  //========================================================================
  for (i <- 0 until sp_banks) {
    val read_a = a_valid && 
                 !a_read_from_acc && 
                 dataAbank === i.U && 
                 is_inputting_a && 
                 !multiply_garbage && 
                 a_row_is_not_all_zeros

    val read_d = d_valid && 
                 !d_read_from_acc && 
                 dataDbank === i.U && 
                 is_inputting_d && 
                 !preload_zeros && 
                 d_row_is_not_all_zeros

    Seq((read_a, a_ready), (read_d, d_ready)).foreach { case (rd, r) =>
      when (rd && !io.srams.read(i).req.ready) {
        r := false.B
      }
    }

    io.srams.read(i).req.valid := read_a || read_d
    io.srams.read(i).req.bits.fromDMA := false.B
    io.srams.read(i).req.bits.addr := Mux(read_d,
      d_address_rs1.sp_row() + DIM.U - 1.U - d_fire_counter,
      a_address_rs1.sp_row() + a_fire_counter)
    io.srams.read(i).resp.ready := true.B
  }

  //========================================================================
  // Accumulator read
  //========================================================================
  for (i <- 0 until acc_banks) {
    val read_a_from_acc = a_valid && 
                          a_read_from_acc && 
                          dataABankAcc === i.U && 
                          is_inputting_a && 
                          !multiply_garbage && 
                          a_row_is_not_all_zeros

    val read_d_from_acc = d_valid && 
                          d_read_from_acc && 
                          dataDBankAcc === i.U && 
                          is_inputting_d && 
                          !preload_zeros && 
                          d_row_is_not_all_zeros

    Seq((read_a_from_acc, a_ready), (read_d_from_acc, d_ready)).foreach { 
      case (rd, r) =>
        when(rd && !io.acc.read(i).req.ready) {
          r := false.B
        }
    }

    io.acc.read(i).req.valid := read_a_from_acc || read_d_from_acc
    io.acc.read(i).req.bits.shift := acc_shift
    io.acc.read(i).req.bits.relu6_shift := relu6_shift
    io.acc.read(i).req.bits.act := activation
    io.acc.read(i).req.bits.fromDMA := false.B

    io.acc.read(i).req.bits.addr := Mux(read_d_from_acc,
      d_address_rs1.acc_row() + DIM.U - 1.U - d_fire_counter,
      a_address_rs1.acc_row() + a_fire_counter)

    io.acc.read(i).resp.ready := true.B
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
      is_inputting_d := true.B
      when (about_to_fire_all_rows) {
        cmd.pop := 1.U
        state_n := s_IDLE
      }
    }
    is (s_MUL_PRE) {
      is_inputting_a := true.B
      is_inputting_d := true.B
      when (about_to_fire_all_rows) {
        cmd.pop := 2.U

        is_mul_tag_finished := true.B
        io.completed.valid := true.B
        io.completed.bits  := cmd.bits(0).rob_id

        state_n := s_IDLE
      }
    }
    is (s_MUL) {
      is_inputting_a := true.B
      when (about_to_fire_all_rows) {
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

  cntlq_in.bits.a_bank := dataAbank
  cntlq_in.bits.d_bank := dataDbank

  cntlq_in.bits.a_bank_acc := dataABankAcc
  cntlq_in.bits.d_bank_acc := dataDBankAcc

  cntlq_in.bits.a_garbage := a_address_rs1.is_garbage() || !is_inputting_a
  cntlq_in.bits.d_garbage := d_address_rs1.is_garbage() || !is_inputting_d

  cntlq_in.bits.a_read_from_acc := a_read_from_acc
  cntlq_in.bits.d_read_from_acc := d_read_from_acc

  cntlq_in.bits.preload_zeros := preload_zeros

  cntlq_in.bits.a_unpadded_cols := Mux(a_row_is_not_all_zeros, a_cols, 0.U)
  cntlq_in.bits.d_unpadded_cols := Mux(d_row_is_not_all_zeros, d_cols, 0.U)

  cntlq_in.bits.a_fire := a_fire
  cntlq_in.bits.d_fire := d_fire

  cntlq_in.bits.c_addr := c_address_rs2
  cntlq_in.bits.c_rows := c_rows
  cntlq_in.bits.c_cols := c_cols

  cntlq_in.bits.rob_id.valid := !c_address_rs2.is_garbage() &&
                                (state === s_MUL_PRE || state === s_PRELOAD)
  cntlq_in.bits.rob_id.bits  := cmd.bits(preload_cmd_place).rob_id
  cntlq_in.bits.prop         := in_prop
  cntlq_in.bits.last_row     := about_to_fire_all_rows

  //========================================================================
  // Instantiate the actual mesh
  //========================================================================
  val mesh = Module(new MeshWithDelays[T](config))

  mesh.io.tag_in.valid := false.B
  mesh.io.tag_in.bits  := DontCare
  mesh.io.a.valid      := false.B
  mesh.io.d.valid      := false.B
  mesh.io.a.bits       := DontCare
  mesh.io.d.bits       := DontCare
  mesh.io.prof         := io.prof

  // busy if the mesh has pending tags
  io.busy := mesh.io.busy

  //=========================================================================
  // write inputs into mesh datapath (read from scratchpad/accumulator)
  //=========================================================================
  val cntlq = mesh_cntl_signals_q.io.deq.bits
  val cntlq_out_ready = mesh_cntl_signals_q.io.deq.ready
  val cntlq_out_valid = mesh_cntl_signals_q.io.deq.valid

  val readData = VecInit(io.srams.read.map(_.resp.bits.data))
  val accReadData = VecInit(io.acc.read.map(_.resp.bits.data.asUInt()))

  val readValid = VecInit(io.srams.read.map(
                    bank => bank.resp.valid && !bank.resp.bits.fromDMA))
  val accReadValid = VecInit(io.acc.read.map(
                      bank => bank.resp.valid && !bank.resp.bits.fromDMA))

  val dataA_valid = cntlq.a_garbage || 
                    cntlq.a_unpadded_cols === 0.U || 
                    Mux(cntlq.a_read_from_acc, 
                      accReadValid(cntlq.a_bank_acc), 
                      readValid(cntlq.a_bank))

  val dataD_valid = cntlq.d_garbage || 
                    cntlq.d_unpadded_cols === 0.U || 
                    MuxCase(readValid(cntlq.d_bank), Seq(
                      cntlq.preload_zeros -> false.B,
                      cntlq.d_read_from_acc -> accReadValid(cntlq.d_bank_acc)
                    ))

  // data read from scratchpad/accumulator
  val dataA_unpadded = Mux(cntlq.a_read_from_acc, 
                        accReadData(cntlq.a_bank_acc), 
                        readData(cntlq.a_bank))

  val dataD_unpadded = MuxCase(readData(cntlq.d_bank), Seq(
                       cntlq.preload_zeros -> 0.U, 
                       cntlq.d_read_from_acc -> accReadData(cntlq.d_bank_acc)
                       ))

  // add zeros if zero-padding is needed
  val dataA = VecInit(dataA_unpadded.asTypeOf(
                Vec(DIM, inputType)).zipWithIndex.map { case (d, i) => 
                  Mux(i.U < cntlq.a_unpadded_cols, d, inputType.zero)})

  val dataD = VecInit(dataD_unpadded.asTypeOf(
                Vec(DIM, inputType)).zipWithIndex.map { case (d, i) => 
                  Mux(i.U < cntlq.d_unpadded_cols, d, inputType.zero)})

  // write the PE-control signals
  mesh.io.pe_ctrl.propagate  := cntlq.prop

  // write tag_in into mesh only on last row written. ALL compute rob_ids are
  // written to the mesh, even if they have garbage output addrs
  mesh.io.tag_in.valid       := cntlq.last_row && cntlq_out_ready
  mesh.io.tag_in.bits.rob_id := cntlq.rob_id
  mesh.io.tag_in.bits.addr   := Mux(cntlq.do_single_mul,
                                    GARBAGE_ADDR.asTypeOf(local_addr_t),
                                    cntlq.c_addr)
  mesh.io.tag_in.bits.cols   := cntlq.c_cols
  mesh.io.tag_in.bits.rows   := cntlq.c_rows

  when (mesh.io.tag_in.valid) {
    assert(mesh.io.tag_in.fire(), "could not write tag_in to mesh!")
  }

  // write the A/D inputs to the mesh. 
  when (cntlq_out_valid) {
    mesh.io.a.valid := cntlq.a_fire && dataA_valid
    mesh.io.d.valid := cntlq.d_fire && dataD_valid
  }
  mesh.io.a.bits := dataA.asTypeOf(A_TYPE)
  mesh.io.d.bits := dataD.asTypeOf(D_TYPE)

  cntlq_out_ready := 
    (!cntlq.a_fire || mesh.io.a.fire() || !mesh.io.a.ready) &&
    (!cntlq.d_fire || mesh.io.d.fire() || !mesh.io.d.ready)

  //=========================================================================
  // Mesh->Scratchpad/Accumulator write datapath
  //=========================================================================
  val w_matrix_rows     = mesh.io.tag_out.rows
  val w_matrix_cols     = mesh.io.tag_out.cols

  val w_address         = mesh.io.tag_out.addr
  val is_output_garbage = w_address.is_garbage()
  val write_to_acc      = w_address.is_acc_addr

  val is_array_outputting = mesh.io.out.fire() && 
                            !is_output_garbage &&
                            mesh.io.tag_out.rob_id.valid 

  // TODO: make this finish after w_matrix_rows
  // TODO: allow this to stall the mesh output (use Decoupled on mesh.io.out)
  val output_row = RegInit(0.U(log2Up(DIM+1).W))
  output_row := wrappingAdd(output_row, is_array_outputting, DIM)

  val is_array_outputting_valid_row = is_array_outputting && 
                                      (output_row < w_matrix_rows)
  val is_array_outputting_last_row = is_array_outputting && 
                                     (output_row === (w_matrix_rows-1.U))

  val w_bank = Mux(write_to_acc, w_address.acc_bank(), w_address.sp_bank())
  val w_row  = Mux(write_to_acc, w_address.acc_row(),  w_address.sp_row())
  val current_w_bank_address = w_row + output_row

  // This is an element-wise mask, rather than a byte-wise mask
  val w_mask = (0 until DIM).map(_.U < w_matrix_cols)

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
  val activated_wdata = VecInit(mesh.io.out.bits.map(v => VecInit(v.map {e =>
    val e_clipped = e.clippedToWidthOf(inputType)
    val e_act = MuxCase(e_clipped, Seq(
      (activation === Activation.RELU) -> e_clipped.relu,
      (activation === Activation.RELU6) -> e_clipped.relu6(relu6_shift)))
    e_act
  })))

  for(i <- 0 until sp_banks) {
    io.srams.write(i).en := is_array_outputting_valid_row && 
                            w_bank === i.U && 
                            !write_to_acc && 
                            !is_output_garbage
    io.srams.write(i).addr := current_w_bank_address
    io.srams.write(i).data := activated_wdata.asUInt()
    io.srams.write(i).mask := w_mask.flatMap(b => 
                          Seq.fill(inputType.getWidth / (aligned_to * 8))(b))
  }

  //-------------------------------------------------------------------------
  // Write to accumulator
  //-------------------------------------------------------------------------
  for (i <- 0 until acc_banks) {
    io.acc.write(i).en := is_array_outputting_valid_row && 
                          w_bank === i.U && 
                          write_to_acc && 
                          !is_output_garbage
    io.acc.write(i).addr := current_w_bank_address
    io.acc.write(i).data := mesh.io.out.bits
    io.acc.write(i).acc := w_address.accumulate
    io.acc.write(i).mask := w_mask.flatMap(b => 
                            Seq.fill(accType.getWidth / (aligned_to * 8))(b))
  }

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

object ExecuteController {
  def apply[T <: Data: Arithmetic]
    (config: GemminiArrayConfig[T])(implicit p: Parameters)
      = Module(new ExecuteController(config))
}
