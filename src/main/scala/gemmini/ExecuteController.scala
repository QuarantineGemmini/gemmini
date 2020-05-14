//============================================================================
// Fine-grained-systolic-array exec-controller
//============================================================================
package gemmini

import scala.math.{pow}
import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import GemminiISA._
import Util._

class ExecuteController[T <: Data](config: GemminiArrayConfig[T])
  (implicit val p: Parameters, ev: Arithmetic[T])
  extends Module with HasCoreParameters {
  import config._
  import ev._
  //=========================================================================
  // I/O interface
  //=========================================================================
  val io = IO(new Bundle {
    val cmd        = Flipped(Decoupled(new GemminiCmd(ROB_ENTRIES_IDX)))
    val read       = Vec(2, new MemUnitExecReadIO(config)))
    val write      = new MemUnitExecWriteReq(config))
    val acc_config = new AccumulatorBankConfigIO(config)
    val completed  = Valid(UInt(ROB_ENTRIES_IDX.W))
    val busy       = Output(Bool())
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

  val functs = VecInit(cmd.bits.map(_.cmd.inst.funct))
  val rs1s   = VecInit(cmd.bits.map(_.cmd.rs1))
  val rs2s   = VecInit(cmd.bits.map(_.cmd.rs2))

  val is_config_cmd  = cmd.valid(0) && (functs(0) === CONFIG_CMD)
  val is_compute_cmd = cmd.valid(0) && (functs(0) === COMPUTE_AND_FLIP_CMD ||
                                        functs(0) === COMPUTE_AND_STAY_CMD)
  val preload_idx    = Mux(is_compute_cmd, 1.U, 0.U)
  val is_preload_cmd = cmd.valid(preload_idx) &&
                       (functs(preload_idx) === PRELOAD_CMD)

  val preload_rs1 = rs1s(preload_idx).asTypeOf(new LocalRange(config))
  val preload_rs2 = rs2s(preload_idx).asTypeOf(new LocalRange(config))
  val compute_rs1 = rs1s(0).asTypeOf(new LocalRange(config))
  val compute_rs2 = rs2s(0).asTypeOf(new LocalRange(config))

  val in_flipped = functs(0) === COMPUTE_AND_FLIP_CMD

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

  io.acc_config.acc_rshift   := acc_shift
  io.acc_config.relu6_lshift := relu6_shift
  io.acc_config.act          := activation

  //=========================================================================
  // fix-latency scratchpad-inputs read
  //=========================================================================
  val sp_read_counter = RegInit(0.U(DIM_IDX.W))

  // deal with bank conflicts
  val bank_conflict_sel_a = RegInit(true.B)
  val bank_conflict = (sp_read_counter < a_nonzero_cycles) && 
                      is_reading_a && 
                      is_reading_b &&
                      a_lrange.conflicts(b_lrange)
  val a_data_fire = Mux(bank_conflict,  bank_conflict_sel_a, true.B)
  val b_data_fire = Mux(bank_conflict, ~bank_conflict_sel_a, true.B)

  bank_conflict_sel_a := Mux(bank_conflict, ~bank_conflict_sel_a, true.B)

  //----------------------------------------
  // a_data input
  //----------------------------------------
  val is_reading_a = WireInit(false.B)
  val a_read_en    = (sp_read_counter < a_lrange.rows) && 
                     is_reading_a && a_data_fire

  io.read(0).req.en   := a_read_en
  io.read(0).req.row  := sp_read_counter
  io.read(0).req.cols := a_lrange.cols

  // get data back 2 cycles later
  val conflict_buf1  = ShiftRegister(bank_conflict, SP_RD_LATENCY+1)
  val conflict_buf   = ShiftRegister(bank_conflict, SP_RD_LATENCY)
  val a_read_en_buf  = ShiftRegister(a_read_en, SP_RD_LATENCY)

  // possibly buffer 1 extra cycle if b_data had a bank-conflict
  val a_zero_data = WireInit(0.asTypeOf(Vec(DIM, inputType)))
  val a_data      = Mux(a_read_en_buf, 
                        io.read(0).resp.data.asTypeOf(Vec(DIM, inputType)),
                        a_zero_data)
  val a_data_buf  = RegNext(a_data)
  val a_data_out  = Mux(conflict_buf1, a_data_buf, 
                     Mux(conflict_buf, a_zero_data, a_data))

  //----------------------------------------
  // b_data input
  //----------------------------------------
  val is_reading_b = WireInit(false.B)
  val b_read_en    = (sp_read_counter < b_lrange.rows) && 
                     is_reading_b && b_data_fire

  io.read(1).req.en   := b_read_en
  io.read(1).req.row  := sp_read_counter
  io.read(1).req.cols := b_lrange.cols

  // get data back 2 cycles later
  val b_data_fire_buf = ShiftRegister(b_data_fire, SP_RD_LATENCY)
  val b_read_en_buf   = ShiftRegister(b_read_en, SP_RD_LATENCY)

  val b_zero_data = WireInit(0.asTypeOf(Vec(DIM, inputType)))
  val b_data      = Mux(b_read_en_buf, 
                        io.read(1).resp.data.asTypeOf(Vec(DIM, inputType)),
                        b_zero_data)

  //=========================================================================
  // global FSM
  //=========================================================================
  val (s_IDLE :: s_PRELOAD :: s_MUL :: s_MUL_PRE :: Nil) = Enum(4)
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

      when (sp_read_counter === (DIM-1).U) {
        cmd.pop         := 1.U
        sp_read_counter := 0.U
        state           := s_IDLE
      }
    }
    is (s_MUL_PRE) {
      assert(!a_lrange.garbage, "a cannot be garbage")
      is_reading_a    := true.B
      is_reading_b    := !b_lrange.garbage
      sp_read_counter := sp_read_counter + b_data_fire

      when (sp_read_counter === (DIM-1).U) {
        cmd.pop             := 2.U
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

      when (sp_read_counter === (DIM-1).U) {
        cmd.pop             := 1.U
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
    val in_valid      = Bool()
    val has_preload   = Bool()
    val rob_id        = UInt(ROB_ENTRIES_IDX.W)
    val c_lrange      = new LocalRange(config)
    val flipped       = Bool()
    val last_row      = Bool()
  }
  val mesh_ctrl = Wire(new ComputeCntrlSignals)
  val mesh_ctrl_buf = ShiftRegister(mesh_ctrl, SP_RD_LATENCY)

  //------------------------------------------------------------------------
  // write next command for meshq input datapath to handle
  //------------------------------------------------------------------------
  mesh_ctrl.in_valid    := (state === s_PRELOAD || state === s_MUL) ||
                           (state === s_MUL_PRE && b_data_fire_buf)
  mesh_ctrl.has_preload := (state === s_PRELOAD) || (state === s_MUL_PRE)
  mesh_ctrl.rob_id      := cmd.bits(preload_idx).rob_id
  mesh_ctrl.c_lrange    := c_lrange
  mesh_ctrl.flipped     := in_flipped && (sp_read_counter === 0.U)
  mesh_ctrl.last_row    := sp_read_counter === (DIM-1).U

  //========================================================================
  // Instantiate the actual mesh and connect non-blocking inputs
  //========================================================================
  val mesh = Module(new Mesh(config))
  mesh.io.in_valid                 := mesh_ctrl_buf.in_valid
  mesh.io.a                        := a_data_out
  mesh.io.b                        := b_data
  mesh.io.flipped                  := mesh_ctrl_buf.flipped
  mesh.io.tag_in.valid             := mesh_ctrl_buf.last_row
  mesh.io.tag_in.bits.do_writeback := mesh_ctrl_buf.has_preload
  mesh.io.tag_in.bits.rob_id       := mesh_ctrl_buf.rob_id
  mesh.io.tag_in.bits.wb_lrange    := mesh_ctrl_buf.c_lrange

  //=========================================================================
  // Mesh->Scratchpad/Accumulator write datapath
  //=========================================================================
  val wb_valid              = mesh.io.out_valid
  val wb_do_writeback       = mesh.io.tag_out.bits.do_writeback
  val wb_data               = mesh.io.out
  val wb_rob_id             = mesh.io.tag_out.bits.rob_id
  val wb_lrange             = mesh.io.tag_out.bits.wb_lrange
  val wb_garbage            = wb_lrange.garbage
  val wb_is_outputting      = wb_valid && wb_do_writeback && !wb_garbage
  val wb_is_outputting_last = wb_is_outputting && mesh.io.tag_out.commit
  assert(!(wb_is_outputting && wb_garbage))

  io.write.en    := wb_is_outputting
  io.write.row   := wb_lrange.row_start
  io.write.cols  := wb_lrange.cols
  io.write.accum := wb_lrange.accum
  io.write.data  := wb_data.asTypeOf(UInt(ACC_ROW_BITS.W))

  //-------------------------------------------------------------------------
  // commit the pending preload tag on the last output row
  //-------------------------------------------------------------------------
  val pending_preload_tag_complete
    = RegInit(0.U.asTypeOf(UDValid(UInt(ROB_ENTRIES_IDX.W))))

  when(wb_is_outputting_last) {
    assert(!pending_preload_tag_complete.valid,
      "can't output last row when we have an existing pending tag!")
    when (is_mul_tag_finished) {
      pending_preload_tag_complete.valid := true.B
      pending_preload_tag_complete.bits  := wb_rob_id
    } .otherwise {
      io.completed.valid := true.B
      io.completed.bits  := wb_rob_id
    }
  }

  when (pending_preload_tag_complete.valid && ~is_mul_tag_finished) {
    io.completed.valid := true.B
    io.completed.bits  := pending_preload_tag_complete.bits
    pending_preload_tag_complete.valid := false.B
  }

  //=========================================================================
  // busy calculation
  //=========================================================================
  io.busy := (state =/= s_IDLE) || mesh.io.busy
}

object ExecuteController {
  def apply[T <: Data: Arithmetic]
    (config: GemminiArrayConfig[T])(implicit p: Parameters)
      = Module(new ExecuteController(config))
}
