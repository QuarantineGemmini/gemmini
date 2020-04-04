package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._

//===========================================================================
// TilerController's Internal FSM implementation
//===========================================================================
class TilerFSM(config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends Module with HasCoreParameters
{
  import config._
  //=========================================================================
  // interface
  //=========================================================================
  val io = IO(new Bundle {
    val cmd_in = Flipped(Decoupled(new TilerCmd))
    val sched_out = Decoupled(new RoCCCommand)
    val busy = Output(Bool())
  }
  val cmd = io.cmd_in
  // hardcode 8 entries with up to 2 pushes per cycle
  val (sched, _) = MultiTailedQueue(io.sched_out, 8, 2);
  val busy = io.busy

  //=========================================================================
  // FSM states (see diagram for what each state does)
  //=========================================================================
  val s_IDLE ::
      //-----------------
      s_RESET_OUTPUT_GROUP ::
      s_RESET_A_TILE_SUBCOL ::
      s_MOVE_FIRST_B_INTO_SP ::
      s_RESET_B_TILE_SUBCOL_IN_SUBROW ::
      s_MAYBE_MOVE_NEXT_B_TILE_INTO_SP ::
      s_RESET_A_TILE_SUBROW_IN_SUBCOL ::
      s_MAYBE_MOVE_A_TILE_INTO_SP ::
      s_MAYBE_MOVE_D_TILE_INTO_ACC ::
      s_PRELOAD_B_TILE_INTO_ARRAY_AND_SET_C_ADDR_IN_ACC ::
      s_DO_MATMUL ::
      s_MAYBE_MOVE_C_TILE_INTO_MEM ::
      //-----------------
      s_NEXT_A_TILE_SUBROW_IN_SUBCOL ::
      s_NEXT_B_TILE_SUBCOL_IN_SUBROW ::
      s_NEXT_A_TILE_SUBCOL ::
      s_NEXT_OUTPUT_GROUP ::
      //-----------------
      Nil = Enum(16)

  val state = RegInit(s_IDLE)
  val state_n = WireDefault(state)
  state_n := state

  //=========================================================================
  // Internal State
  //=========================================================================

  // creates a Reg and the next-state Wire, and returns both
  def regwire(bits: Int) = {
    val wire = Wire(UInt(bits.W))
    val reg = RegNext(wire)
    wire := reg // default wire to read from reg
    (reg, wire)
  }

  //------------------------------------------------------------------------
  // input data-specific constants
  //------------------------------------------------------------------------
  val g_HAS_BIAS              = Reg(Bool())
  val g_REPEATING_BIAS        = Reg(Bool())
  // exec-configs
  val g_DATAFLOW              = Reg(UInt(1.W))
  val g_ACTIVATION            = Reg(UInt(2.W))
  val g_SYSTOLIC_OUT_RSHIFT   = Reg(UInt(LOG2_OTYPE_BITS.W))
  val g_ACC_OUT_RSHIFT        = Reg(UInt(LOG2_OTYPE_BITS.W))
  val g_RELU6_IN_LSHIFT       = Reg(UInt(LOG2_OTYPE_BITS.W))
  // mem-addr of A-matrix
  val g_A_MEM_ADDR            = Reg(UInt(xLen.W))
  val g_B_MEM_ADDR            = Reg(UInt(xLen.W))
  val g_C_MEM_ADDR            = Reg(UInt(xLen.W))
  val g_D_MEM_ADDR            = Reg(UInt(xLen.W))
  // bytes in A-matrix row * rows-per-tile
  val g_A_BYTEs_PER_TILE_ROW  = Reg(UInt(LOG2_MNK_BYTES_PER_TILE_ROW.W))
  val g_B_BYTEs_PER_TILE_ROW  = Reg(UInt(LOG2_MNK_BYTES_PER_TILE_ROW.W))
  val g_C_BYTEs_PER_TILE_ROW  = Reg(UInt(LOG2_MNK_BYTES_PER_TILE_ROW.W))
  val g_D_BYTEs_PER_TILE_ROW  = Reg(UInt(LOG2_MNK_BYTES_PER_TILE_ROW.W))
  // bytes in A-matrix row
  val g_A_BYTEs_PER_ROW       = Reg(UInt(LOG2_MNK_BYTES.W))
  val g_B_BYTEs_PER_ROW       = Reg(UInt(LOG2_MNK_BYTES.W))
  val g_C_BYTEs_PER_ROW       = Reg(UInt(LOG2_MNK_BYTES.W))
  val g_D_BYTEs_PER_ROW       = Reg(UInt(LOG2_MNK_BYTES.W))
  // needed for 0-padding last rows/cols
  val g_LAST_M_ITEMS          = Reg(UInt(LOG2_DIM_COUNT.W))
  val g_LAST_N_ITEMS          = Reg(UInt(LOG2_DIM_COUNT.W))
  val g_LAST_K_ITEMS          = Reg(UInt(LOG2_DIM_COUNT.W))
  // last (x,y,x) tile idx in (C,C,A) matrix
  val g_TILE_ROW_END          = Reg(UInt(LOG2_TILE_IDX.W))
  val g_TILE_COL_END          = Reg(UInt(LOG2_TILE_IDX.W))
  val g_K_TILE_COL_END        = Reg(UInt(LOG2_TILE_IDX.W))

  //------------------------------------------------------------------------
  // global state persistent across all loops
  //------------------------------------------------------------------------
  // current output-group tile (y,x)
  val (gbl_tile_row, gbl_tile_row_n) = regwire(LOG2_TILE_IDX)
  val (gbl_tile_col, gbl_tile_col_n) = regwire(LOG2_TILE_IDX)
  // how many elements (tall,wide) is this tile
  val (gbl_item_rows, gbl_item_rows_n) = regwire(LOG2_DIM_COUNT)
  val (gbl_item_cols, gbl_item_rows_n) = regwire(LOG2_DIM_COUNT)
  // which tmp-slot in sp being used now, and which is the alternate
  val (gbl_B_cur_sp_row_addr, gbl_B_cur_sp_row_addr_n) = regwire(LOG2_SP_ROWS)
  val (gbl_B_alt_sp_row_addr, gbl_B_alt_sp_row_addr_n) = regwire(LOG2_SP_ROWS)

  //------------------------------------------------------------------------
  // global state that is reset for each output-group
  //------------------------------------------------------------------------
  // where to put next C/D-tile in acc
  val (gbl_CD_acc_row_addr, gbl_CD_acc_row_addr_n) = regwire(LOG2_ACC_ROWS)

  //------------------------------------------------------------------------
  // loop1-local state
  //------------------------------------------------------------------------
  // (ul-x,ul-y,br-x,br-y) tile x in output-group
  val (loop1_tile_col_start, loop1_tile_col_start_n) = regwire(LOG2_TILE_IDX)
  val (loop1_tile_col_end,   loop1_tile_col_end_n)   = regwire(LOG2_TILE_IDX)
  val (loop1_tile_row_start, loop1_tile_row_start_n) = regwire(LOG2_TILE_IDX)
  val (loop1_tile_row_end,   loop1_tile_row_end_n)   = regwire(LOG2_TILE_IDX)
  // initialized from global-constants
  val (loop1_A_mem_addr, loop1_A_mem_addr_n) = regwire(xLen)
  val (loop1_B_mem_addr, loop1_B_mem_addr_n) = regwire(xLen)
  val (loop1_C_mem_addr, loop1_C_mem_addr_n) = regwire(xLen)
  val (loop1_D_mem_addr, loop1_D_mem_addr_n) = regwire(xLen)

  //------------------------------------------------------------------------
  // loop2-local state
  //------------------------------------------------------------------------
  // which tile-column in A we are in
  val (loop2_k_tile_col, loop2_k_tile_col_n) = regwire(LOG2_TILE_IDX)
  // how many elems in k-dim is this tile
  val (loop2_k_item_dims, loop2_k_item_dims_n) = regwire(LOG2_DIM_COUNT)
  // initialized from loop1 values
  val (loop2_A_mem_addr, loop2_A_mem_addr_n) = regwire(xLen)
  val (loop2_B_mem_addr, loop2_B_mem_addr_n) = regwire(xLen)
  val (loop2_C_mem_addr, loop2_C_mem_addr_n) = regwire(xLen)
  val (loop2_D_mem_addr, loop2_D_mem_addr_n) = regwire(xLen)

  //------------------------------------------------------------------------
  // loop3-local state
  //------------------------------------------------------------------------
  // initialized from loop2 values
  val (loop3_A_mem_addr, loop3_A_mem_addr_n) = regwire(xLen)
  val (loop3_B_mem_addr, loop3_B_mem_addr_n) = regwire(xLen)
  val (loop3_C_mem_addr, loop3_C_mem_addr_n) = regwire(xLen)
  val (loop3_D_mem_addr, loop3_D_mem_addr_n) = regwire(xLen)

  //------------------------------------------------------------------------
  // loop4-local state
  //------------------------------------------------------------------------
  // where in the sp is the next A tile
  val (loop4_A_sp_row_addr, loop4_A_sp_row_addr_n) = regwire(LOG2_SP_ROWS)
  // initialized from loop3 values
  val (loop4_A_mem_addr, loop4_A_mem_addr_n) = regwire(xLen)
  val (loop4_B_mem_addr, loop4_B_mem_addr_n) = regwire(xLen)
  val (loop4_C_mem_addr, loop4_C_mem_addr_n) = regwire(xLen)
  val (loop4_D_mem_addr, loop4_D_mem_addr_n) = regwire(xLen)

  //=========================================================================
  // utilies used by FSM core
  //=========================================================================

  // continuous assigns (only added in the switch-cases that call this!)
  def update_tile_dims(dummy: Int = 0) = {
    gbl_item_rows_n     := Mux(gbl_tile_row_n === g_TILE_ROW_END, 
                               g_LAST_M_ITEMS, DIM.U)
    gbl_item_cols_n     := Mux(gbl_tile_col_n === g_TILE_COL_END, 
                               g_LAST_N_ITEMS, DIM.U)
    loop2_k_item_dims_n := Mux(loop2_k_tile_col_n === g_K_TILE_COL_END,
                               g_LAST_K_ITEMS, DIM.U)
  }

  def MIN(a: UInt, b: Uint) = Mux(a < b, a, b)

  def hwprintf(str: String) = printf(f"HW-FSM: $str")

  def ACC_ADDR_RD(w: Wire)  = Cat(2.U(2), w(29,0))
  def ACC_ADDR_NEW(w: Wire) = Cat(2.U(2), w(29,0))
  def ACC_ADDR_ACC(w: Wire) = Cat(3.U(2), w(29,0))

  //=========================================================================
  // FSM core
  //=========================================================================
  cmd.ready := false.B
  sched.push := 0.U
  busy := true.B

  switch(state) {
    is (s_IDLE) {
      val l_A_BYTE_WIDTH  = WireDefault(cmd.K << LOG2_ITYPE_BYTES.U)
      val l_BC_BYTE_WIDTH = WireDefault(cmd.N << LOG2_ITYPE_BYTES.U)
      val l_D_BYTE_WIDTH  = WireDefault(cmd.N << LOG2_OTYPE_BYTES.U)

      g_HAS_BIAS              := cmd.bias
      g_REPEATING_BIAS        := cmd.repeating_bias

      g_DATAFLOW              := WEIGHT_STATIONARY.U
      g_ACTIVATION            := cmd.activation
      g_SYSTOLIC_OUT_RSHIFT   := 0.U
      g_ACC_OUT_RSHIFT        := cmd.shift
      g_RELU6_IN_LSHIFT       := cmd.relu6_shift

      g_A_MEM_ADDR            := cmd.A
      g_B_MEM_ADDR            := cmd.B
      g_C_MEM_ADDR            := cmd.C
      g_D_MEM_ADDR            := cmd.D

      g_A_BYTES_PER_TILE_ROW  := l_A_BYTE_WIDTH  << LOG2_DIM.U
      g_B_BYTES_PER_TILE_ROW  := l_BC_BYTE_WIDTH << LOG2_DIM.U
      g_C_BYTES_PER_TILE_ROW  := l_BC_BYTE_WIDTH << LOG2_DIM.U
      g_D_BYTES_PER_TILE_ROW  := l_D_BYTE_WIDTH  << LOG2_DIM.U

      g_A_BYTES_PER_ROW       := l_A_BYTE_WIDTH
      g_B_BYTES_PER_ROW       := l_BC_BYTE_WIDTH
      g_C_BYTES_PER_ROW       := l_BC_BYTE_WIDTH
      g_D_BYTES_PER_ROW       := l_D_BYTE_WIDTH

      g_LAST_M_ITEMS := Mux(cmd.M(LOG2_DIM-1,0).orR,cmd.M(LOG2_DIM-1,0),DIM.U)
      g_LAST_N_ITEMS := Mux(cmd.N(LOG2_DIM-1,0).orR,cmd.N(LOG2_DIM-1,0),DIM.U)
      g_LAST_K_ITEMS := Mux(cmd.K(LOG2_DIM-1,0).orR,cmd.K(LOG2_DIM-1,0),DIM.U)

      g_TILE_ROW_END   := (cmd.M >> LOG2_DIM) + cmd.M(LOG2_DIM-1,0).orR
      g_TILE_COL_END   := (cmd.N >> LOG2_DIM) + cmd.N(LOG2_DIM-1,0).orR
      g_K_TILE_COL_END := (cmd.K >> LOG2_DIM) + cmd.K(LOG2_DIM-1,0).orR

      // update interface signals. we are only ready when an input cmd is
      // ready AND the output queue has 2 slots available to write to
      cmd.ready := sched.ready(1,0).andR

      // issue gemmini commands
      when(cmd.fire()) {
        sched.push               := 2.U
        sched.bits(0).inst.funct := CONFIG_CMD
        sched.bits(0).rs1        := (g_ACC_OUT_RSHIFT << 32) |
                                    (g_ACTIVATION << 3) |
                                    (g_DATAFLOW << 2) |
                                    CONFIG_EX
        sched.bits(0).rs2        := (g_RELU6_IN_LSHIFT << 32) |
                                    g_SYSTOLIC_OUT_RSHIFT
        sched.bits(1).inst.funct := CONFIG_CMD
        sched.bits(1).rs1        := CONFIG_STORE
        sched.bits(1).rs2        := g_C_BYTES_PER_ROW

        // update next state
        state_n := s_RESET_OUTPUT_GROUP
      }
      .otherwise {
        // if we are sitting in idle state, we are not busy!
        busy := false.B
      }
    }
    //=======================================================================
    is (s_RESET_OUTPUT_GROUP) {
      // define mutable gbl state, persist across all ogs
      gbl_tile_row_n          := 0.U
      gbl_tile_col_n          := 0.U
      gbl_B_cur_sp_row_addr_n := GBL_B_SP_ROW_ADDR_1.U
      gbl_B_alt_sp_row_addr_n := GBL_B_SP_ROW_ADDR_2.U
      update_tile_dims()

      // define mutable gbl state, reset after each og
      gbl_CD_acc_row_addr_n := 0.U

      // update the start/end tiles for this output-group (inclusive)
      // NOTE: duplicated with next_output_group!!
      loop1_tile_col_start_n := gbl_tile_col_n
      loop1_tile_col_end_n   := MIN(gbl_tile_col_n + TILE_COLS_PER_GROUP_M1.U,
                                    g_TILE_COL_END.U)
      loop1_tile_row_start_n := gbl_tile_row_n
      loop1_tile_row_end_n   := MIN(gbl_tile_row_n + TILE_ROWS_PER_GROUP_M1.U,
                                    g_TILE_ROW_END.U)

      // derived pointers to matrices in memory for this og
      loop1_A_mem_addr_n := g_A_MEM_ADDR
      loop1_B_mem_addr_n := g_B_MEM_ADDR
      loop1_C_mem_addr_n := g_C_MEM_ADDR
      loop1_D_mem_addr_n := g_D_MEM_ADDR

      // update next state
      state_n := s_RESET_A_TILE_SUBCOL
    }
    //=======================================================================
    is (s_RESET_A_TILE_SUBCOL) {
      loop2_k_tile_col_n    := 0.U
      gbl_tile_row_n        := loop1_tile_row_start
      gbl_tile_col_n        := loop1_tile_col_start
      gbl_CD_acc_row_addr_n := 0.U
      update_tile_dims()

      loop2_A_mem_addr_n := loop1_A_mem_addr
      loop2_B_mem_addr_n := loop1_B_mem_addr
      loop2_C_mem_addr_n := loop1_C_mem_addr
      loop2_D_mem_addr_n := loop1_D_mem_addr

      // update next state
      state_n := s_MOVE_FIRST_B_INTO_SP
    }
    //=======================================================================
    is (s_MOVE_FIRST_B_INTO_SP) {
      // calculate mvin parameters
      val B_mem_addr    = loop2_B_mem_addr
      val B_mem_stride  = B_BYTES_PER_ROW
      val B_sp_row_addr = gbl_B_cur_sp_row_addr(31,0)
      val B_item_rows   = loop2_k_item_dims(15,0)
      val B_item_cols   = gbl_item_cols(15,0)

      // issue gemmini commands
      when(sched.ready(1,0).andR) {
        sched.push               := 2.U
        sched.bits(0).inst.funct := CONFIG_CMD
        sched.bits(0).rs1        := CONFIG_LOAD
        sched.bits(0).rs2        := B_mem_stride
        sched.bits(1).inst.funct := LOAD_CMD
        sched.bits(1).rs1        := Cat(B_item_rows,B_item_cols,B_sp_row_addr)
        sched.bits(1).rs2        := B_mem_addr

        // update next state
        state_n := s_RESET_B_TILE_SUBCOL_IN_SUBROW
      }
    }
    //=======================================================================
    is (s_RESET_B_TILE_SUBCOL_IN_SUBROW) {
      loop3_A_mem_addr_n := loop2_A_mem_addr
      loop3_B_mem_addr_n := loop2_B_mem_addr
      loop3_C_mem_addr_n := loop2_C_mem_addr
      loop3_D_mem_addr_n := loop2_D_mem_addr

      // update next state
      state_n := s_MAYBE_MOVE_NEXT_B_TILE_INTO_SP
    }
    //=======================================================================
    is (s_MAYBE_MOVE_NEXT_B_TILE_INTO_SP) {
      // calculate mvin parameters
      val B_mem_addr    = loop3_B_mem_addr + I_TILE_BYTE_WIDTH.U
      val B_mem_stride  = B_BYTES_PER_ROW.U
      val B_sp_row_addr = gbl_B_alt_sp_row_addr(31,0)
      val B_item_rows   = loop2_k_item_dims(15,0)
      val B_item_cols   = Mux(gbl_tile_col === TILE_COL_END-1,
                              LAST_N_ITEMS(15,0), DIM(15,0))

      // can't load next B-tile if we are already on the last one
      when (gbl_tile_col === loop1_tile_col_end) {
        state_n := s_RESET_A_TILE_SUBROW_IN_SUBCOL
      }
      .elsewhen (sched.ready(1,0).andR) {
        sched.push               := 2.U
        sched.bits(0).inst.funct := CONFIG_CMD
        sched.bits(0).rs1        := CONFIG_LOAD
        sched.bits(0).rs2        := B_mem_stride
        sched.bits(1).inst.funct := LOAD_CMD
        sched.bits(1).rs1        := Cat(B_item_rows,B_item_cols,B_sp_row_addr)
        sched.bits(1).rs2        := B_mem_addr

        // update next state
        state_n := s_RESET_A_TILE_SUBROW_IN_SUBCOL
      }
    }
    //=======================================================================
    is (s_RESET_A_TILE_SUBROW_IN_SUBCOL) {
      // this scope modifies: gbl_tile_row
      //                      gbl_CD_acc_row_addr
      loop4_A_mem_addr_n := loop3_A_mem_addr
      loop4_B_mem_addr_n := loop3_B_mem_addr
      loop4_C_mem_addr_n := loop3_C_mem_addr
      loop4_D_mem_addr_n := loop3_D_mem_addr

      loop4_A_sp_row_addr_n := 0.U

      // update next state
      state_n := s_MAYBE_MOVE_A_TILE_INTO_SP
    }
    //=======================================================================
    is (s_MAYBE_MOVE_A_TILE_INTO_SP) {
      // calculate mvin parameters
      val A_mem_addr    = loop4_A_mem_addr
      val A_mem_stride  = g_A_BYTES_PER_ROW
      val A_sp_row_addr = loop4_A_sp_row_addr(31,0)
      val A_item_rows   = gbl_item_rows(15,0)
      val A_item_cols   = loop2_k_item_dims(15,0)

      // only move A-tiles in during first column of tiles in the og
      when (gbl_tile_col =/= loop1_tile_col_start) {
        state_n := s_MAYBE_MOVE_D_TILE_INTO_ACC
      }
      .elsewhen (sched.ready(1,0).andR) {
        sched.push               := 2.U
        sched.bits(0).inst.funct := CONFIG_CMD
        sched.bits(0).rs1        := CONFIG_LOAD
        sched.bits(0).rs2        := A_mem_stride
        sched.bits(1).inst.funct := LOAD_CMD
        sched.bits(1).rs1        := Cat(A_item_rows,A_item_cols,A_sp_row_addr)
        sched.bits(1).rs2        := A_mem_addr

        // update next state
        state_n := s_MAYBE_MOVE_D_TILE_INTO_ACC
      }
    }
    //=======================================================================
    is (s_MAYBE_MOVE_D_TILE_INTO_ACC) {
      // calculate mvin parameters (NOTE: we know D is valid at this point)
      val D_mem_addr     = loop4_D_mem_addr
      val D_mem_stride   = Mux(g_REPEATING_BIAS, 0.U, g_D_BYTES_PER_ROW)
      val D_acc_row_addr = ACC_ADDR_NEW(gbl_CD_acc_row_addr)
      val D_item_rows    = gbl_item_rows(15,0)
      val D_item_cols    = gbl_item_cols(15,0)

      // only move D-tiles in during first partial-sum in an output-group
      when((loop2_k_tile_col =/= 0) || !g_HAS_BIAS)) {
        state_n := s_PRELOAD_B_TILE_INTO_ARRAY_AND_SET_C_ADDR_IN_ACC
      }
      .elsewhen (sched.ready(1,0).andR) {
        sched.push               := 2.U
        sched.bits(0).inst.funct := CONFIG_CMD
        sched.bits(0).rs1        := CONFIG_LOAD
        sched.bits(0).rs2        := D_mem_stride
        sched.bits(1).inst.funct := LOAD_CMD
        sched.bits(1).rs1        := Cat(D_item_rows,D_item_cols,D_sp_row_addr)
        sched.bits(1).rs2        := D_mem_addr

        // update next state
        state_n := s_PRELOAD_B_TILE_INTO_ARRAY_AND_SET_C_ADDR_IN_ACC
      }
    }
    //=======================================================================
    is (s_PRELOAD_B_TILE_INTO_ARRAY_AND_SET_C_ADDR_IN_ACC) {
      // on first tile in 4th loop: preload this B-tile
      // else:                      preload garbage B-tile (no spad load)
      val B_sp_row_addr = Mux(gbl_tile_row === loop1_tile_row_start,
                              gbl_B_cur_sp_row_addr(31,0),
                              GARBAGE_ADDR(31,0))
      val B_item_rows   = loop2_k_item_dims(15,0)
      val B_item_cols   = gbl_item_cols(15,0)

      // if has D-bias already loaded: accumulate c in accumulator
      // elif first k-col in 2nd loop: overwrite c in accumulator
      // else:                         accumulate c in accumulator
      val C_acc_row_addr = Mux(g_HAS_BIAS || (loop2_k_tile_col>0),
                             ACC_ADDR_ACC(gbl_CD_acc_row_addr),
                             ACC_ADDR_NEW(gbl_CD_acc_row_addr))
      val C_item_rows    = gbl_item_rows(15,0)
      val C_item_cols    = gbl_item_cols(15,0)

      when (sched.ready(0)) {
        sched.push               := 1.U
        sched.bits(0).inst.funct := PRELOAD_CMD
        sched.bits(0).rs1        := Cat(B_item_rows,B_item_cols,B_sp_row_addr)
        sched.bits(0).rs2        := Cat(C_item_rows,C_item_cols,C_sp_row_addr)

        // update next state
        state_n := s_DO_MATMUL
      }
    }
    //=======================================================================
    is (s_DO_MATMUL) {
      // calculate compute parameters
      val A_sp_row_addr = loop4_A_sp_row_addr(31,0)
      val A_item_rows   = gbl_item_rows(15,0)
      val A_item_cols   = loop2_k_item_dims(15,0)

      val D_sp_row_addr = GARBAGE_ADDR(31,0)
      val D_item_rows   = gbl_item_rows(15,0)
      val D_item_cols   = gbl_item_cols(15,0)

      // on first tile in 4th loop: compute_preloaded
      // else: compute_accumulated
      when (sched.ready(0)) {
        sched.push        := 1.U
        sched.bits(0).rs1 := Cat(A_item_rows,A_item_cols,A_sp_row_addr)
        sched.bits(0).rs2 := Cat(D_item_rows,D_item_cols,D_sp_row_addr)
        when (gbl_tile_row === loop1_tile_row_start)) {
          sched.bits(0).inst.funct := COMPUTE_AND_FLIP_CMD
        }
        .otherwise {
          sched.bits(0).inst.funct := COMPUTE_AND_STAY_CMD
        }
        // update next state
        state_n := s_MAYBE_MOVE_C_TILE_INTO_MEM
      }
    }
    //=======================================================================
    is (s_MAYBE_MOVE_C_TILE_INTO_MEM) {
      val C_mem_addr     = loop4_C_mem_addr
      val C_mem_stride   = g_C_BYTES_PER_ROW
      val C_acc_row_addr = ACC_ADDR_RD(gbl_CD_acc_row_addr(29,0))
      val C_item_rows    = gbl_item_rows(15,0)
      val C_item_cols    = gbl_item_cols(15,0)

      when(loop2_k_tile_col =/= K_TILE_COL_END) {
        state_n := s_NEXT_A_TILE_SUBROW_IN_SUBCOL
      }
      .elsewhen (sched.ready(0)) {
        sched.push               := 1.U
        sched.bits(0).inst.funct := STORE_CMD
        sched.bits(0).rs1        := Cat(C_item_rows,C_item_cols,C_sp_row_addr)
        sched.bits(0).rs2        := C_mem_addr

        // update next state
        state_n := s_NEXT_A_TILE_SUBROW_IN_SUBCOL
      }
    }
    //=======================================================================
    is (s_NEXT_A_TILE_SUBROW_IN_SUBCOL) {
      when (gbl_tile_row === loop1_tile_row_end) {
        // just finished the final row of tiles in the 4th loop
        state_n := s_NEXT_B_TILE_SUBCOL_IN_SUBROW
      }
      .otherwise {
        // modify global state
        gbl_tile_row_n        := gbl_tile_row        + 1.U
        gbl_CD_acc_row_addr_n := gbl_CD_acc_row_addr + BYTE_ROWS_PER_TILE
        update_tile_dims()

        // modify loop4-local state
        loop4_A_mem_addr_n    := loop4_A_mem_addr + g_A_BYTES_PER_TILE_ROW
        loop4_C_mem_addr_n    := loop4_C_mem_addr + g_C_BYTES_PER_TILE_ROW
        loop4_D_mem_addr_n    := loop4_D_mem_addr +
                                  Mux(g_HAS_BIAS && !g_REPEATING_BIAS,
                                      g_D_BYTES_PER_TILE_ROW, 0.U)

        loop4_A_sp_row_addr_n := loop4_A_sp_row_addr + BYTE_ROWS_PER_TILE.U

        // update next state
        state_n := s_MAYBE_MOVE_A_TILE_INTO_SP
      }
    }
    //=======================================================================
    is (s_NEXT_B_TILE_SUBCOL_IN_SUBROW) {
      when (gbl_tile_col === loop1_tile_col_end) {
        // we have already done the last column in the output-group
        state_n := s_NEXT_A_TILE_SUBCOL
      }
      .otherwise {
        // modify global state
        gbl_tile_row_n        := loop1_tile_row_start
        gbl_tile_col_n        := gbl_tile_col        + 1.U
        gbl_CD_acc_row_addr_n := gbl_CD_acc_row_addr + BYTE_ROWS_PER_TILE.U
        update_tile_dims()

        gbl_B_cur_sp_row_addr_n := gbl_B_alt_sp_row_addr
        gbl_B_alt_sp_row_addr_n := gbl_B_cur_sp_row_addr

        // modify loop3-local state
        loop3_A_mem_addr_n := loop3_A_mem_addr + 0.U
        loop3_B_mem_addr_n := loop3_B_mem_addr + I_TILE_BYTE_WIDTH.U
        loop3_C_mem_addr_n := loop3_C_mem_addr + I_TILE_BYTE_WIDTH.U
        loop3_D_mem_addr_n := loop3_D_mem_addr + O_TILE_BYTE_WIDTH.U

        // update next state
        state_n := s_MAYBE_MOVE_NEXT_B_TILE_INTO_SP
      }
    }
    //=======================================================================
    is (s_NEXT_A_TILE_SUBCOL) {
      when (loop2_k_tile_col === K_TILE_COL_END) {
        state_n := s_NEXT_OUTPUT_GROUP
      }
      .otherwise {
        loop2_k_tile_col_n    := loop2_k_tile_col + 1.U
        gbl_tile_row_n        := loop1_tile_row_start
        gbl_tile_col_n        := loop1_tile_col_start
        gbl_CD_acc_row_addr_n := 0.U
        update_tile_dims()

        loop2_A_mem_addr_n := loop2_A_mem_addr + I_TILE_BYTE_WIDTH.U
        loop2_B_mem_addr_n := loop2_B_mem_addr + g_B_BYTES_PER_TILE_ROW
        loop2_C_mem_addr_n := loop2_C_mem_addr + 0.U
        loop2_D_mem_addr_n := loop2_D_mem_addr + 0.U

        // swap current/alternate B-tile scratchpad addrs
        gbl_B_cur_sp_row_addr_n := gbl_B_alt_sp_row_addr
        gbl_B_alt_sp_row_addr_n := gbl_B_cur_sp_row_addr

        // update next state
        state_n := s_MOVE_FIRST_B_TILE_INTO_SP
      }
    }
    //=======================================================================
    is (s_NEXT_OUTPUT_GROUP) {
      val l_did_row_incr = WireDefault(false.B)
      val l_did_col_incr = WireDefault(false.B)

      when (gbl_tile_col === TILE_COL_END && gbl_tile_row === TILE_ROW_END) {
        // update next state
        state_n := s_IDLE
      }
      .otherwise {
        when (gbl_tile_col === TILE_COL_END) {
          gbl_tile_row_n := gbl_tile_row + 1.U
          gbl_tile_col_n := 0
          update_tile_dims()
          l_did_row_incr := true.B
        }
        .otherwise {
          gbl_tile_row_n := gbl_tile_row
          gbl_tile_col_n := gbl_tile_col + 1.U
          update_tile_dims()
          l_did_col_incr := true.B
        }
   
        // reset global state that resets for each new output-group
        gbl_CD_acc_row_addr_n := 0.U

        // update the start/end tiles for this output-group (inclusive)
        loop1_tile_col_start_n := gbl_tile_col_n
        loop1_tile_col_end_n := MIN(gbl_tile_col_n + TILE_COLS_PER_GROUP_M1.U,
                                    TILE_COL_END)
        loop1_tile_row_start_n := gbl_tile_row_n
        loop1_tile_row_end_n := MIN(gbl_tile_row_n + TILE_ROWS_PER_GROUP_M1.U,
                                    TILE_ROW_END)
         
        // update all derived pointers to matrices in memory
        when(l_did_row_incr) {
          loop1_A_mem_addr_n := g_A_MEM_ADDR + (loop1_tile_row_start_n *
                                                g_A_BYTES_PER_TILE_ROW)
          loop1_B_mem_addr_n := g_B_MEM_ADDR
          loop1_C_mem_addr_n := g_C_MEM_ADDR + (loop1_tile_row_start_n *
                                                g_C_BYTES_PER_TILE_ROW)
          loop1_D_mem_addr_n := Mux(!g_HAS_BIAS, 0.U,
                                  Mux(g_REPEATING_BIAS, g_D_MEM_ADDR,
                                    (g_D_MEM_ADDR + (loop1_tile_row_start_n *
                                                     g_D_BYTES_PER_TILE_ROW))))
        }
        .elsewhen (l_did_col_incr) {
          loop1_A_mem_addr_n := loop1_A_mem_addr + 0.U
          loop1_B_mem_addr_n := loop1_B_mem_addr + I_BYTE_COLS_PER_GROUP.U
          loop1_C_mem_addr_n := loop1_C_mem_addr + I_BYTE_COLS_PER_GROUP.U
          loop1_D_mem_addr_n := loop1_D_mem_addr + 
                                Mux(!g_HAS_BIAS, 0.U, O_BYTE_COLS_PER_GROUP.U)
        }

        // update next state
        state_n := s_RESET_A_TILE_SUBCOL
      }
    }
  }
}

object TilerFSM {
  def apply(config: GemminiArrayConfig[T])(implicit p: Parameters) :TilerFSM 
    = Module(new TilerFSM(config))
}
