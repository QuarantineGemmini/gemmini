package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._

//===========================================================================
// TilerController Interface
//===========================================================================
class TilerCmd(config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends Bundle with HasCoreParameters
{
  import config._
  val M              = Input(UInt(32.U))
  val N              = Input(UInt(32.U))
  val K              = Input(UInt(32.U))
  val A              = Input(UInt(xLen.W))
  val B              = Input(UInt(xLen.W))
  val D              = Input(UInt(xLen.W))
  val C              = Input(UInt(xLen.W))
  val in_shift       = Input(UInt(log2Up(accType.getWidth).W))
  val acc_shift      = Input(UInt(log2Up(accType.getWidth).W))
  val relu6_shift    = Input(UInt(log2Up(accType.getWidth).W))
  val activation     = Input(UInt(2.W))
  val repeating_bias = Input(Bool())

  override def cloneType: this.type =
    new TilerCmd(config).asInstanceOf[this.type]
}

class TilerEvent(config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends Bundle with HasCoreParameters
{
  // TODO: enumerate types of events
  // TODO: figure out better way to union the possible data for each event.
  //       should we burst the data if it doesn't fit in 32 bits?
  val valid = Output(Bool())
  val type  = Output(UInt(5.W))
  val data  = Output(Uint(32.W))

  override def cloneType: this.type =
    new TilerEvent(config).asInstanceOf[this.type]
}

object TilerControllerIO {
  def apply(config: GemminiArrayConfig[T]) = IO(new Bundle {

    val in = Flipped(Decoupled(new TilerCmd))

    val completed = Flipped(Valid(UInt(log2Up(rob_entries).W)))

    val issue = new Bundle {
      val load  = new ROBIssue(cmd_t, rob_entries)
      val store = new ROBIssue(cmd_t, rob_entries)
      val exec  = new ROBIssue(cmd_t, rob_entries)
      val flush = new ROBIssue(cmd_t, rob_entries)
    }
    val completed = new Bundle {
      val ld = new ROBIssue(cmd_t, nEntries)
      val st = new ROBIssue(cmd_t, nEntries)
      val ex = new ROBIssue(cmd_t, nEntries)
    }
    val busy = Output(Bool())

    val eventbus =
  }
}

class RoCCInstruction extends Bundle {
  val funct = Bits(width = 7)
  val rs2 = Bits(width = 5)
  val rs1 = Bits(width = 5)
  val xd = Bool()
  val xs1 = Bool()
  val xs2 = Bool()
  val rd = Bits(width = 5)
  val opcode = Bits(width = 7)
}

class RoCCCommand(implicit p: Parameters) extends CoreBundle()(p) {
  val inst = new RoCCInstruction
  val rs1 = Bits(width = xLen)
  val rs2 = Bits(width = xLen)
  val status = new MStatus
}

val sched_insn1 = new RoCCInstruction
val sched_insn2 = new RoCCInstruction
val sched_cmd1 = new RoCCommand
val sched_cmd2 = new RoCCommand

def make_mvin_cmd()(implicit p: Parameters) = {
  sched_cmd1.inst.funct := CONFIG_CMD
  sched_cmd1.rs1        := CONFIG_LOAD
  sched_cmd1.rs2        := 
  sched_cmd2.inst.funct := LOAD_CMD

class TilerSchedulerCmd()

class TilerEvent(config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends Bundle with HasCoreParameters
{
  // TODO: enumerate types of events
  // TODO: figure out better way to union the possible data for each event.
  //       should we burst the data if it doesn't fit in 32 bits?
  val valid = Output(Bool())
  val type  = Output(UInt(5.W))
  val data  = Output(Uint(32.W))

  override def cloneType: this.type =
    new TilerEvent(config).asInstanceOf[this.type]
}

//===========================================================================
// TilerController Implementation
//===========================================================================
class TilerController(config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends Bundle with HasCoreParameters
{
  import config._

  val io = TilerControllerIO(config)

  //=========================================================================
  // arbitrate incoming completions
  //=========================================================================
  val completed_arb = Module(new Arbiter(UInt(log2Up(rob_entries).W), 4))

  // TODO: ExecuteController does not have a Decoupled completed interface!
  completed_arb.io.in(0).valid <> exec.io.completed.valid
  completed_arb.io.in(0).bits  <> exec.io.completed.bits
  completed_arb.io.in(1)       <> load.io.completed
  completed_arb.io.in(2)       <> store.io.completed
  completed_arb.io.in(3)       <> flush.io.completed

  // NOTE: i have to emit RoCCcommands to each unit!

  //val completed_valid        = completed_arb.io.out.valid
  //val completed_tag          = completed_arb.io.out.bits
  //completed_arb.io.out.ready := retire_completed_command

  //=========================================================================
  // input data-specific constants
  //=========================================================================
  val HAs_BIAS              = RegInit(0.U(1))
  val REPEATING_BIAS        = RegInit(0.U(1))

  val DATAFLOW              = WEIGHT_STATIONARY;
  val ACTIVATION            = act;
  val SYSTOLIC_OUT_RSHIFT   = 0;
  val ACC_OUT_RSHIFT        = shift;
  val RELU6_IN_LSHIFT       = relu6_shift;

  val A_MEM_ADDR            = A;
  val B_MEM_ADDR            = B;
  val C_MEM_ADDR            = C;
  val D_MEM_ADDR            = D;
  val A_BYTEs_PER_TILE_ROW  = DIM * A_BYTE_WIDTH;
  val B_BYTEs_PER_TILE_ROW  = DIM * BC_BYTE_WIDTH;
  val C_BYTEs_PER_TILE_ROW  = DIM * BC_BYTE_WIDTH;
  val D_BYTEs_PER_TILE_ROW  = DIM * D_BYTE_WIDTH;
  val A_BYTEs_PER_ROW       = A_BYTE_WIDTH;
  val B_BYTEs_PER_ROW       = BC_BYTE_WIDTH;
  val C_BYTEs_PER_ROW       = BC_BYTE_WIDTH;
  val D_BYTEs_PER_ROW       = D_BYTE_WIDTH;

  val LAST_M_ITEMS          = default_if_zero(M % DIM, DIM);
  val LAST_N_ITEMS          = default_if_zero(N % DIM, DIM);
  val LAST_K_ITEMS          = default_if_zero(K % DIM, DIM);

  val TILE_ROW_END          = ((M + DIM - 1) / DIM) - 1;
  val TILE_COL_END          = ((N + DIM - 1) / DIM) - 1;
  val K_TILE_COL_END        = ((K + DIM - 1) / DIM) - 1;

  //=========================================================================
  // input data-specific constants
  //=========================================================================
  HAs_BIAS =

  val HAs_BIAS              = bias;
  val REPEATING_BIAS        = repeating_bias;

  val DATAFLOW              = WEIGHT_STATIONARY;
  val ACTIVATION            = act;
  val SYSTOLIC_OUT_RSHIFT   = 0;
  val ACC_OUT_RSHIFT        = shift;
  val RELU6_IN_LSHIFT       = relu6_shift;

  val A_MEM_ADDR            = A;
  val B_MEM_ADDR            = B;
  val C_MEM_ADDR            = C;
  val D_MEM_ADDR            = D;
  val A_BYTEs_PER_TILE_ROW  = DIM * A_BYTE_WIDTH;
  val B_BYTEs_PER_TILE_ROW  = DIM * BC_BYTE_WIDTH;
  val C_BYTEs_PER_TILE_ROW  = DIM * BC_BYTE_WIDTH;
  val D_BYTEs_PER_TILE_ROW  = DIM * D_BYTE_WIDTH;
  val A_BYTEs_PER_ROW       = A_BYTE_WIDTH;
  val B_BYTEs_PER_ROW       = BC_BYTE_WIDTH;
  val C_BYTEs_PER_ROW       = BC_BYTE_WIDTH;
  val D_BYTEs_PER_ROW       = D_BYTE_WIDTH;

  val LAST_M_ITEMS          = default_if_zero(M % DIM, DIM);
  val LAST_N_ITEMS          = default_if_zero(N % DIM, DIM);
  val LAST_K_ITEMS          = default_if_zero(K % DIM, DIM);

  val TILE_ROW_END          = ((M + DIM - 1) / DIM) - 1;
  val TILE_COL_END          = ((N + DIM - 1) / DIM) - 1;
  val K_TILE_COL_END        = ((K + DIM - 1) / DIM) - 1;


}

//===========================================================================
// TilerController's Internal FSM implementation
//===========================================================================
class TilerFSM(config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends Bundle with HasCoreParameters
{
  import config._

  // see diagram for what each state does
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

  val cmd = io.cmd_in

  val io = IO(new Bundle {
    val cmd = Decoupled(new FSM)
    val sched = Decoupled(new RoCCCommand)
  }

  val state = RegInit(s_IDLE);

  io.in.ready := false.B

  def hwprintf(str: String) = printf(f"HW-FSM: $str")

  //=========================================================================
  // utilies used by FSM core
  //=========================================================================
  // continuous assigns
  def update_tile_dims(dummy: Int = 0) = {
    gbl_item_rows     := (gbl_tile_row === TILE_ROW_END) ? LAST_M_ITEMS : DIM
    gbl_item_cols     := (gbl_tile_col === TILE_COL_END) ? LAST_N_ITEMS : DIM
    loop2_k_item_dims := (loop2_k_tile_col===K_TILE_COL_END)?LAST_K_ITEMS:DIM
  }

  def MIN(a: UInt, b: Uint) = (a < b) ? a : b

  // used by s_NEXT_OUTPUT_GROUP state
  val did_row_incr = Bool()
  val did_col_incr = Bool()
  did_row_incr     := false.B
  did_col_incr     := false.B

  //=========================================================================
  // queue requests to the TilerScheduler. Enqueue up to 2 cmds per cycle
  //=========================================================================
  val (sched, _) = MultiTailedQueue(io.sched, 8, 2);

  sched.push := 0.U

  def ACC_ADDR_RD(d: UInt)  = Cat("10".U(2), d(29,0))
  def ACC_ADDR_NEW(d: UInt) = Cat("10".U(2), d(29,0))
  def ACC_ADDR_ACC(d: UInt) = Cat("11".U(2), d(29,0))

  //=========================================================================
  // FSM core 
  //=========================================================================
  switch(state) {
    is (s_IDLE) {
      val A_BYTE_WIDTH  = cmd.K << (log2up(inputType.getWidth) - 1);
      val BC_BYTE_WIDTH = cmd.N << (log2up(inputType.getWidth) - 1);
      val D_BYTE_WIDTH  = cmd.N << (log2up(accType.getWidth) - 1);

      HAS_BIAS              := cmd.bias;
      REPEATING_BIAS        := cmd.repeating_bias;

      DATAFLOW              := WEIGHT_STATIONARY;
      ACTIVATION            := cmd.act;
      SYSTOLIC_OUT_RSHIFT   := 0;
      ACC_OUT_RSHIFT        := cmd.shift;
      RELU6_IN_LSHIFT       := cmd.relu6_shift;

      A_MEM_ADDR            := cmd.A;
      B_MEM_ADDR            := cmd.B;
      C_MEM_ADDR            := cmd.C;
      D_MEM_ADDR            := cmd.D;
      A_BYTES_PER_TILE_ROW  := cmd.A_BYTE_WIDTH  << LOG2DIM;
      B_BYTES_PER_TILE_ROW  := cmd.BC_BYTE_WIDTH << LOG2DIM;
      C_BYTES_PER_TILE_ROW  := cmd.BC_BYTE_WIDTH << LOG2DIM;
      D_BYTES_PER_TILE_ROW  := cmd.D_BYTE_WIDTH  << LOG2DIM;
      A_BYTES_PER_ROW       := cmd.A_BYTE_WIDTH;
      B_BYTES_PER_ROW       := cmd.BC_BYTE_WIDTH;
      C_BYTES_PER_ROW       := cmd.BC_BYTE_WIDTH;
      D_BYTES_PER_ROW       := cmd.D_BYTE_WIDTH;

      LAST_M_ITEMS := (cmd.M(LOG2MIN-1,0) === 0.U) ? cmd.M(LOG2DIM-1,0) : DIM
      LAST_N_ITEMS := (cmd.N(LOG2MIN-1,0) === 0.U) ? cmd.N(LOG2DIM-1,0) : DIM
      LAST_K_ITEMS := (cmd.K(LOG2MIN-1,0) === 0.U) ? cmd.K(LOG2DIM-1,0) : DIM

      TILE_ROW_END   := (cmd.M >> LOG2DIM) + cmd.M(LOG2DIM-1,0).orR;
      TILE_COL_END   := (cmd.N >> LOG2DIM) + cmd.N(LOG2DIM-1,0).orR;
      K_TILE_COL_END := (cmd.K >> LOG2DIM) + cmd.K(LOG2DIM-1,0).orR;

      // update interface signals
      io.in.ready := true.B

      // update next state
      when(io.in.fire()) {
        state := s_RESET_OUTPUT_GROUP
      }
    }
    //=======================================================================
    is (s_RESET_OUTPUT_GROUP) {
      // define mutable gbl state, persist across all ogs
      gbl_tile_row          := 0.U
      gbl_tile_col          := 0.U
      gbl_B_cur_sp_row_addr := GBL_B_SP_ROW_ADDR_1
      gbl_B_alt_sp_row_addr := GBL_B_SP_ROW_ADDR_2
      update_tile_dims()

      // define mutable gbl state, reset after each og
      gbl_CD_acc_row_addr := 0.U

      // update the start/end tiles for this output-group (inclusive)
      // NOTE: duplicated with next_output_group!!
      loop1_tile_col_start := gbl_tile_col
      loop1_tile_col_end   := min(gbl_tile_col + TILE_COLS_PER_GROUP - 1,
                                  TILE_COL_END)
      loop1_tile_row_start := gbl_tile_row
      loop1_tile_row_end   := max(gbl_tile_row + TILE_ROWS_PER_GROUP - 1,
                                  TILE_ROW_END)

      // derived pointers to matrices in memory for this og
      loop1_A_mem_addr := A_MEM_ADDR
      loop1_B_mem_addr := B_MEM_ADDR
      loop1_C_mem_addr := C_MEM_ADDR
      loop1_D_mem_addr := D_MEM_ADDR

      // update next state
      state := s_RESET_A_TILE_SUBCOL
      DBG_OG("reset_output_group");
    }
    //=======================================================================
    is (s_RESET_A_TILE_SUBCOL) {
      loop2_k_tile_col    := 0
      gbl_tile_row        := loop1_tile_row_start
      gbl_tile_col        := loop1_tile_col_start
      gbl_CD_acc_row_addr := 0
      update_tile_dims()

      loop2_A_mem_addr := loop1_A_mem_addr
      loop2_B_mem_addr := loop1_B_mem_addr
      loop2_C_mem_addr := loop1_C_mem_addr
      loop2_D_mem_addr := loop1_D_mem_addr

      // update next state
      state := s_MOVE_FIRST_B_INTO_SP
      DBG_LOOP("  reset_A_tile_subcol              ")
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
        state := s_RESET_B_TILE_SUBCOL_IN_SUBROW
        DBG_MVIN_B(B_mem_stride, B_item_rows, B_item_cols, B_sp_row_addr)
      }
    }
    //=======================================================================
    is (s_RESET_B_TILE_SUBCOL_IN_SUBROW) {
      loop3_A_mem_addr := loop2_A_mem_addr
      loop3_B_mem_addr := loop2_B_mem_addr
      loop3_C_mem_addr := loop2_C_mem_addr
      loop3_D_mem_addr := loop2_D_mem_addr

      // update next state
      state := s_MAYBE_MOVE_NEXT_B_TILE_INTO_SP
      DBG_LOOP("    reset_B_tile_subcol_in_subrow  ");
    }
    //=======================================================================
    is (s_MAYBE_MOVE_NEXT_B_TILE_INTO_SP) {
      // calculate mvin parameters
      val B_mem_addr    = loop3_B_mem_addr + I_TILE_BYTE_WIDTH
      val B_mem_stride  = B_BYTES_PER_ROW
      val B_sp_row_addr = gbl_B_alt_sp_row_addr(31,0)
      val B_item_rows   = loop2_k_item_dims(15,0)
      val B_item_cols   = Mux(gbl_tile_col === TILE_COL_END-1,
                              LAST_N_ITEMS(15,0), DIM(15,0))

      // can't load next B-tile if we are already on the last one
      when (gbl_tile_col === loop1_tile_col_end) {
        state := s_RESET_A_TILE_SUBROW_IN_SUBCOL
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
        state := s_RESET_A_TILE_SUBROW_IN_SUBCOL
        DBG_MVIN_B(B_mem_stride, B_item_rows, B_item_cols, B_sp_row_addr)
      }
    }
    //=======================================================================
    is (s_RESET_A_TILE_SUBROW_IN_SUBCOL) {
      // this scope modifies: gbl_tile_row
      //                      gbl_CD_acc_row_addr
      loop4_A_mem_addr    := loop3_A_mem_addr
      loop4_B_mem_addr    := loop3_B_mem_addr
      loop4_C_mem_addr    := loop3_C_mem_addr
      loop4_D_mem_addr    := loop3_D_mem_addr

      loop4_A_sp_row_addr := 0.U

      // update next state
      state := s_MAYBE_MOVE_A_TILE_INTO_SP
      DBG_LOOP("      reset_A_tile_subrow_in_subcol")
    }
    //=======================================================================
    is (s_MAYBE_MOVE_A_TILE_INTO_SP) {
      // calculate mvin parameters
      val A_mem_addr    = loop4_A_mem_addr
      val A_mem_stride  = A_BYTES_PER_ROW
      val A_sp_row_addr = loop4_A_sp_row_addr(31,0)
      val A_item_rows   = gbl_item_rows(15,0)
      val A_item_cols   = loop2_k_item_dims(15,0)

      // only move A-tiles in during first column of tiles in the og
      when (gbl_tile_col =/= loop1_tile_col_start) {
        state := s_MAYBE_MOVE_D_TILE_INTO_ACC
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
        state := s_MAYBE_MOVE_D_TILE_INTO_ACC
        DBG_MVIN_A(A_mem_stride, A_item_rows, A_item_cols, A_sp_row_addr)
      }
    }
    //=======================================================================
    is (s_MAYBE_MOVE_D_TILE_INTO_ACC) {
      // calculate mvin parameters (NOTE: we know D is valid at this point)
      val D_mem_addr     = loop4_D_mem_addr
      val D_mem_stride   = REPEATING_BIAS ? 0.U : D_BYTES_PER_ROW
      val D_acc_row_addr = ACC_ADDR_NEW(gbl_CD_acc_row_addr)
      val D_item_rows    = gbl_item_rows(15,0)
      val D_item_cols    = gbl_item_cols(15,0)

      // only move D-tiles in during first partial-sum in an output-group
      when(!((loop2_k_tile_col === 0) & HAS_BIAS)) {
        state := s_PRELOAD_B_TILE_INTO_ARRAY_AND_SET_C_ADDR_IN_ACC
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
        state := s_PRELOAD_B_TILE_INTO_ARRAY_AND_SET_C_ADDR_IN_ACC
        DBG_MVIN_A(D_mem_stride, D_item_rows, D_item_cols, D_sp_row_addr)
      }
    }
    //=======================================================================
    is (s_PRELOAD_B_TILE_INTO_ARRAY_AND_SET_C_ADDR_IN_ACC) {
      // on first tile in 4th loop: preload this B-tile
      // else:                      preload garbage B-tile (no spad load)
      val B_sp_row_addr = Mux(gbl_tile_row == loop1_tile_row_start,
                              gbl_B_cur_sp_row_addr(31,0), 
                              GARBAGE_ADDR(31,0)
      val B_item_rows   = loop2_k_item_dims(15,0)
      val B_item_cols   = gbl_item_cols(15,0)

      // if has D-bias already loaded: accumulate c in accumulator
      // elif first k-col in 2nd loop: overwrite c in accumulator
      // else:                         accumulate c in accumulator
      val C_acc_row_addr = Mux(HAS_BIAS || (loop2_k_tile_col>0),
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
        state := s_DO_MATMUL
        DBG_PRELOAD_B(B_item_rows,B_item_cols,B_sp_row_addr,
                      C_item_rows,C_item_cols,C_sp_row_addr)
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
          DBG_COMPUTE_PRE(A_item_rows,A_item_cols,A_sp_row_addr,
                          D_item_rows,D_item_cols,D_sp_row_addr)
        }
        .otherwise {
          sched.bits(0).inst.funct := COMPUTE_AND_STAY_CMD
          DBG_COMPUTE_ACC(A_item_rows,A_item_cols,A_sp_row_addr,
                          D_item_rows,D_item_cols,D_sp_row_addr)
        }
        // update next state
        state := s_MAYBE_MOVE_C_TILE_INTO_MEM
      }
    }
    //=======================================================================
    is (s_MAYBE_MOVE_C_TILE_INTO_MEM) {
      val C_mem_addr     = loop4_C_mem_addr
      val C_mem_stride   = C_BYTES_PER_ROW
      val C_acc_row_addr = ACC_ADDR_RD(gbl_CD_acc_row_addr(29,0))
      val C_item_rows    = gbl_item_rows(15,0)
      val C_item_cols    = gbl_item_cols(15,0)

      when(loop2_k_tile_col =/= K_TILE_COL_END) {
        state := s_NEXT_A_TILE_SUBROW_IN_SUBCOL
      }
      .elsewhen (sched.ready(0)) {
        sched.push               := 1.U
        sched.bits(0).inst.funct := STORE_CMD
        sched.bits(0).rs1        := Cat(C_item_rows,C_item_cols,C_sp_row_addr)
        sched.bits(0).rs2        := C_mem_addr

        // update next state
        state := s_NEXT_A_TILE_SUBROW_IN_SUBCOL
        DBG_MVOUT_C(C_mem_stride, C_item_rows, C_item_cols, C_sp_row_addr)
      }
    }
    //=======================================================================
    is (s_NEXT_A_TILE_SUBROW_IN_SUBCOL) {
      when (gbl_tile_row === loop1_tile_row_end) {
        // just finished the final row of tiles in the 4th loop
        state := s_NEXT_B_TILE_SUBCOL_IN_SUBROW
        DBG_LOOP("    <-next_A_tile_subrow_in_subcol ")
      }
      .otherwise {
        // modify global state
        gbl_tile_row        := gbl_tile_row        + 1.U
        gbl_CD_acc_row_addr := gbl_CD_acc_row_addr + BYTE_ROWS_PER_TILE
        update_tile_dims()

        // modify loop4-local state
        loop4_A_mem_addr    := loop4_A_mem_addr + A_BYTES_PER_TILE_ROW
        loop4_C_mem_addr    := loop4_C_mem_addr + C_BYTES_PER_TILE_ROW
        loop4_D_mem_addr    := loop4_D_mem_addr + 
                                (HAS_BIAS && !REPEATING_BIAS) 
                                  ? D_BYTES_PER_TILE_ROW : 0
                                                   
        loop4_A_sp_row_addr := loop4_A_sp_row_addr  BYTE_ROWS_PER_TILE

        // update next state
        state := s_MAYBE_MOVE_A_TILE_INTO_SP
        DBG_LOOP("      next_A_tile_subrow_in_subcol ")
      }
    }
    //=======================================================================
    is (s_NEXT_B_TILE_SUBCOL_IN_SUBROW) {
      when (gbl_tile_col === loop1_tile_col_end) {
        // we have already done the last column in the output-group
        state := s_NEXT_A_TILE_SUBCOL
        DBG_LOOP("  <-next_B_tile_subcol_in_subrow   ")
      }
      .otherwise {
        // modify global state
        gbl_tile_row        := loop1_tile_row_start
        gbl_tile_col        := gbl_tile_col        + 1.U
        gbl_CD_acc_row_addr := gbl_CD_acc_row_addr + BYTE_ROWS_PER_TILE
        update_tile_dims()

        gbl_B_cur_sp_row_addr := gbl_B_alt_sp_row_addr
        gbl_B_alt_sp_row_addr := gbl_B_cur_sp_row_addr

        // modify loop3-local state
        loop3_A_mem_addr := loop3_A_mem_addr + 0.U
        loop3_B_mem_addr := loop3_B_mem_addr + I_TILE_BYTE_WIDTH
        loop3_C_mem_addr := loop3_C_mem_addr + I_TILE_BYTE_WIDTH
        loop3_D_mem_addr := loop3_D_mem_addr + O_TILE_BYTE_WIDTH

        // update next state
        state := s_MAYBE_MOVE_NEXT_B_TILE_INTO_SP
        DBG_LOOP("    next_B_tile_subcol_in_subrow   ")
      }
    }
    //=======================================================================
    is (s_NEXT_A_TILE_SUBCOL) {
      when (loop2_k_tile_col === K_TILE_COL_END) {
        state := s_NEXT_OUTPUT_GROUP
        DBG_LOOP("<-next_A_tile_subcol               ")
      }
      .otherwise {
        loop2_k_tile_col    := loop2_k_tile_col + 1.U
        gbl_tile_row        := loop1_tile_row_start
        gbl_tile_col        := loop1_tile_col_start
        gbl_CD_acc_row_addr := 0.U
        update_tile_dims()

        loop2_A_mem_addr := loop2_A_mem_addr + I_TILE_BYTE_WIDTH
        loop2_B_mem_addr := loop2_B_mem_addr + B_BYTES_PER_TILE_ROW
        loop2_C_mem_addr := loop2_C_mem_addr + 0.U
        loop2_D_mem_addr := loop2_D_mem_addr + 0.U

        // swap current/alternate B-tile scratchpad addrs
        gbl_B_cur_sp_row_addr := gbl_B_alt_sp_row_addr
        gbl_B_alt_sp_row_addr := gbl_B_cur_sp_row_addr

        // update next state
        state := s_MOVE_FIRST_B_TILE_INTO_SP
        DBG_LOOP("  next_A_tile_subcol               ")
      }
    }
    //=======================================================================
    is (s_NEXT_OUTPUT_GROUP ) {
      when (gbl_tile_col === TILE_COL_END && 
            gbl_tile_row === TILE_ROW_END) {
        state := s_IDLE
        DBG_OG("output_group finished")
      }
      .otherwise {
        when (gbl_tile_col === TILE_COL_END) {
          did_row_incr := true.B
          gbl_tile_col := 0
          gbl_tile_row := gbl_tile_row + 1.U
          update_tile_dims(self)
        }
        .otherwise {
          did_col_incr := true.B
          gbl_tile_col := gbl_tile_col + 1.U
          gbl_tile_row := loop1_tile_row_start
          update_tile_dims(self)
        }

        TODO TODO TODO TODO: chisel is NOT procedural! you need the next
          state and previous state signals for synchronous logic!
          for example:
            foo_n := x
            foo := foo_n

        // reset global state that resets for each new output-group
        gbl_CD_acc_row_addr := 0.U

        // update the start/end tiles for this output-group (inclusive)
        loop1_tile_col_start := gbl_tile_col
        loop1_tile_col_end   := MIN(gbl_tile_col + TILE_COLS_PER_GROUP - 1.U,
                                   TILE_COL_END)
        loop1_tile_row_start := gbl_tile_row
        loop1_tile_row_end   := MIN(gbl_tile_row + TILE_ROWS_PER_GROUP - 1.U,
                                   TILE_ROW_END)

        // update all derived pointers to matrices in memory
        when (did_row_incr) {
          loop1_A_mem_addr  := A_MEM_ADDR + (loop1_tile_row_start *
                                             A_BYTES_PER_TILE_ROW)
          loop1_B_mem_addr  := B_MEM_ADDR
          loop1_C_mem_addr  := C_MEM_ADDR + (loop1_tile_row_start *
                                             C_BYTES_PER_TILE_ROW)
          loop1_D_mem_addr  := !HAS_BIAS ? 0.U
                                : (REPEATING_BIAS ? D_MEM_ADDR
                                  : D_MEM_ADDR + (loop1_tile_row_start *
                                                  D_BYTES_PER_TILE_ROW))
        }
        when (did_col_incr) {
          loop1_A_mem_addr := loop1_A_mem_addr + 0.U
          loop1_B_mem_addr := loop1_B_mem_addr + I_BYTE_COLS_PER_GROUP
          loop1_C_mem_addr := loop1_C_mem_addr + I_BYTE_COLS_PER_GROUP
          loop1_D_mem_addr := loop1_D_mem_addr + (!HAS_BIAS ? 0.U
                                                    : O_BYTE_COLS_PER_GROUP
        }
        // update next state
        state := s_RESET_A_TILE_SUBCOL
        DBG_OG("next_output_group ")
      }
    }
  }
}


object TilerController {
  def apply(enq: ReadyValidIO[RoCCCommand], block_size: Int)(implicit p: Parameters): DecoupledIO[RoCCCommand] = {
  //  val lu = Module(new LoopUnroller(block_size))
  //  lu.io.in <> enq
  //  lu.io.out
  //}
}
