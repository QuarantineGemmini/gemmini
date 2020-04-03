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
      s_INIT ::
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
      Nil = Enum(17)

  val cmd = io.cmd_in

  val state = RegInit(s_IDLE);

  io.in.ready := false.B

  def hwprintf(str: String) = printf(f"HW-FSM: $str")

  // continuous assigns
  def update_tile_dims(dummy: Int = 0) = {
    gbl_item_rows     := (gbl_tile_row === TILE_ROW_END) ? LAST_M_ITEMS : DIM
    gbl_item_cols     := (gbl_tile_col === TILE_COL_END) ? LAST_N_ITEMS : DIM
    loop2_k_item_dims := (loop2_k_tile_col===K_TILE_COL_END)?LAST_K_ITEMS:DIM
  }

  def min(a: UInt, b: Uint) = (a < b) ? a : b
  def max(a: UInt, b: Uint) = (a > b) ? a : b

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
    is (s_RESET_OUTPUT_GROUP ) {
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
      loop1_A_mem_addr := A_MEM_ADDR;
      loop1_B_mem_addr := B_MEM_ADDR;
      loop1_C_mem_addr := C_MEM_ADDR;
      loop1_D_mem_addr := D_MEM_ADDR;

      // update next state
      state := s_RESET_A_TILE_SUBCOL 
    }
    is (s_RESET_A_TILE_SUBCOL ) {
    }
    is (s_MOVE_FIRST_B_INTO_SP ) {
    }
    is (s_RESET_B_TILE_SUBCOL_IN_SUBROW ) {
    }
    is (s_MAYBE_MOVE_NEXT_B_TILE_INTO_SP ) {
    }
    is (s_RESET_A_TILE_SUBROW_IN_SUBCOL ) {
    }
    is (s_MAYBE_MOVE_A_TILE_INTO_SP ) {
    }
    is (s_MAYBE_MOVE_D_TILE_INTO_ACC ) {
    }
    is (s_PRELOAD_B_TILE_INTO_ARRAY_AND_SET_C_ADDR_IN_ACC ) {
    }
    is (s_DO_MATMUL ) {
    }
    is (s_MAYBE_MOVE_C_TILE_INTO_MEM ) {
    }
    is (s_NEXT_A_TILE_SUBROW_IN_SUBCOL ) {
    }
    is (s_NEXT_B_TILE_SUBCOL_IN_SUBROW ) {
    }
    is (s_NEXT_A_TILE_SUBCOL ) {
    }
    is (s_NEXT_OUTPUT_GROUP ) {
      val did_row_incr := Bool()
      val did_col_incr := Bool()

      did_row_incr := false.B
      did_col_incr := false.B

      when (gbl_tile_col === TILE_COL_END) {
        when (gbl_tile_row === TILE_ROW_END) {
          // we finished the last output group. so we're done
          hwprintf("output_group finished")
          return false
        }
        .otherwise {
          did_row_incr := true.B
          gbl_tile_col := 0
          gbl_tile_row := gbl_tile_row + 1.U
          update_tile_dims(self)
        }
      }
      .otherwise {
        did_col_incr := true.B
        gbl_tile_col := gbl_tile_col + 1.U
        gbl_tile_row := loop1_tile_row_start
        update_tile_dims(self)
      }

      // reset global state that resets for each new output-group
      gbl_CD_acc_row_addr := 0.U

      // update the start/end tiles for this output-group (inclusive)
      loop1_tile_col_start := gbl_tile_col
      loop1_tile_col_end   := min(gbl_tile_col + TILE_COLS_PER_GROUP - 1.U,
                                 TILE_COL_END)
      loop1_tile_row_start := gbl_tile_row
      loop1_tile_row_end   := min(gbl_tile_row + TILE_ROWS_PER_GROUP - 1.U,
                                 TILE_ROW_END)

      // update all derived pointers to matrices in memory
      when (did_row_incr) {
        loop1_A_mem_addr  := A_MEM_ADDR + (loop1_tile_row_start *
                                  A_BYTES_PER_TILE_ROW)
        loop1_B_mem_addr  := B_MEM_ADDR
        loop1_C_mem_addr  := C_MEM_ADDR + (loop1_tile_row_start *
                                  C_BYTES_PER_TILE_ROW)
        loop1_D_mem_addr  := !HAS_BIAS ? 0 :
                                  (REPEATING_BIAS ? D_MEM_ADDR :
                                   D_MEM_ADDR + (loop1_tile_row_start *
                                   D_BYTES_PER_TILE_ROW))
      }
      .elsewhen (did_col_incr) {
        loop1_A_mem_addr += 0.U
        loop1_B_mem_addr += I_BYTE_COLS_PER_GROUP
        loop1_C_mem_addr += I_BYTE_COLS_PER_GROUP
        loop1_D_mem_addr += !HAS_BIAS ? 0.U : O_BYTE_COLS_PER_GROUP
      }

      // update next state
    }
  }
}

    reset_output_group(self);
    do {
      reset_A_tile_subcol(self);
      do {
        move_first_B_tile_into_sp(self);
        reset_B_tile_subcol_in_subrow(self);
        do {
          maybe_move_next_B_tile_into_sp(self);
          reset_A_tile_subrow_in_subcol(self);
          do {
            maybe_move_A_tile_into_sp(self);
            maybe_move_D_tile_into_acc(self);
            preload_B_tile_into_array_and_set_C_addr_in_acc(self);
            do_matmul(self);
            maybe_move_C_tile_into_mem(self);

          } while(next_A_tile_subrow_in_subcol(self));
        } while(next_B_tile_subcol_in_subrow(self));
      } while(next_A_tile_subcol(self));
    } while(next_output_group(self));

    // cleanup the state object
    destroy_gemmini(self);
    unpin_matrices();


object TilerController {
  def apply(enq: ReadyValidIO[RoCCCommand], block_size: Int)(implicit p: Parameters): DecoupledIO[RoCCCommand] = {
  //  val lu = Module(new LoopUnroller(block_size))
  //  lu.io.in <> enq
  //  lu.io.out
  //}
}
