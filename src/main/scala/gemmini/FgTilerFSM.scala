//===========================================================================
// TilerController's Internal FSM implementation
//===========================================================================
package gemmini

import scala.math.{pow,sqrt}
import chisel3._
import chisel3.util._
import chisel3.experimental._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import GemminiISA._

class FgTilerFSM[T <: Data : Arithmetic]
  (config: FgGemminiArrayConfig[T])(implicit val p: Parameters) 
  extends Module with HasCoreParameters {
  import config._

  //=========================================================================
  // interface
  //=========================================================================
  val io = IO(new Bundle {
    val cmd_in    = Flipped(Decoupled(new TilerCmd(OTYPE_BITS_IDX)))
    val sched_out = Decoupled(new RoCCCommand)
    val busy      = Output(Bool())
  })
  // hardcode 8 entries with up to 2 pushes per cycle
  val schedq = Module(new MultiTailedQueue(new RoCCCommand, 8, 2))
  io.sched_out <> schedq.io.deq

  val sched = schedq.io.enq
  val cmd   = io.cmd_in.bits
  val busy  = io.busy

  // initialize ports/pins
  for (i <- 0 to 1) {
    sched.bits(i) := DontCare
    sched.bits(i).status := io.cmd_in.bits.status
  }
  sched.push := 0.U
  io.cmd_in.ready := false.B
  busy := true.B

  //=========================================================================
  // FSM states (see diagram for what each state does)
  //=========================================================================
  val (s_IDLE ::
      s_FINISH_INIT1 ::
      s_FINISH_INIT2 ::
      s_FINISH_INIT3 ::
      s_FINISH_INIT4 ::
      s_RESET_OUTPUT_GROUP ::
      s_RESET_A_TILE_SUBCOL ::
      s_MOVE_FIRST_B_TILE_INTO_SP ::
      s_RESET_B_TILE_SUBCOL_IN_SUBROW ::
      s_MAYBE_MOVE_NEXT_B_TILE_INTO_SP ::
      s_RESET_A_TILE_SUBROW_IN_SUBCOL ::
      s_MAYBE_MOVE_A_TILE_INTO_SP ::
      s_MAYBE_MOVE_D_TILE_INTO_ACC ::
      s_PRELOAD_B_TILE_INTO_ARRAY_AND_SET_C_ADDR_IN_ACC ::
      s_DO_MATMUL ::
      s_MAYBE_MOVE_C_TILE_INTO_MEM ::
      s_NEXT_A_TILE_SUBROW_IN_SUBCOL ::
      s_NEXT_B_TILE_SUBCOL_IN_SUBROW ::
      s_NEXT_A_TILE_SUBCOL ::
      s_NEXT_OUTPUT_GROUP ::
      Nil) = Enum(20)

  val state = RegInit(s_IDLE)

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
  // fg-tiler-fsm hardwired constants
  //------------------------------------------------------------------------
  val I_BYTE_COLS_PER_TILE      = FG_DIM * FG_NUM * ITYPE_BYTES
  val O_BYTE_COLS_PER_TILE      = FG_DIM * FG_NUM * OTYPE_BYTES
  val I_BYTE_COLS_PER_GROUP     = I_BYTE_COLS_PER_TILE * FG_NUM
  val O_BYTE_COLS_PER_GROUP     = O_BYTE_COLS_PER_TILE * FG_NUM
  val I_BYTE_COLS_PER_TILE_CTR  = log2Up(I_BYTE_COLS_PER_TILE+1)
  val O_BYTE_COLS_PER_TILE_CTR  = log2Up(O_BYTE_COLS_PER_TILE+1)
  val I_BYTE_COLS_PER_GROUP_CTR = log2Up(I_BYTE_COLS_PER_GROUP+1)
  val O_BYTE_COLS_PER_GROUP_CTR = log2Up(O_BYTE_COLS_PER_GROUP+1)

  val BYTES_PER_ROW           = Int.MaxValue / DIM
  val BYTES_PER_ROW_CTR       = log2Up(BYTES_PER_ROW+1)
  val BYTES_PER_TILE_ROW      = BYTES_PER_ROW * DIM
  val BYTES_PER_TILE_ROW_CTR  = log2Up(BYTES_PER_TILE_ROW+1)
  val TILES                   = BYTES_PER_ROW / ITYPE_BYTES / (DIM/2)
  val TILES_IDX               = log2Up(TILES)
  val ELEMS_PER_TILE          = FG_DIM * FG_NUM
  val ELEMS_PER_TILE_CTR      = log2Up(ELEMS_PER_TILE+1)

  // TOTAL_ACC_TILES is how many square tiles can be output to accumulator
  val TOTAL_ACC_TILES = FG_NUM
  val SQRT_ACC_TILES  = sqrt(TOTAL_ACC_TILES).toInt
  val OG_HEIGHT_MAP   = (1 to TOTAL_ACC_TILES).sortWith((h1, h2) => {
    (h1 - SQRT_ACC_TILES).abs > (h2 - SQRT_ACC_TILES).abs
  })
  // list of numbers of fg-arrays that are <= sqrt(FG_NUM). used to determine
  // what the width/height of the tile is in terms of fine-grained tiles
  val FGS_LE_SQ_TILE = (0 to log2Up(SQRT_FG_NUM)).map { e=>math.pow(2,e) }

  //------------------------------------------------------------------------
  // input data-specific constants BEFORE TILE-SIZE CALCULATION
  //------------------------------------------------------------------------
  val g_HAS_BIAS              = Reg(Bool())
  val g_REPEATING_BIAS        = Reg(Bool())
  // exec-configs
  val g_DATAFLOW              = Reg(UInt(1.W))
  val g_ACTIVATION            = Reg(UInt(2.W))
  val g_SYSTOLIC_OUT_RSHIFT   = Reg(UInt(OTYPE_BITS_IDX.W))
  val g_ACC_OUT_RSHIFT        = Reg(UInt(OTYPE_BITS_IDX.W))
  val g_RELU6_IN_LSHIFT       = Reg(UInt(OTYPE_BITS_IDX.W))
  // mem-addr of A-matrix
  val g_A_MEM_ADDR            = Reg(UInt(xLen.W))
  val g_B_MEM_ADDR            = Reg(UInt(xLen.W))
  val g_C_MEM_ADDR            = Reg(UInt(xLen.W))
  val g_D_MEM_ADDR            = Reg(UInt(xLen.W))
  // bytes in A-matrix row
  val g_A_BYTES_PER_ROW       = Reg(UInt(BYTES_PER_ROW_CTR.W))
  val g_B_BYTES_PER_ROW       = Reg(UInt(BYTES_PER_ROW_CTR.W))
  val g_C_BYTES_PER_ROW       = Reg(UInt(BYTES_PER_ROW_CTR.W))
  val g_D_BYTES_PER_ROW       = Reg(UInt(BYTES_PER_ROW_CTR.W))
  // last (x,y,x) tile idx in (C,C,A) matrix
  val g_FG_TILE_ROW_END       = Reg(UInt(TILES_IDX.W))
  val g_FG_TILE_COL_END       = Reg(UInt(TILES_IDX.W))
  val g_FG_K_TILE_COL_END     = Reg(UInt(TILES_IDX.W))

  //------------------------------------------------------------------------
  // input data-specific constants AFTER TILE-SIZE CALCULATION (cycle 1)
  //------------------------------------------------------------------------
  val g_FG_TILE_ROWS_PER_TILE = Reg(UInt(FG_NUM_CTR.W))
  val g_FG_TILE_COLS_PER_TILE = Reg(UInt(FG_NUM_CTR.W)) 
  //------------------------------------------------------------------------
  // input data-specific constants AFTER TILE-SIZE CALCULATION (cycle 2)
  //------------------------------------------------------------------------
  val g_TILE_ROWS_PER_GROUP   = Reg(UInt(FG_NUM_CTR.W))
  val g_TILE_COLS_PER_GROUP   = Reg(UInt(FG_NUM_CTR.W)) 
  val g_ITEM_COLS_PER_TILE    = Reg(UInt(ELEMS_PER_TILE_CTR.W))
  val g_ITEM_ROWS_PER_TILE    = Reg(UInt(ELEMS_PER_TILE_CTR.W))
  //------------------------------------------------------------------------
  // input data-specific constants AFTER TILE-SIZE CALCULATION (cycle 3)
  //------------------------------------------------------------------------
  val g_I_BYTE_COLS_PER_TILE  = Reg(UInt(I_BYTE_COLS_PER_TILE_CTR.W))
  val g_O_BYTE_COLS_PER_TILE  = Reg(UInt(O_BYTE_COLS_PER_TILE_CTR.W))
  val g_I_BYTE_COLS_PER_GROUP = Reg(UInt(I_BYTE_COLS_PER_GROUP_CTR.W))
  val g_O_BYTE_COLS_PER_GROUP = Reg(UInt(I_BYTE_COLS_PER_GROUP_CTR.W))
  //------------------------------------------------------------------------
  // input data-specific constants AFTER TILE-SIZE CALCULATION (cycle 4)
  //------------------------------------------------------------------------
  // needed for 0-padding last rows/cols
  val g_LAST_M_ITEMS          = Reg(UInt(ELEMS_PER_TILE_CTR.W))
  val g_LAST_N_ITEMS          = Reg(UInt(ELEMS_PER_TILE_CTR.W))
  val g_LAST_K_ITEMS          = Reg(UInt(ELEMS_PER_TILE_CTR.W))
  // last (x,y,x) tile idx in (C,C,A) matrix
  val g_TILE_ROW_END          = Reg(UInt(TILES_IDX.W))
  val g_TILE_COL_END          = Reg(UInt(TILES_IDX.W))
  val g_K_TILE_COL_END        = Reg(UInt(TILES_IDX.W))
  // bytes in A-matrix row * rows-per-tile
  val g_A_BYTES_PER_TILE_ROW  = Reg(UInt(BYTES_PER_TILE_ROW_CTR.W))
  val g_B_BYTES_PER_TILE_ROW  = Reg(UInt(BYTES_PER_TILE_ROW_CTR.W))
  val g_C_BYTES_PER_TILE_ROW  = Reg(UInt(BYTES_PER_TILE_ROW_CTR.W))
  val g_D_BYTES_PER_TILE_ROW  = Reg(UInt(BYTES_PER_TILE_ROW_CTR.W))

  //------------------------------------------------------------------------
  // global state persistent across all loops
  //------------------------------------------------------------------------
  // current output-group tile (y,x)
  val (gbl_tile_row, gbl_tile_row_n) = regwire(TILES_IDX)
  val (gbl_tile_col, gbl_tile_col_n) = regwire(TILES_IDX)
  // how many elements (tall,wide) is this tile
  val gbl_item_rows = Reg(UInt(ELEMS_PER_TILE_CTR.W))
  val gbl_item_cols = Reg(UInt(ELEMS_PER_TILE_CTR.W))
  // which tmp-slot in sp being used now, and which is the alternate
  val gbl_B_cur_row_addr = Reg(UInt(FG_DIM_CTR.W))
  val gbl_B_alt_row_addr = Reg(UInt(FG_DIM_CTR.W))

  //------------------------------------------------------------------------
  // global state that is reset for each output-group
  //------------------------------------------------------------------------
  // where to put next A-tile in the A-scratchpad
  val gbl_A_row_addr     = Reg(UInt(A_SP_ROWS_IDX.W))
  val gbl_A_fg_col_start = Reg(UInt(A_SP_FG_COLS_IDX.W))

  // where to put next C/D-tile in accumulator
  val gbl_CD_row_addr     = Reg(UInt(CD_ACC_ROWS_IDX.W))
  val gbl_CD_fg_col_start = Reg(UInt(CD_ACC_FG_COLS_IDX.W))

  //------------------------------------------------------------------------
  // loop1-local state
  //------------------------------------------------------------------------
  // (ul-x,ul-y,br-x,br-y) tile x in output-group
  val loop1_tile_col_start = Reg(UInt(TILES_IDX.W))
  val loop1_tile_col_end   = Reg(UInt(TILES_IDX.W))
  val loop1_tile_row_start = Reg(UInt(TILES_IDX.W))
  val loop1_tile_row_end   = Reg(UInt(TILES_IDX.W))
  // initialized from global-constants
  val loop1_A_mem_addr = Reg(UInt(xLen.W))
  val loop1_B_mem_addr = Reg(UInt(xLen.W))
  val loop1_C_mem_addr = Reg(UInt(xLen.W))
  val loop1_D_mem_addr = Reg(UInt(xLen.W))

  //------------------------------------------------------------------------
  // loop2-local state
  //------------------------------------------------------------------------
  // which tile-column in A we are in
  val (loop2_k_tile_col, loop2_k_tile_col_n) = regwire(TILES_IDX)
  // how many elems in k-dim is this tile
  val loop2_k_item_dims = Reg(UInt(FG_DIM_IDX.W))
  // initialized from loop1 values
  val loop2_A_mem_addr = Reg(UInt(xLen.W))
  val loop2_B_mem_addr = Reg(UInt(xLen.W))
  val loop2_C_mem_addr = Reg(UInt(xLen.W))
  val loop2_D_mem_addr = Reg(UInt(xLen.W))

  //------------------------------------------------------------------------
  // loop3-local state
  //------------------------------------------------------------------------
  // initialized from loop2 values
  val loop3_A_mem_addr = Reg(UInt(xLen.W))
  val loop3_B_mem_addr = Reg(UInt(xLen.W))
  val loop3_C_mem_addr = Reg(UInt(xLen.W))
  val loop3_D_mem_addr = Reg(UInt(xLen.W))

  //------------------------------------------------------------------------
  // loop4-local state
  //------------------------------------------------------------------------
  // initialized from loop3 values
  val loop4_A_mem_addr = Reg(UInt(xLen.W))
  val loop4_B_mem_addr = Reg(UInt(xLen.W))
  val loop4_C_mem_addr = Reg(UInt(xLen.W))
  val loop4_D_mem_addr = Reg(UInt(xLen.W))

  //=========================================================================
  // utilies used by FSM core
  //=========================================================================

  // continuous assigns (only added in the switch-cases that call this!)
  def update_tile_dims(dummy: Int = 0) = {
    gbl_item_rows     := Mux(gbl_tile_row_n === g_TILE_ROW_END, 
                             g_LAST_M_ITEMS, g_ITEM_ROWS_PER_TILE)
    gbl_item_cols     := Mux(gbl_tile_col_n === g_TILE_COL_END, 
                             g_LAST_N_ITEMS, g_ITEM_COLS_PER_TILE)
    loop2_k_item_dims := Mux(loop2_k_tile_col_n === g_K_TILE_COL_END,
                             g_LAST_K_ITEMS, FG_DIM.U)
  }

  def MIN(a: UInt, b: UInt) = Mux(a < b, a, b)

  //=========================================================================
  // FSM core
  //=========================================================================
  switch (state) {
    is (s_IDLE) {
      val l_HAS_BIAS      = (cmd.addr_d =/= 0.U)
      val l_A_BYTE_WIDTH  = WireDefault(cmd.k << ITYPE_BYTES_IDX.U)
      val l_BC_BYTE_WIDTH = WireDefault(cmd.n << ITYPE_BYTES_IDX.U)
      val l_D_BYTE_WIDTH  = WireDefault(cmd.n << OTYPE_BYTES_IDX.U)

      g_HAS_BIAS              := l_HAS_BIAS
      g_REPEATING_BIAS        := cmd.repeating_bias

      g_DATAFLOW              := Dataflow.WS.id.U
      g_ACTIVATION            := cmd.activation
      g_SYSTOLIC_OUT_RSHIFT   := cmd.in_rshift
      g_ACC_OUT_RSHIFT        := cmd.acc_rshift
      g_RELU6_IN_LSHIFT       := cmd.relu6_lshift

      g_A_MEM_ADDR            := cmd.addr_a
      g_B_MEM_ADDR            := cmd.addr_b
      g_C_MEM_ADDR            := cmd.addr_c
      g_D_MEM_ADDR            := cmd.addr_d

      g_A_BYTES_PER_ROW       := l_A_BYTE_WIDTH
      g_B_BYTES_PER_ROW       := l_BC_BYTE_WIDTH
      g_C_BYTES_PER_ROW       := l_BC_BYTE_WIDTH
      g_D_BYTES_PER_ROW       := l_D_BYTE_WIDTH

      //-------------------------------------------------------------------
      // need shape of tile to determine this!
      //-------------------------------------------------------------------
      g_FG_TILE_ROW_END   := (cmd.m >> FG_DIM_IDX) + 
                              cmd.m(FG_DIM_IDX-1,0).orR - 1.U
      g_FG_TILE_COL_END   := (cmd.n >> FG_DIM_IDX) + 
                              cmd.n(FG_DIM_IDX-1,0).orR - 1.U
      g_FG_K_TILE_COL_END := (cmd.k >> FG_DIM_IDX) + 
                              cmd.k(FG_DIM_IDX-1,0).orR - 1.U

      // update next state
      state := s_FINISH_INIT1
    }
    is (s_FINISH_INIT1) {
      // set the number of fg-tiles per row/col of the tile 
      g_FG_TILE_ROWS_PER_TILE := MuxCase(SQRT_FG_NUM.U,
        (g_FG_TILE_ROW_END < SQRT_FG_NUM.U) -> Mux1H(FGS_LE_SQ_FG_NUM.map { 
          h => (g_FG_TILE_ROW_END < h.U) -> h.U
        }),
        (g_FG_TILE_COL_END < SQRT_FG_NUM.U) -> Mux1H(FGS_LE_SQ_FG_NUM.map { 
          w => (g_FG_TILE_COL_END < w.U) -> (FG_NUM/w).U
        }))
      g_FG_TILE_COLS_PER_TILE := MuxCase(SQRT_FG_NUM.U,
        (g_FG_TILE_ROW_END < SQRT_FG_NUM.U) -> Mux1H(FGS_LE_SQ_FG_NUM.map { 
          h => (g_FG_TILE_ROW_END < h.U) -> (FG_NUM/h).U
        }),
        (g_FG_TILE_COL_END < SQRT_FG_NUM.U) -> Mux1H(FGS_LE_SQ_FG_NUM.map { 
          w => (g_FG_TILE_COL_END < w.U) -> w.U
        }))

      // update next state
      state := s_FINISH_INIT2
    }
    is (s_FINISH_INIT2) {
      //-------------------------------------------------------------------
      // set tile shape and output-group shape (both in terms of fg-tiles)
      //-------------------------------------------------------------------
      val l_SQ_TILE_ROW_END = (cmd.m >> DIM_IDX) + 
                               cmd.m(DIM_IDX-1,0).orR - 1.U
      val l_SQ_TILE_COL_END = (cmd.n >> DIM_IDX) + 
                               cmd.n(DIM_IDX-1,0).orR - 1.U
      val l_OG_DIM_SELECT = OG_HEIGHT_MAP.zipWithIndex.map{ case(h,i) => 
        val w = TOTAL_ACC_TILES/h
        if (h < w)      WireDefault(l_SQ_TILE_ROW_END < h.U)
        else if(h > w)  WireDefault(l_SQ_TILE_COL_END < w.U)
        else            WireDefault((i == (OG_HEIGHT_MAP.size-1)).asBool)
      }
      val l_SQ_TILE_ROWS_PER_GROUP = MuxCase(
        OG_HEIGHT_MAP(OG_HEIGHT_MAP.size-1).U,
        OG_HEIGHT_MAP.zipWithIndex.map{
          case(h,i) => (l_OG_DIM_SELECT(i) -> h.U)
        })
      val l_SQ_TILE_COLS_PER_GROUP = MuxCase(
        (TOTAL_ACC_TILES / OG_HEIGHT_MAP(OG_HEIGHT_MAP.size-1)).U,
        OG_HEIGHT_MAP.zipWithIndex.map{
          case(h,i) => (l_OG_DIM_SELECT(i) -> (TOTAL_ACC_TILES/h).U)
        })

      val l_IS_SKINNY_TALL_TILE = (g_FG_TILE_COLS_PER_TILE < SQRT_FG_NUM.U)
      val l_IS_WIDE_SHORT_TILE  = (g_FG_TILE_COLS_PER_TILE > SQRT_FG_NUM.U)

      val l_SQ_TILE_ROWS_PER_TILE = Mux(g_IS_WIDE_SHORT_TILE, 1.U,
                                     g_FG_TILE_ROWS_PER_TILE / SQRT_FG_NUM.U)
      val l_SQ_TILE_COLS_PER_TILE = Mux(l_IS_SKINNY_TALL_TILE, 1.U,
                                     g_FG_TILE_COLS_PER_TILE / SQRT_FG_NUM.U)

      g_TILE_ROWS_PER_GROUP := l_SQ_TILE_ROWS_PER_GROUP / 
                               l_SQ_TILE_ROWS_PER_TILE
      g_TILE_COLS_PER_GROUP := l_SQ_TILE_COLS_PER_GROUP / 
                               l_SQ_TILE_COLS_PER_TILE

      g_ITEM_COLS_PER_TILE := g_FG_TILE_COLS_PER_TILE * FG_DIM.U
      g_ITEM_ROWS_PER_TILE := g_FG_TILE_ROWS_PER_TILE * FG_DIM.U

      // update next state
      state := s_FINISH_INIT3
    }
    is (s_FINISH_INIT3) {
      val l_FG_TILE_COLS_PER_GROUP = g_TILE_COLS_PER_GROUP *
                                     g_FG_TILE_COLS_PER_TILE 
      val l_ITEM_COLS_PER_GROUP    = l_FG_TILE_COLS_PER_GROUP * FG_DIM.U

      g_I_BYTE_COLS_PER_GROUP := l_ITEM_COLS_PER_GROUP * ITYPE_BYTES.U
      g_O_BYTE_COLS_PER_GROUP := l_ITEM_COLS_PER_GROUP * OTYPE_BYTES.U

      g_I_BYTE_COLS_PER_TILE := g_ITEM_COLS_PER_TILE * ITYPE_BYTES.U
      g_O_BYTE_COLS_PER_TILE := g_ITEM_COLS_PER_TILE * OTYPE_BYTES.U

      // update next state
      state := s_FINISH_INIT4
    }
    is (s_FINISH_INIT4) {
      //-------------------------------------------------------------------
      // how many items on last iteration
      // - TODO: don't use modulus here
      //-------------------------------------------------------------------
      val l_extra_m_elems = cmd.m % g_ITEM_ROWS_PER_TILE
      val l_extra_n_elems = cmd.n % g_ITEM_COLS_PER_TILE
      val l_extra_k_elems = cmd.k % FG_DIM.U
      g_LAST_M_ITEMS := Mux(l_extra_m_elems.orR, l_extra_m_elems, 
                            g_ITEM_ROWS_PER_TILE)
      g_LAST_N_ITEMS := Mux(l_extra_n_elems.orR, l_extra_n_elems, 
                            g_ITEM_COLS_PER_TILE)
      g_LAST_K_ITEMS := Mux(l_extra_k_elems.orR, l_extra_k_elems, 
                            FG_DIM.U)

      //-------------------------------------------------------------------
      // for calculating DRAM addresses each iteration
      //-------------------------------------------------------------------
      g_A_BYTES_PER_TILE_ROW  := l_A_BYTE_WIDTH  * g_ITEM_ROWS_PER_TILE 
      g_B_BYTES_PER_TILE_ROW  := l_BC_BYTE_WIDTH * g_ITEM_ROWS_PER_TILE 
      g_C_BYTES_PER_TILE_ROW  := l_BC_BYTE_WIDTH * g_ITEM_ROWS_PER_TILE 
      g_D_BYTES_PER_TILE_ROW  := l_D_BYTE_WIDTH  * g_ITEM_ROWS_PER_TILE 

      //-------------------------------------------------------------------
      // the last tile indexes
      // - TODO: don't use division here
      //-------------------------------------------------------------------
      g_TILE_ROW_END   := (cmd.m/g_ITEM_ROWS_PER_TILE) + l_extra_m_elems.orR
      g_TILE_COL_END   := (cmd.n/g_ITEM_COLS_PER_TILE) + l_extra_n_elems.orR
      g_K_TILE_COL_END := (cmd.k/FG_DIM.U)             + l_extra_k_elems.orR

      //-------------------------------------------------------------------
      // update interface signals. we are only ready when an input cmd is
      // ready AND the output queue has 2 slots available to write to
      //-------------------------------------------------------------------
      io.cmd_in.ready := (sched.ready >= 2.U)

      // issue gemmini commands
      when(io.cmd_in.fire()) {
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
        state := s_RESET_OUTPUT_GROUP
      }
      .otherwise {
        // if we are sitting in idle state, we are not busy!
        busy := false.B
      }
    }
    //=======================================================================
    is (s_RESET_OUTPUT_GROUP) {
      // define mutable gbl state, persist across all ogs
      gbl_tile_row_n     := 0.U
      gbl_tile_col_n     := 0.U
      gbl_B_cur_row_addr := 0.U
      gbl_B_alt_row_addr := FG_DIM.U
      update_tile_dims()

      // define mutable gbl state, reset after each og
      gbl_CD_row_addr     := 0.U
      gbl_CD_fg_col_start := 0.U

      loop1_tile_col_start := gbl_tile_col_n
      loop1_tile_col_end   := MIN(gbl_tile_col_n + g_TILE_COLS_PER_GROUP-1.U,
                                  g_TILE_COL_END)
      loop1_tile_row_start := gbl_tile_row_n
      loop1_tile_row_end   := MIN(gbl_tile_row_n + g_TILE_ROWS_PER_GROUP-1.U,
                                  g_TILE_ROW_END)

      // derived pointers to matrices in memory for this og
      loop1_A_mem_addr := g_A_MEM_ADDR
      loop1_B_mem_addr := g_B_MEM_ADDR
      loop1_C_mem_addr := g_C_MEM_ADDR
      loop1_D_mem_addr := g_D_MEM_ADDR

      // update next state
      state := s_RESET_A_TILE_SUBCOL
    }
    //=======================================================================
    is (s_RESET_A_TILE_SUBCOL) {
      loop2_k_tile_col_n  := 0.U
      gbl_tile_row_n      := loop1_tile_row_start
      gbl_tile_col_n      := loop1_tile_col_start
      gbl_CD_row_addr     := 0.U
      gbl_CD_fg_col_start := 0.U
      update_tile_dims()

      loop2_A_mem_addr := loop1_A_mem_addr
      loop2_B_mem_addr := loop1_B_mem_addr
      loop2_C_mem_addr := loop1_C_mem_addr
      loop2_D_mem_addr := loop1_D_mem_addr

      // update next state
      state := s_MOVE_FIRST_B_TILE_INTO_SP
    }
    //=======================================================================
    is (s_MOVE_FIRST_B_TILE_INTO_SP) {
      // calculate mvin parameters
      val B_mem_addr    = loop2_B_mem_addr
      val B_mem_stride  = g_B_BYTES_PER_ROW

      val configB = Wire(new FgConfigRs1)
      configB.garbage := 0.U
      configB.is_acc  := false.B
      configB.is_B_sp := true.B
      configB.cfgtype := CONFIG_LOAD

      val rangeB = Wire(new FgLocalRange(config))
      rangeB.rows         := loop2_k_item_dims
      rangeB.cols         := gbl_item_cols
      rangeB.is_acc       := false.B
      rangeB.is_accum     := false.B
      rangeB.is_B_sp      := true.B
      rangeB.garbage      := false.B
      rangeB.fg_col_start := 0.U
      rangeB.row_start    := gbl_B_cur_row_addr

      // issue gemmini commands
      when(sched.ready >= 2.U) {
        sched.push               := 2.U
        sched.bits(0).rs1        := configB.asUInt()
        sched.bits(0).rs2        := B_mem_stride
        sched.bits(0).inst.funct := CONFIG_CMD
        sched.bits(1).rs1        := B_mem_addr
        sched.bits(1).rs2        := rangeB.asUInt()
        sched.bits(1).inst.funct := LOAD_CMD

        // update next state
        state := s_RESET_B_TILE_SUBCOL_IN_SUBROW
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
    }
    //=======================================================================
    is (s_MAYBE_MOVE_NEXT_B_TILE_INTO_SP) {
      // calculate mvin parameters
      val B_mem_addr    = loop3_B_mem_addr + g_I_BYTE_COLS_PER_TILE
      val B_mem_stride  = g_B_BYTES_PER_ROW

      val configB = Wire(new FgConfigRs1)
      configB.garbage := 0.U
      configB.is_acc  := false.B
      configB.is_B_sp := true.B
      configB.cfgtype := CONFIG_LOAD

      val rangeB = Wire(new FgLocalRange(config))
      rangeB.rows         := loop2_k_item_dims
      rangeB.cols         := Mux(gbl_tile_col === g_TILE_COL_END-1.U,
                              g_LAST_N_ITEMS, 
                              g_ITEM_COLS_PER_TILE)
      rangeB.is_acc       := false.B
      rangeB.is_accum     := false.B
      rangeB.is_B_sp      := true.B
      rangeB.garbage      := false.B
      rangeB.fg_col_start := 0.U
      rangeB.row_start    := gbl_B_alt_row_addr


      // can't load next B-tile if we are already on the last one
      when (gbl_tile_col === loop1_tile_col_end) {
        state := s_RESET_A_TILE_SUBROW_IN_SUBCOL
      }
      .elsewhen (sched.ready >= 2.U) {
        sched.push               := 2.U
        sched.bits(0).rs1        := configB.asUInt()
        sched.bits(0).rs2        := B_mem_stride
        sched.bits(0).inst.funct := CONFIG_CMD
        sched.bits(1).rs1        := B_mem_addr
        sched.bits(1).rs2        := rangeB.asUInt()
        sched.bits(1).inst.funct := LOAD_CMD

        // update next state
        state := s_RESET_A_TILE_SUBROW_IN_SUBCOL
      }
    }
    //=======================================================================
    is (s_RESET_A_TILE_SUBROW_IN_SUBCOL) {
      // this scope modifies: gbl_tile_row
      //                      gbl_CD_acc_row_addr
      loop4_A_mem_addr := loop3_A_mem_addr
      loop4_B_mem_addr := loop3_B_mem_addr
      loop4_C_mem_addr := loop3_C_mem_addr
      loop4_D_mem_addr := loop3_D_mem_addr

      gbl_A_row_addr     := 0.U
      gbl_A_fg_col_start := 0.U

      // update next state
      state := s_MAYBE_MOVE_A_TILE_INTO_SP
    }
    //=======================================================================
    is (s_MAYBE_MOVE_A_TILE_INTO_SP) {
      // calculate mvin parameters
      val configA = Wire(new FgConfigRs1)
      configA.garbage := 0.U
      configA.is_acc  := false.B
      configA.is_B_sp := false.B
      configA.cfgtype := CONFIG_LOAD

      val rangeA = Wire(new FgLocalRange(config))
      rangeA.rows         := gbl_item_rows
      rangeA.cols         := loop2_k_item_dims
      rangeA.is_acc       := true.B
      rangeA.is_accum     := false.B
      rangeA.is_B_sp      := false.B
      rangeA.garbage      := false.B
      rangeA.fg_col_start := gbl_A_fg_col_start
      rangeA.row_start    := gbl_A_row_addr

      // only move A-tiles in during first column of tiles in the og
      when (gbl_tile_col =/= loop1_tile_col_start) {
        state := s_MAYBE_MOVE_D_TILE_INTO_ACC
      }
      .elsewhen (sched.ready >= 2.U) {
        sched.push               := 2.U
        sched.bits(0).rs1        := configA.asUInt()
        sched.bits(0).rs2        := A_mem_stride
        sched.bits(0).inst.funct := CONFIG_CMD
        sched.bits(1).rs1        := A_mem_addr
        sched.bits(1).rs2        := rangeA.asUInt()
        sched.bits(1).inst.funct := LOAD_CMD

        // update next state
        state := s_MAYBE_MOVE_D_TILE_INTO_ACC
      }
    }
    //=======================================================================
    is (s_MAYBE_MOVE_D_TILE_INTO_ACC) {
      // calculate mvin parameters (NOTE: we know D is valid at this point)
      val D_mem_addr   = loop4_D_mem_addr
      val D_mem_stride = Mux(g_REPEATING_BIAS, 0.U, g_D_BYTES_PER_ROW)

      val configD = Wire(new FgConfigRs1)
      configD.garbage := 0.U
      configD.is_acc  := true.B
      configD.is_B_sp := false.B
      configD.cfgtype := CONFIG_LOAD

      val rangeD = Wire(new FgLocalRange(config))
      rangeD.rows         := gbl_item_rows
      rangeD.cols         := gbl_item_cols
      rangeD.is_acc       := true.B
      rangeD.is_accum     := false.B
      rangeD.is_B_sp      := false.B
      rangeD.garbage      := false.B
      rangeD.fg_col_start := gbl_CD_fg_col_start
      rangeD.row_start    := gbl_CD_row_addr


      // only move D-tiles in during first partial-sum in an output-group
      when((loop2_k_tile_col =/= 0.U) || !g_HAS_BIAS) {
        state := s_PRELOAD_B_TILE_INTO_ARRAY_AND_SET_C_ADDR_IN_ACC
      }
      .elsewhen (sched.ready >= 2.U) {
        sched.push               := 2.U
        sched.bits(0).rs1        := configD.asUInt()
        sched.bits(0).rs2        := D_mem_stride
        sched.bits(0).inst.funct := CONFIG_CMD
        sched.bits(1).rs1        := D_mem_addr
        sched.bits(1).rs2        := rangeD.asUInt()
        sched.bits(1).inst.funct := LOAD_CMD

        // update next state
        state := s_PRELOAD_B_TILE_INTO_ARRAY_AND_SET_C_ADDR_IN_ACC
      }
    }
    //=======================================================================
    is (s_PRELOAD_B_TILE_INTO_ARRAY_AND_SET_C_ADDR_IN_ACC) {
      // on first tile in 4th loop: preload this B-tile
      // else:                      preload garbage B-tile (no spad load)
      val rangeB = Wire(new FgLocalRange(config))
      rangeB.rows         := loop2_k_item_dims
      rangeB.cols         := gbl_item_cols
      rangeB.is_acc       := false.B
      rangeB.is_accum     := false.B
      rangeB.is_B_sp      := true.B
      rangeB.garbage      := (gbl_tile_row =/= loop1_tile_row_start)
      rangeB.fg_col_start := 0.U
      rangeB.row_start    := gbl_B_cur_row_addr

      // if has D-bias already loaded: accumulate c in accumulator
      // elif first k-col in 2nd loop: overwrite c in accumulator
      // else:                         accumulate c in accumulator
      val rangeC = Wire(new FgLocalRange(config))
      rangeC.rows         := gbl_item_rows
      rangeC.cols         := gbl_item_cols
      rangeC.is_acc       := true.B
      rangeC.is_accum     := (g_HAS_BIAS || (loop2_k_tile_col > 0.U))
      rangeC.is_B_sp      := false.B
      rangeC.garbage      := false.B
      rangeC.fg_col_start := gbl_CD_fg_col_start
      rangeC.row_start    := gbl_CD_row_addr

      when (sched.ready >= 1.U) {
        sched.push               := 1.U
        sched.bits(0).rs1        := rangeB.asUInt()
        sched.bits(0).rs2        := rangeC.asUInt()
        sched.bits(0).inst.funct := PRELOAD_CMD

        // update next state
        state := s_DO_MATMUL
      }
    }
    //=======================================================================
    is (s_DO_MATMUL) {
      // calculate compute parameters
      val rangeA = Wire(new FgLocalRange(config))
      rangeA.rows         := gbl_item_rows
      rangeA.cols         := loop2_k_item_dims
      rangeA.is_acc       := false.B
      rangeA.is_accum     := false.B
      rangeA.is_B_sp      := false.B
      rangeA.garbage      := false.B
      rangeA.fg_col_start := gbl_A_fg_col_start
      rangeA.row_start    := gbl_A_row_addr

      // don't care about D
      val rangeD = Wire(new FgLocalRange(config))
      rangeD := DontCare

      // on first tile in 4th loop: compute_preloaded
      // else: compute_accumulated
      when (sched.ready >= 1.U) {
        sched.push               := 1.U
        sched.bits(0).rs1        := rangeA.asUInt()
        sched.bits(0).rs2        := rangeD.asUInt()
        sched.bits(0).inst.funct := Mux(gbl_tile_row === loop1_tile_row_start,
                                        COMPUTE_AND_FLIP_CMD,
                                        COMPUTE_AND_STAY_CMD)
        // update next state
        state := s_MAYBE_MOVE_C_TILE_INTO_MEM
      }
    }
    //=======================================================================
    is (s_MAYBE_MOVE_C_TILE_INTO_MEM) {
      val C_mem_addr     = loop4_C_mem_addr
      val C_mem_stride   = g_C_BYTES_PER_ROW

      val rangeC = Wire(new FgLocalRange(config))
      rangeC.rows         := gbl_item_rows
      rangeC.cols         := gbl_item_cols
      rangeC.is_acc       := true.B
      rangeC.is_accum     := false.B
      rangeC.is_B_sp      := false.B
      rangeC.garbage      := false.B
      rangeC.fg_col_start := gbl_CD_fg_col_start
      rangeC.row_start    := gbl_CD_row_addr

      when(loop2_k_tile_col =/= g_K_TILE_COL_END) {
        state := s_NEXT_A_TILE_SUBROW_IN_SUBCOL
      }
      .elsewhen (sched.ready >= 1.U) {
        sched.push               := 1.U
        sched.bits(0).rs1        := C_mem_addr
        sched.bits(0).rs2        := rangeC.toUInt()
        sched.bits(0).inst.funct := STORE_CMD

        // update next state
        state := s_NEXT_A_TILE_SUBROW_IN_SUBCOL
      }
    }
    //=======================================================================
    is (s_NEXT_A_TILE_SUBROW_IN_SUBCOL) {
      when (gbl_tile_row === loop1_tile_row_end) {
        // just finished the final row of tiles in the 4th loop
        state := s_NEXT_B_TILE_SUBCOL_IN_SUBROW
      }
      .otherwise {
        val gbl_A_row_addr_next  = gbl_A_row_addr + g_ITEM_ROWS_PER_TILE
        val gbl_CD_row_addr_next = gbl_CD_row_addr + g_ITEM_ROWS_PER_TILE

        // modify global state
        gbl_tile_row_n  := gbl_tile_row + 1.U
        gbl_A_row_addr  := gbl_A_row_addr_next
        gbl_CD_row_addr := gbl_CD_row_addr_next
        when (gbl_A_row_addr_next === 0.U) {
          gbl_A_fg_col_start := gbl_A_fg_col_start + 1.U
        }
        when (gbl_CD_row_addr_next === 0.U) {
          gbl_CD_fg_col_start := gbl_CD_fg_col_start + SQ_FG_NUM.U
        }
        update_tile_dims()

        // modify loop4-local state
        loop4_A_mem_addr := loop4_A_mem_addr + g_A_BYTES_PER_TILE_ROW
        loop4_B_mem_addr := loop4_B_mem_addr
        loop4_C_mem_addr := loop4_C_mem_addr + g_C_BYTES_PER_TILE_ROW
        loop4_D_mem_addr := loop4_D_mem_addr +
                             Mux(g_HAS_BIAS && !g_REPEATING_BIAS,
                                 g_D_BYTES_PER_TILE_ROW, 0.U)

        // update next state
        state := s_MAYBE_MOVE_A_TILE_INTO_SP
      }
    }
    //=======================================================================
    is (s_NEXT_B_TILE_SUBCOL_IN_SUBROW) {
      when (gbl_tile_col === loop1_tile_col_end) {
        // we have already done the last column in the output-group
        state := s_NEXT_A_TILE_SUBCOL
      }
      .otherwise {
        // if we get here, we know the tile is SQ_FG_NUM padded fg-tiles tall
        val gbl_CD_row_addr_next = gbl_CD_row_addr + (SQ_FG_NUM.U * FG_DIM.U)

        // modify global state
        gbl_tile_row_n     := loop1_tile_row_start
        gbl_tile_col_n     := gbl_tile_col + 1.U
        gbl_A_row_addr     := 0.U
        gbl_CD_row_addr    := gbl_CD_row_addr_next
        gbl_A_fg_col_start := 0.U
        when (gbl_CD_row_addr_next === 0.U) {
          gbl_CD_fg_col_start := gbl_CD_fg_col_start + g_FG_TILE_COLS_PER_TILE
        }
        update_tile_dims()

        gbl_B_cur_row_addr := gbl_B_alt_row_addr
        gbl_B_alt_row_addr := gbl_B_cur_row_addr

        // modify loop3-local state
        loop3_A_mem_addr := loop3_A_mem_addr + 0.U
        loop3_B_mem_addr := loop3_B_mem_addr + g_I_BYTE_COLS_PER_TILE
        loop3_C_mem_addr := loop3_C_mem_addr + g_I_BYTE_COLS_PER_TILE
        loop3_D_mem_addr := loop3_D_mem_addr + g_O_BYTE_COLS_PER_TILE

        // update next state
        state := s_MAYBE_MOVE_NEXT_B_TILE_INTO_SP
      }
    }
    //=======================================================================
    is (s_NEXT_A_TILE_SUBCOL) {
      when (loop2_k_tile_col === g_K_TILE_COL_END) {
        state := s_NEXT_OUTPUT_GROUP
      }
      .otherwise {
        loop2_k_tile_col_n := loop2_k_tile_col + 1.U
        gbl_tile_row_n     := loop1_tile_row_start
        gbl_tile_col_n     := loop1_tile_col_start
        gbl_CD_row_addr    := 0.U
        update_tile_dims()

        loop2_A_mem_addr := loop2_A_mem_addr + g_I_BYTE_COLS_PER_TILE
        loop2_B_mem_addr := loop2_B_mem_addr + g_B_BYTES_PER_TILE_ROW
        loop2_C_mem_addr := loop2_C_mem_addr + 0.U
        loop2_D_mem_addr := loop2_D_mem_addr + 0.U

        // swap current/alternate B-tile scratchpad addrs
        gbl_B_cur_row_addr := gbl_B_alt_row_addr
        gbl_B_alt_row_addr := gbl_B_cur_row_addr

        // update next state
        state := s_MOVE_FIRST_B_TILE_INTO_SP
      }
    }
    //=======================================================================
    is (s_NEXT_OUTPUT_GROUP) {
      val l_did_row_incr = WireDefault(false.B)
      val l_did_col_incr = WireDefault(false.B)

      when (gbl_tile_col === g_TILE_COL_END && 
            gbl_tile_row === g_TILE_ROW_END) {
        // update next state
        state := s_IDLE
      }
      .otherwise {
        when (gbl_tile_col === g_TILE_COL_END) {
          l_did_row_incr := true.B
          gbl_tile_row_n := gbl_tile_row + 1.U
          gbl_tile_col_n := 0.U
          update_tile_dims()
        } .otherwise {
          l_did_col_incr := true.B
          gbl_tile_row_n := gbl_tile_row
          gbl_tile_col_n := gbl_tile_col + 1.U
          update_tile_dims()
        }
   
        // reset global state that resets for each new output-group
        gbl_CD_row_addr     := 0.U
        gbl_CD_fg_col_start := 0.U

        // update the start/end tiles for this output-group (inclusive)
        val l_tile_col_start = gbl_tile_col_n
        val l_tile_col_end  = MIN(gbl_tile_col_n + g_TILE_COLS_PER_GROUP-1.U,
                                  g_TILE_COL_END)
        val l_tile_row_start = gbl_tile_row_n
        val l_tile_row_end  = MIN(gbl_tile_row_n + g_TILE_ROWS_PER_GROUP-1.U,
                                  g_TILE_ROW_END)

        loop1_tile_col_start := l_tile_col_start
        loop1_tile_col_end   := l_tile_col_end
                                
        loop1_tile_row_start := l_tile_row_start
        loop1_tile_row_end   := l_tile_row_end
         
        // update all derived pointers to matrices in memory
        when(l_did_row_incr) {
          loop1_A_mem_addr := g_A_MEM_ADDR + (l_tile_row_start *
                                              g_A_BYTES_PER_TILE_ROW)
          loop1_B_mem_addr := g_B_MEM_ADDR
          loop1_C_mem_addr := g_C_MEM_ADDR + (l_tile_row_start *
                                              g_C_BYTES_PER_TILE_ROW)
          loop1_D_mem_addr := Mux(!g_HAS_BIAS, 0.U,
                                Mux(g_REPEATING_BIAS, g_D_MEM_ADDR,
                                 (g_D_MEM_ADDR + (l_tile_row_start *
                                                  g_D_BYTES_PER_TILE_ROW))))
        }
        .elsewhen (l_did_col_incr) {
          loop1_A_mem_addr := loop1_A_mem_addr + 0.U
          loop1_B_mem_addr := loop1_B_mem_addr + g_I_BYTE_COLS_PER_GROUP
          loop1_C_mem_addr := loop1_C_mem_addr + g_I_BYTE_COLS_PER_GROUP
          loop1_D_mem_addr := loop1_D_mem_addr + 
                              Mux(!g_HAS_BIAS, 0.U, g_O_BYTE_COLS_PER_GROUP)
        }

        // update next state
        state := s_RESET_A_TILE_SUBCOL
      }
    }
  }
}

object FgTilerFSM {
  def apply[T <: Data: Arithmetic]
    (config: FgGemminiArrayConfig[T])(implicit p: Parameters)
      = Module(new FgTilerFSM(config))
}
