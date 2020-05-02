package gemmini

import chisel3._
import chisel3.util._

class FgAccumulatorBankReadReq[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en           = Output(Bool())
  val row          = Output(UInt(FG_DIM_IDX.W))
  val fg_col_start = Output(UInt(FG_NUM_IDX.W))
}

class FgAccumulatorBankReadResp[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val data = Output(UInt(C_STORE_ROW_BITS.W))
}

class FgAccumulatorBankReadIO[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val req = new FgAccumulatorBankReadReq(config))
  val resp = Flipped(new FgAccumulatorBankReadResp(config))
}

class FgAccumulatorBankWriteReq[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en           = Output(Bool())
  val row          = Output(UInt(FG_DIM_IDX.W))
  val cols         = Output(UInt(CD_ACC_ROW_ELEMS_CTR.W))
  val fg_col_start = Output(UInt(FG_NUM_IDX.W))
  val data         = Output(UInt(D_LOAD_ROW_BITS.W))
  val accum        = Output(Bool())
}

//============================================================================
// accumulator configuration
//============================================================================
class FgAccumulatorBankConfigIO[T <: Data](config: FgGemminiArrayConfig[T]) 
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val shift      = Output(UInt(OTYPE_BITS_CTR.W))
  val relu_shift = Output(UInt(OTYPE_BITS_CTR.W))
  val act        = Output(UInt(2.W))
}

//==========================================================================
// Accumulator Bank
// - fg_col_start shifts for reads/writes done WITHIN THE BANK!
//==========================================================================
class FgAccumulatorBank[T <: Data: Arithmetic]
  (config: FgGemminiArrayConfig[T])
  (implicit p: Parameters, ev: Arithmetic[T]) extends Module {
  import config._
  import ev._
  val ROW_TYPE = Vec(CD_ACC_ROW_ELEMS, accType.getType)

  val io = IO(new Bundle {
    val read = Flipped(new FgAccumulatorBankReadIO(config))
    val write = Flipped(new FgAccumulatorBankWriteReq(config))
    val config = Flipped(new FgAccumulatorBankConfigIO(config))
  })
  val mem = SyncReadMem(FG_DIM, ROW_TYPE)

  //-------------------------------------
  // unpack write signals
  //-------------------------------------
  val wr_en           = io.write.en
  val wr_row          = io.write.row
  val wr_cols         = io.write.cols
  val wr_data         = io.write.data
  val wr_fg_col_start = io.write.fg_col_start
  val wr_accum        = io.write.accum
  val wr_elemshift    = (wr_fg_col_start * FG_DIM)
  val wr_data         = io.write.data.asTypeOf(ROW_TYPE) << wr_elemshift
  val wr_mask         = ((1.U << wr_cols) - 1.U) << wr_elemshift

  //-------------------------------------
  // unpack read signals
  //-------------------------------------
  val rd_en           = io.read.en
  val rd_row          = io.read.row
  val rd_fg_col_start = io.read.fg_col_start
  val rd_shift        = io.config.req.shift
  val rd_relu6_shift  = io.config.req.relu6_shift
  val rd_act          = io.config.req.act

  //-------------------------------------
  // read from bank (for read or write req)
  //-------------------------------------
  val bank_rd_en  = wr_en || rd_en
  val bank_rd_row = Mux(wr_en, wr_row, rd_row)
  val bank_rdata  = mem.read(bank_row, bank_rd_en).asUInt()
  assert(!(wr_en && rd_en), "wr_en and rd_en in same cycle")

  //-------------------------------------
  // write the (possibly accumulated) data 1 cycle later
  //-------------------------------------
  val wr_en_buf    = ShiftRegister(wr_en,    1)
  val wr_row_buf   = ShiftRegister(wr_row,   1)
  val wr_data_buf  = ShiftRegister(wr_data,  1)
  val wr_mask_buf  = ShiftRegister(wr_mask,  1)
  val wr_accum_buf = ShiftRegister(wr_accum, 1)
  val wr_sum       = VecInit(bank_rdata zip wdata_buf).map {case (r,w)=> r+w }
  when (wr_en_buf) {
    mem.write(wr_row_buf, Mux(wr_accum_buf, wr_sum, wr_data_buf), wr_mask_buf)
  }

  //-------------------------------------
  // read/output the activated data 2 cycles later
  //-------------------------------------
  val rd_en_buf           = ShiftRegister(rd_en,           1)
  val rd_row_buf          = ShiftRegister(rd_row,          1)
  val rd_fg_col_start_buf = ShiftRegister(rd_fg_col_start, 1)
  val rd_shift_buf        = ShiftRegister(rd_shift,        1)
  val rd_relu6_shift_buf  = ShiftRegister(rd_relu6_shift,  1)
  val rd_act_buf          = ShiftRegister(rd_act,          1)

  val rd_elemshift    = rd_fg_col_start_buf * FG_DIM
  val rd_shifted_data = bank_rdata >> rd_elemshift

  val activated_rdata = WireInit(VecInit(rd_shifted_data.map(e =>
    val e_clipped = (e >> rd_shift_buf).clippedToWidthOf(inputType)
    val e_act = MuxCase(e_clipped, Seq(
      (rd_act_buf === Activation.RELU) -> e_clipped.relu,
      (rd_act_buf === Activation.RELU6) -> e_clipped.relu6(rd_relu6_shift_buf)
    ))
    e_act
  )))
  io.read.resp.data := RegNext(activated_rdata)
}
