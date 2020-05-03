package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._

class FgAccumulatorBankReadReq[T <: Data](val config:FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en           = Output(Bool())
  val row          = Output(UInt(FG_DIM_IDX.W))
  val fg_col_start = Output(UInt(FG_NUM_IDX.W))
}

class FgAccumulatorBankReadResp[T <: Data](val config:FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val data = Output(UInt(C_STORE_ROW_BITS.W))
}

class FgAccumulatorBankReadIO[T <: Data](val config:FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val req = new FgAccumulatorBankReadReq(config)
  val resp = Flipped(new FgAccumulatorBankReadResp(config))
}

class FgAccumulatorBankWriteReq[T <: Data](val config:FgGemminiArrayConfig[T])
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
class FgAccumulatorBankConfigIO[T <: Data](val config:FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val in_rshift    = Output(UInt(OTYPE_BITS_IDX.W))
  val acc_rshift   = Output(UInt(OTYPE_BITS_IDX.W))
  val relu6_lshift = Output(UInt(OTYPE_BITS_IDX.W))
  val act          = Output(UInt(2.W))
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
  val ROW_TYPE = Vec(CD_ACC_ROW_ELEMS, accType)

  val io = IO(new Bundle {
    val read = Flipped(new FgAccumulatorBankReadIO(config))
    val write = Flipped(new FgAccumulatorBankWriteReq(config))
    val acc_config = Flipped(new FgAccumulatorBankConfigIO(config))
  })
  val mem = SyncReadMem(FG_DIM, ROW_TYPE)

  val in_rshift    = io.acc_config.in_rshift
  val acc_rshift   = io.acc_config.acc_rshift
  val relu6_lshift = io.acc_config.relu6_lshift
  val activation   = io.acc_config.act

  //-------------------------------------
  // unpack write signals
  //-------------------------------------
  val wr_en           = io.write.en
  val wr_row          = io.write.row
  val wr_cols         = io.write.cols
  val wr_fg_col_start = io.write.fg_col_start
  val wr_accum        = io.write.accum
  val wr_elemshift    = (wr_fg_col_start * FG_DIM.U)
  val wr_bitshift     = wr_elemshift * OTYPE_BITS.U
  val wr_data         = (io.write.data << wr_bitshift).asTypeOf(ROW_TYPE)
  val wr_mask         = (((1.U << wr_cols) - 1.U) << wr_elemshift)
                        .asTypeOf(Vec(FG_DIM, Bool()))

  //-------------------------------------
  // unpack read signals
  //-------------------------------------
  val rd_en           = io.read.req.en
  val rd_row          = io.read.req.row
  val rd_fg_col_start = io.read.req.fg_col_start

  //-------------------------------------
  // read from bank (for read or write req)
  //-------------------------------------
  val bank_rd_en  = wr_en || rd_en
  val bank_rd_row = Mux(wr_en, wr_row, rd_row)
  val bank_rdata  = mem.read(bank_rd_row, bank_rd_en)
  assert(!(wr_en && rd_en), "wr_en and rd_en in same cycle")

  //-------------------------------------
  // write the (possibly accumulated) data 1 cycle later
  //-------------------------------------
  val wr_en_buf    = ShiftRegister(wr_en,    1)
  val wr_row_buf   = ShiftRegister(wr_row,   1)
  val wr_data_buf  = ShiftRegister(wr_data,  1)
  val wr_mask_buf  = ShiftRegister(wr_mask,  1)
  val wr_accum_buf = ShiftRegister(wr_accum, 1)
  val wr_sum = VecInit((bank_rdata zip wr_data_buf).map { case (r,w) => r+w })
  when (wr_en_buf) {
    mem.write(wr_row_buf, Mux(wr_accum_buf, wr_sum, wr_data_buf), wr_mask_buf)
  }

  //-------------------------------------
  // read/output the activated data 2 cycles later
  //-------------------------------------
  val rd_en_buf           = ShiftRegister(rd_en,           1)
  val rd_row_buf          = ShiftRegister(rd_row,          1)
  val rd_fg_col_start_buf = ShiftRegister(rd_fg_col_start, 1)

  val rd_elemshift    = rd_fg_col_start_buf * FG_DIM.U
  val rd_bitshift     = rd_elemshift * ITYPE_BITS.U
  val rd_shifted_data = (bank_rdata.asUInt()>>rd_bitshift).asTypeOf(ROW_TYPE)

  val activated_rdata = WireInit(VecInit(rd_shifted_data.map(e => {
    val e_clipped = (e >> acc_rshift).clippedToWidthOf(inputType)
    val e_act = MuxCase(e_clipped, Seq(
      (activation === Activation.RELU) -> e_clipped.relu,
      (activation === Activation.RELU6) -> e_clipped.relu6(relu6_lshift)
    ))
    e_act
  })))
  io.read.resp.data := RegNext(activated_rdata.asUInt())
}
