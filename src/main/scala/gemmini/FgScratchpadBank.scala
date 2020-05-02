package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import Util._

//===========================================================================
// internal scratchpad bank interfaces (all non-decoupled)
//===========================================================================
class FgScratchpadBankReadReq[T <: Data]
  (config: FgGemminiArrayConfig[T], fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en           = Output(Bool())
  val row          = Output(UInt(FG_DIM_IDX.W))
  val fg_col_start = Output(UInt(log2Up(fg_cols).W))
}

class FgScratchpadBankReadResp[T <: Data]
  (config: FgGemminiArrayConfig[T], port_fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val PORT_BITS = port_fg_cols * FG_DIM * ITYPE_BITS
  val data = Output(UInt(PORT_BITS.W))
}

class FgScratchpadBankReadIO[T <: Data]
  (config: FgGemminiArrayConfig[T], fg_cols: Int, port_fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  val req  = new FgScratchpadBankReadReq(config, fg_cols)
  val resp = Flipped(new FgScratchpadBankReadResp(config, port_fg_cols))
}

class FgScratchpadBankWriteReq[T <: Data]
  (config: FgGemminiArrayConfig[T], fg_cols: Int, port_fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val PORT_ELEMS = port_fg_cols * FG_DIM
  val PORT_BITS  = PORT_ELEMS * ITYPE_BITS
  val en           = Output(Bool())
  val row          = Output(UInt(FG_DIM_IDX.W))
  val cols         = Output(UInt(log2Up(PORT_ELEMS + 1).W))
  val fg_col_start = Output(UInt(log2Up(fg_cols).W))
  val data         = Output(UInt(PORT_BITS.W))
}

//============================================================================
// scratchpad bank
// - 2-cycle latency hardcoded!
// - fg_col_start shifts for reads/writes done WITHIN THE BANK!
//============================================================================
class FgScratchpadBank[T <: Data]
  (config: FgGemminiArrayConfig[T], fg_cols: Int, port_fg_cols: Int)
  (implicit p: Parameters) extends CoreModule {
  import config._
  val PORT_BITS = port_fg_cols * FG_DIM * ITYPE_BITS
  val ROW_ELEMS  = fg_cols * FG_DIM
  //-------------------------------------
  // I/O interface
  //-------------------------------------
  val io = IO(new Bundle {
    val read = Flipped(
                new FgScratchpadBankReadIO(config, fg_cols, port_fg_cols))
    val write = Flipped(
                new FgScratchpadBankWriteReq(config, fg_cols, port_fg_cols))
  })
  val mem = SyncReadMem(FG_DIM, Vec(ROW_ELEMS, inputType))

  //--------------------------------------
  // write path
  //--------------------------------------
  val wr_en           = io.write.en
  val wr_row          = io.write.row
  val wr_cols         = io.write.cols
  val wr_fg_col_start = io.write.fg_col_start
  val wr_bitshift     = (wr_fg_col_start * FG_DIM.U * ITYPE_BITS.U)
  val wr_elemshift    = (wr_fg_col_start * FG_DIM.U)
  val wr_data         = (io.write.data << wr_bitshift).asTypeOf(
                          Vec(ROW_ELEMS, inputType))
  val wr_mask         = (((1.U << wr_cols) - 1.U) << wr_elemshift).asTypeOf(
                          Vec(FG_DIM, Bool()))
  when (wr_en) {
    mem.write(wr_row, wr_data, wr_mask)
  }

  //--------------------------------------
  // read path
  //--------------------------------------
  val rd_row          = RegNext(io.read.req.row)
  val rd_fg_col_start = RegNext(io.read.req.fg_col_start)
  val rd_en           = io.read.req.en
  val rd_data         = mem.read(rd_row, rd_en).asUInt()
  val rd_bitshift     = (rd_fg_col_start * FG_DIM.U * ITYPE_BITS.U)
  val data_shifted    = (rd_data >> rd_bitshift)
  io.read.resp.data  := RegNext(data_shifted(PORT_BITS-1,0))
}

