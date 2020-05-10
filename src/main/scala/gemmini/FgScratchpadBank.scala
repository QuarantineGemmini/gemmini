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
  (val config: FgGemminiArrayConfig[T], val fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en        = Output(Bool())
  val row       = Output(UInt(FG_DIM_IDX.W))
  val col_start = Output(UInt(log2Up(fg_cols * FG_DIM).W))
}

class FgScratchpadBankReadResp[T <: Data]
  (val config: FgGemminiArrayConfig[T], val port_fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val PORT_BITS = port_fg_cols * FG_DIM * ITYPE_BITS
  val data = Output(UInt(PORT_BITS.W))
}

class FgScratchpadBankReadIO[T <: Data]
  (val config:FgGemminiArrayConfig[T], val fg_cols:Int, val port_fg_cols:Int)
  (implicit p: Parameters) extends CoreBundle {
  val req  = new FgScratchpadBankReadReq(config, fg_cols)
  val resp = Flipped(new FgScratchpadBankReadResp(config, port_fg_cols))
}

class FgScratchpadBankWriteReq[T <: Data]
  (val config:FgGemminiArrayConfig[T], val fg_cols:Int, val port_fg_cols:Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val PORT_ELEMS = port_fg_cols * FG_DIM
  val PORT_BITS  = PORT_ELEMS * ITYPE_BITS
  val en        = Output(Bool())
  val row       = Output(UInt(FG_DIM_IDX.W))
  val cols      = Output(UInt(log2Up(PORT_ELEMS + 1).W))
  val col_start = Output(UInt(log2Up(PORT_ELEMS + 1).W))
  val data      = Output(UInt(PORT_BITS.W))
}

//============================================================================
// scratchpad bank
// - 2-cycle latency hardcoded!
// - col_start shifts for reads/writes done WITHIN THE BANK!
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
  val wr_en        = io.write.en
  val wr_row       = io.write.row
  val wr_cols      = io.write.cols
  val wr_col_start = io.write.col_start
  val wr_bitshift  = wr_col_start * ITYPE_BITS.U
  val wr_elemshift = wr_col_start
  val wr_data      = (io.write.data << wr_bitshift).asTypeOf(
                       Vec(ROW_ELEMS, inputType))
  val wr_mask      = (((1.U << wr_cols) - 1.U) << wr_elemshift)
                       .asTypeOf(Vec(fg_cols*FG_DIM, Bool()))
  when (wr_en) {
    mem.write(wr_row, wr_data, wr_mask)
  }

  //--------------------------------------
  // read path
  //--------------------------------------
  val rd_row           = io.read.req.row
  val rd_en            = io.read.req.en
  val rd_col_start_buf = RegNext(io.read.req.col_start)
  val rd_data          = mem.read(rd_row, rd_en).asUInt()
  val rd_bitshift      = (rd_col_start_buf * ITYPE_BITS.U)
  val data_shifted     = (rd_data >> rd_bitshift)
  io.read.resp.data := RegNext(data_shifted(PORT_BITS-1,0))
}

