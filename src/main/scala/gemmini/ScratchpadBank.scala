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
class ScratchpadBankReadReq[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en  = Output(Bool())
  val row = Output(UInt(SP_BANK_ROWS_IDX.W))
}

class ScratchpadBankReadResp[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val data = Output(UInt(SP_ROW_BITS.W))
}

class ScratchpadReadIO[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val req = new ScratchpadReadReq(config)
  val resp = Flipped(new ScratchpadReadResp(config))
}

class ScratchpadWriteIO[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val en   = Output(Bool())
  val row  = Output(UInt(SP_BANK_ROWS_IDX.W))
  val mask = Output(UInt(DIM.W))
  val data = Output(UInt(SP_ROW_BITS.W))
}

//===========================================================================
// single scratchpad bank
//===========================================================================
class ScratchpadBank[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends Module {
  import config._
  //-------------------------------------
  // I/O interface
  //-------------------------------------
  val io = IO(new Bundle {
    val read = Flipped(new ScratchpadBankReadIO(config))
    val write = Flipped(new ScratchpadBankWriteReq(config))
  })
  val mem = SyncReadMem(SP_BANK_ROWS, Vec(DIM, inputType))

  //--------------------------------------
  // write path
  //--------------------------------------
  val wr_en   = io.write.en
  val wr_row  = io.write.row
  val wr_data = io.write.data.asTypeOf(Vec(DIM, inputType))
  val wr_mask = io.write.mask.asTypeOf(Vec(DIM, Bool()))
  when (wr_en) {
    mem.write(wr_row, wr_data, wr_mask)
  }

  //--------------------------------------
  // read path (1-cycle latency)
  //--------------------------------------
  val rd_en  = io.read.req.en
  val rd_row = io.read.req.row
  io.read.resp.data := mem.read(rd_row, rd_en).asUInt()
}

