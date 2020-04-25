package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import Util._

//===========================================================================
// internal scratchpad bank interfaces 
//===========================================================================
class FgScratchpadBankReadReq[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val row          = UInt(LOG2_FG_DIM.W)
  val cols         = UInt(LOG2_SP_ROW_ELEMS.W)
  val sq_col_start = UInt(LOG2_FG_NUM.W)
}

class FgScratchpadBankReadResp[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val data = UInt(SP_ROW_BITS.W)
}

class FgScratchpadBankReadIO[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val req = FgScratchpadBankReadReq(config)
  val resp = Flipped(FgScratchpadBankReadResp(config))
}

class FgScratchpadBankWriteReq[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val row          = Output(UInt(LOG2_FG_DIM.W))
  val cols         = Output(UInt(LOG2_SP_ROW_ELEMS.W))
  val sq_col_start = Output(UInt(LOG2_FG_NUM.W))
  val data         = Output(UInt(ACC_ROW_BITS.W))
}

//============================================================================
// scratchpad bank
//============================================================================
class FgScratchpadBank[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  //-------------------------------------
  // I/O interface
  //-------------------------------------
  val io = IO(new Bundle {
    val read = Flipped(new FgScratchpadBankReadIO(config))
    val write = Flipped(new FgScratchpadBankWriteReq(config))
  })

  val mem = SyncReadMem(n, Vec(mask_len, mask_elem))

  when (io.write.en) {
    mem.write(io.write.addr, 
      io.write.data.asTypeOf(Vec(mask_len, mask_elem)), io.write.mask)
  }

  val raddr = io.read.req.bits.row
  val ren   = io.read.req.fire()
  val rdata = mem.read(row, ren).asUInt()

  // Make a queue which buffers the result of an SRAM read if it 
  // can't immediately be consumed
  val q = Module(new Queue(new ScratchpadReadResp(w), 1, true, true))
  q.io.enq.valid := RegNext(ren)
  q.io.enq.bits.data := rdata

  val q_will_be_empty = 
    (q.io.count +& q.io.enq.fire()) - q.io.deq.fire() === 0.U
  io.read.req.ready := q_will_be_empty

  // Build the rest of the resp pipeline
  val rdata_p = Pipeline(q.io.deq, mem_pipeline)
  io.read.resp <> rdata_p
}

