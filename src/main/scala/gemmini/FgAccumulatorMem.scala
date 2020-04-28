package gemmini

import chisel3._
import chisel3.util._

class FgAccumulatorBankReadReq[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val row          = UInt(LOG2_FG_DIM.W)
  val cols         = UInt(LOG2_ACC_ROW_ELEMS.W)
  val sq_col_start = UInt(LOG2_FG_NUM.W)
}

class FgAccumulatorBankReadResp[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val data = UInt(ACC_ROW_BITS.W)
}

// NOTE: the read req blocks if accum is issued same cycle, since the read
//       comes from DMA writer, which can wait
class FgAccumulatorBankReadIO[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val req = Decoupled(new FgAccumulatorBankReadReq(config))
  val resp = Flipped(new FgAccumulatorBankReadResp(config))
}

class FgAccumulatorBankWriteReq[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en           = Output(Bool())
  val row          = Output(UInt(LOG2_FG_DIM.W))
  val cols         = Output(UInt(LOG2_ACC_ROW_ELEMS.W))
  val sq_col_start = Output(UInt(LOG2_FG_NUM.W))
  val data         = Output(UInt(ACC_ROW_BITS.W))
  val accum        = Output(Bool())
}

//==========================================================================
// Accumulator Bank
//==========================================================================
class FgAccumulatorMem[T <: Data: Arithmetic](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters, ev: Arithmetic[T]) extends Module {
  import config._
  import ev._

  val io = IO(new Bundle {
    val read = Flipped(new FgAccumulatorBankReadIO(config))
    val write = Flipped(new FgAccumulatorBankWriteReq(config))
  })

  val mem = SyncReadMem(FG_DIM, Vec(ACC_ROW_ELEMS, accType))

  val wr_row          = io.write.row
  val wr_cols         = io.write.cols
  val wr_data         = io.write.data
  val wr_sq_col_start = io.write.sq_col_start
  val wr_data = io.write.data.asTypeOf(Vec(ACC_ROW_ELEMS, accType))
  val wr_mask = ((1.U << wr_cols) - 1.U) << (wr_sq_col_start * SQ_COL_ELEMS)
  when (
  when (io.write.en) {
    mem.write(wr_row, wr_data, wr_mask
  }

  val raddr = io.read.req.bits.row
  val ren   = io.read.req.fire()

  val rd_req_fire  = io.read.req.fire()
  val wr_req_fire  = io.write.en && !io.write.accum
  val acc_req_fire = io.write.en &&  io.write.accum

  val raddr = Mux(acc_req_fire, io.write
  io.read.req.bits.row
  val ren   = true.B
  val rdata = mem.read(row, ren).asUInt()

  //==========================================================================
  // 2-cycle write/accumulate pipeline (need to read first)
  //==========================================================================
  val wdata_buf   = ShiftRegister(io.write.data,  2)
  val waddr_buf   = ShiftRegister(io.write.addr,  2)
  val acc_buf     = ShiftRegister(io.write.accum, 2)
  val mask_buf    = ShiftRegister(io.write.mask,  2)
  val w_buf_valid = ShiftRegister(io.write.en,    2)

  val w_sum = VecInit((RegNext(mem.io.rdata) zip wdata_buf).map { 
    case (rv, wv) => VecInit((rv zip wv).map(t => t._1 + t._2))
  })

  mem.io.waddr := waddr_buf
  mem.io.wen   := w_buf_valid
  mem.io.wdata := Mux(acc_buf, w_sum, wdata_buf)
  mem.io.mask  := mask_buf

  //==========================================================================
  // (internal) 2-cycle SRAM read pipeline
  //==========================================================================
  mem.io.ren   := rd_req_fire || acc_req_fire
  mem.io.raddr := Mux(acc_req_fire, io.write.addr, io.read.req.bits.addr)

  class PipelinedRdataAndActT extends Bundle {
    val data        = mem.io.rdata.cloneType
    val shift       = io.read.req.bits.shift.cloneType
    val relu6_shift = io.read.req.bits.relu6_shift.cloneType
    val act         = io.read.req.bits.act.cloneType
  }
  
  val q = Module(new Queue(new PipelinedRdataAndActT, 1, true, true))
  q.io.enq.bits.data        := mem.io.rdata
  q.io.enq.bits.shift       := RegNext(io.read.req.bits.shift)
  q.io.enq.bits.relu6_shift := RegNext(io.read.req.bits.relu6_shift)
  q.io.enq.bits.act         := RegNext(io.read.req.bits.act)
  q.io.enq.valid            := RegNext(io.read.req.fire())

  val p = Pipeline(q.io.deq, mem_pipeline, 
      Seq.fill(mem_pipeline)((x: PipelinedRdataAndActT) => x) :+ {
    x: PipelinedRdataAndActT =>
      val activated_rdata = VecInit(x.data.map(v => VecInit(v.map { e =>
        val e_clipped = (e >> x.shift).clippedToWidthOf(inputType)
        val e_act = MuxCase(e_clipped, Seq(
          (x.act === Activation.RELU) -> e_clipped.relu,
          (x.act === Activation.RELU6) -> e_clipped.relu6(x.relu6_shift)))
        e_act
      })))
      val result = WireInit(x)
      result.data := activated_rdata
      result
  })

  //==========================================================================
  // read-request (2-cycles, stalls when write/accumulate in progress)
  //==========================================================================
  val q_will_be_empty 
    = (q.io.count +& q.io.enq.fire()) - q.io.deq.fire() === 0.U

  io.read.req.ready := 
    q_will_be_empty &&
    !acc_req_fire &&
    !(RegNext(io.write.en) && 
      RegNext(io.write.addr) === io.read.req.bits.addr) &&
    !(w_buf_valid && waddr_buf === io.read.req.bits.addr)

  io.read.resp.bits.data    := p.bits.data
  io.read.resp.valid        := p.valid
  p.ready                   := io.read.resp.ready

  //==========================================================================
  // sanity checks
  //==========================================================================
  val rd_wr_same_addr_1 = rd_req_fire && 
                          (wr_req_fire || acc_req_fire) && 
                          io.read.req.bits.addr === io.write.addr
  val rd_wr_same_addr_2 = rd_req_fire && 
                          w_buf_valid && 
                          waddr_buf === io.read.req.bits.addr

  assert(!rd_wr_same_addr_1,
         "reading from and writing to same address is not supported")
  assert(!rd_wr_same_addr_2,
         "reading from an address immediately after writing " +
         "to it is not supported")
}
