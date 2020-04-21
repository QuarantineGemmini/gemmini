package gemmini

import chisel3._
import chisel3.util._


class AccumulatorReadReq(val n: Int, val shift_width: Int) extends Bundle {
  val addr = UInt(log2Ceil(n).W)
  val shift = UInt(shift_width.W)
  val relu6_shift = UInt(shift_width.W)
  val act = UInt(2.W)

  val fromDMA = Bool()
}

class AccumulatorReadResp[T <: Data: Arithmetic](rdataType: Vec[Vec[T]]) extends Bundle {
  val data = rdataType.cloneType
  val fromDMA = Bool()

  override def cloneType: this.type = new AccumulatorReadResp(rdataType.cloneType).asInstanceOf[this.type]
}

class AccumulatorReadIO[T <: Data: Arithmetic]
  
  (n: Int, shift_width: Int, rdataType: Vec[Vec[T]]) extends Bundle {
  val req = Decoupled(new AccumulatorReadReq(n, shift_width))
  val resp = Flipped(Decoupled(new AccumulatorReadResp(rdataType.cloneType)))

  override def cloneType: this.type = new AccumulatorReadIO(n, shift_width, rdataType.cloneType).asInstanceOf[this.type]
}

class AccumulatorWriteIO[T <: Data: Arithmetic](n: Int, t: Vec[Vec[T]]) extends Bundle {
  val en = Output(Bool())
  val addr = Output(UInt(log2Ceil(n).W))
  val data = Output(t)
  val acc = Output(Bool())
  val mask = Output(Vec(t.getWidth / 8, Bool())) // TODO Use aligned_to here

  override def cloneType: this.type = new AccumulatorWriteIO(n, t).asInstanceOf[this.type]
}

class AccumulatorMemIO[T <: Data: Arithmetic](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends Bundle {
  import config._
  val read = Flipped(
    new AccumulatorReadIO(n, log2Ceil(t.head.head.getWidth), rdata))
  val write = Flipped(new AccumulatorWriteIO(n, t))
  override def cloneType: this.type 
    = new AccumulatorMemIO(n, t, rdata).asInstanceOf[this.type]
}

//==========================================================================
// Accumulator Bank
//==========================================================================
class AccumulatorMem[T <: Data: Arithmetic](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters, ev: Arithmetic[T]) extends Module {
  import config._
  import ev._

  val ACC_TYPE = Vec(meshColumns, Vec(tileColumns, accType))
  val SP_TYPE  = Vec(meshColumns, Vec(tileColumns, inputType))

  val io = IO(new AccumulatorMemIO(ACC_BANK_ROWS, ACC_TYPE, SP_TYPE))

  val mem = TwoPortSyncMem(ACC_BANK_ROWS, ACC_TYPE, aligned_to) 

  val rd_req_fire  = io.read.req.fire()
  val wr_req_fire  = io.write.en && !io.write.acc
  val acc_req_fire = io.write.en &&  io.write.acc

  //==========================================================================
  // 2-cycle write/accumulate pipeline (need to read first)
  //==========================================================================
  val wdata_buf   = ShiftRegister(io.write.data, 2)
  val waddr_buf   = ShiftRegister(io.write.addr, 2)
  val acc_buf     = ShiftRegister(io.write.acc, 2)
  val mask_buf    = ShiftRegister(io.write.mask, 2)
  val w_buf_valid = ShiftRegister(io.write.en, 2)

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
    val fromDMA     = io.read.req.bits.fromDMA.cloneType
  }
  
  val q = Module(new Queue(new PipelinedRdataAndActT, 1, true, true))
  q.io.enq.bits.data        := mem.io.rdata
  q.io.enq.bits.shift       := RegNext(io.read.req.bits.shift)
  q.io.enq.bits.relu6_shift := RegNext(io.read.req.bits.relu6_shift)
  q.io.enq.bits.act         := RegNext(io.read.req.bits.act)
  q.io.enq.bits.fromDMA     := RegNext(io.read.req.bits.fromDMA)
  q.io.enq.valid            := RegNext(io.read.req.fire())

  val p = Pipeline(q.io.deq, mem_pipeline, 
      Seq.fill(mem_pipeline)((x: PipelinedRdataAndActT) => x) :+ {
    x: PipelinedRdataAndActT =>
      val activated_rdata = VecInit(x.data.map(v => VecInit(v.map { e =>
        val e_clipped = (e >> x.shift).clippedToWidthOf(SP_TYPE.head.head)
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
  io.read.resp.bits.fromDMA := p.bits.fromDMA
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
