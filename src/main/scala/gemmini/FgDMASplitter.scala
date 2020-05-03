package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import Util._

//============================================================================
// FgDMASplitter
// - inputs are transactions belonging to the same store request
// - this splitter splits transactions into TL-A beats
// - achieves max throughput of 1 tl-A request/cycle
//============================================================================
class FgDMASplitter[T <: Data]
  (config: FgGemminiArrayConfig[T], max_xfer_bytes: Int)
  (implicit p: Parameters) extends CoreModule {
  import config._
  val max_xfer_bits = max_xfer_bytes*8
  //---------------------------------
  // I/O interface
  //---------------------------------
  val io = IO(new Bundle {
    // req is driven from FgDMA req port
    val req = Flipped(Decoupled(new FgDMAStoreRequest(config,max_xfer_bytes)))
    // txn is driven from the FgControl
    val txn = Flipped(Decoupled(new FgDMADispatch(config)))
    // combinationally view the FgXactTracker entry for txn's xactid
    val peek = new FgDMATrackerPeekIO(config, max_xfer_bytes)
    // output beats to tl_a interface
    val tl_a = Decoupled(new Bundle {
      val xactid     = UInt(DMA_REQS_IDX.W)
      val paddr      = UInt(coreMaxAddrBits.W)
      val log2_bytes = UInt(DMA_TXN_BYTES_CTR_IDX.W)
      val is_full    = Bool()
      val data       = Output(UInt((DMA_BUS_BYTES*8).W))
      val mask       = Output(UInt(DMA_BUS_BYTES.W))
    })
  })

  //---------------------------------
  // internal state managed by FSM
  //---------------------------------
  // pre-load the req, which comes in 1+ cycles before the 1st matching txn
  val req = Queue(io.req, 2)
  req.ready := false.B
  val cur_req = Reg(new FgDMAStoreRequest(config, max_xfer_bytes))
  val data = cur_req.data
  
  // current txn that we are splitting into beats
  val txn = Queue(io.txn, 4)
  txn.ready := false.B
  val cur_txn = Reg(new FgDMADispatch(config))
  io.peek.xactid := cur_txn.xactid

  val req_useful_bytes = io.peek.entry.req_useful_bytes
  val data_start_idx   = io.peek.entry.data_start_idx
  val data_end_idx     = data_start_idx + req_useful_bytes - 1.U
  val txn_useful_bytes = io.peek.entry.txn_useful_bytes
  val txn_bytes        = io.peek.entry.txn_bytes
  val txn_log2_bytes   = io.peek.entry.txn_log2_bytes
  val txn_start_idx    = io.peek.entry.txn_start_idx
  val paddr            = io.peek.entry.paddr       
 
  val useful_bytes_sent = RegInit(0.U(log2Ceil(max_xfer_bytes+1).W))
  val txn_bytes_sent    = RegInit(0.U(DMA_TXN_BYTES_CTR.W))
  val beat_idx          = RegInit(0.U(DMA_TXN_BEATS_IDX.W))

  //---------------------------------
  // output logic
  //---------------------------------
  // calculate beat offsets into data-buffer
  val beat_start_idx = txn_start_idx + (beat_idx * DMA_BUS_BYTES.U)
  val beat_end_idx   = beat_start_idx + DMA_BUS_BYTES.U - 1.U

  // firrtl requires < 20 bits to represent the shift-amount
  val lshift_bits = Wire(UInt(log2Ceil(max_xfer_bits+1).W))
  val rshift_bits = Wire(UInt(log2Ceil(max_xfer_bits+1).W))
  lshift_bits := (data_start_idx - beat_start_idx)*8.U
  rshift_bits := (beat_start_idx - data_start_idx)*8.U

  val beat_data_full = Mux(beat_start_idx > data_start_idx,
                           data >> rshift_bits,
                           data << lshift_bits)
  val beat_data = beat_data_full(DMA_BUS_BITS-1, 0)

  // beat write-mask calculations
  val is_full  = (beat_start_idx >= data_start_idx) &&
                 (beat_end_idx   <= data_end_idx)
  val is_empty = (beat_start_idx  > data_end_idx) ||
                 (beat_end_idx    < data_start_idx)
  val start_mask_offset = Mux(is_full || is_empty, 0.U, 
                              Mux(beat_start_idx >= data_start_idx, 0.U,
                                  data_start_idx - beat_start_idx))
  val end_mask_offset   = Mux(is_full || is_empty, 0.U, 
                              Mux(beat_end_idx <= data_end_idx, 0.U,
                                  beat_start_idx - data_start_idx))

  // firrtl requires < 20 bits to represent the shift-amount
  val p1_mask_bits = Wire(UInt(log2Ceil(max_xfer_bytes+1).W))
  val p2_mask_bits = Wire(UInt(log2Ceil(max_xfer_bytes+1).W))
  p1_mask_bits := DMA_BUS_BYTES.U - end_mask_offset
  p2_mask_bits := start_mask_offset
  val beat_mask = Mux(is_full || is_empty, 0.U,
                      (((1.U << p1_mask_bits) - 1.U) & 
                      ~((1.U << p2_mask_bits) - 1.U)))
 
  //---------------------------------
  // outputs
  //---------------------------------
  io.tl_a.valid           := false.B
  io.tl_a.bits.is_full    := is_full
  io.tl_a.bits.xactid     := cur_txn.xactid
  io.tl_a.bits.paddr      := paddr
  io.tl_a.bits.log2_bytes := txn_log2_bytes
  io.tl_a.bits.data       := beat_data
  io.tl_a.bits.mask       := beat_mask

  //---------------------------------
  // next-state logic
  //---------------------------------
  val txn_bytes_sent_next    = txn_bytes_sent + DMA_BUS_BYTES.U
  val useful_bytes_sent_next = useful_bytes_sent + txn_useful_bytes
  val txn_finished_next      = (txn_bytes_sent_next === txn_bytes)
  val xfer_finished_next     = (useful_bytes_sent_next === req_useful_bytes)

  //---------------------------------
  // FSM
  //---------------------------------
  val (s_PENDING_REQ :: s_PENDING_TXN :: s_SEND_BEAT :: Nil) = Enum(3)
  val state = RegInit(s_PENDING_REQ)

  def start_next_txn(dummy: Int = 0) = {
    txn_bytes_sent := 0.U
    beat_idx := 0.U
    txn.ready := true.B
    when (txn.fire()) {
      cur_txn := txn.bits
      state := s_SEND_BEAT
    } .otherwise {
      state := s_PENDING_TXN
    }
  }

  def start_next_req(dummy: Int = 0) = {
    useful_bytes_sent := 0.U
    req.ready := true.B
    when (req.fire()) {
      cur_req := req.bits
      start_next_txn()
    } .otherwise {
      state := s_PENDING_REQ
    }
  }

  switch (state) {
    is (s_PENDING_REQ) {
      start_next_req()
    }
    is (s_PENDING_TXN) {
      start_next_txn()
    }
    is (s_SEND_BEAT) {
      io.tl_a.valid := true.B
      when(io.tl_a.fire()) {
        txn_bytes_sent := txn_bytes_sent_next
        beat_idx := beat_idx + 1.U
        when (xfer_finished_next) {
          assert(txn_finished_next, "req finished but one of its txn did not")
          start_next_req()
        } .elsewhen (txn_finished_next) {
          useful_bytes_sent := useful_bytes_sent_next
          start_next_txn()
        }
      }
    }
  }
}
