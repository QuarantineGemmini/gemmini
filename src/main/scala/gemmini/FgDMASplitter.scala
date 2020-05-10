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
    val req = Flipped(Decoupled(new FgDMAStoreRequest(config,max_xfer_bytes)))
    val txn = Flipped(Decoupled(new FgDMADispatch(config)))
    val peek = new FgDMATrackerPeekIO(config, max_xfer_bytes)
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
  
  // current txn that we are splitting into beats
  val txn = Queue(io.txn, 4)
  txn.ready := false.B
  val cur_txn = Reg(new FgDMADispatch(config))
  io.peek.xactid := cur_txn.xactid

  val txn_log2_bytes = cur_txn.txn_log2_bytes
  val paddr          = cur_txn.paddr       
 
  //--------------------------------------
  val beat_idx = RegInit(0.U(DMA_TXN_BEATS_CTR.W))
  val bytes_sent_next = (beat_idx + 1.U) * DMA_BUS_BYTES.U

  val is_last_txn      = io.peek.entry.is_last_txn
  val req_useful_bytes = io.peek.entry.req_useful_bytes
  val txn_useful_bytes = io.peek.entry.txn_useful_bytes
  val txn_bytes        = io.peek.entry.txn_bytes
  val txn_start_idx    = io.peek.entry.txn_start_idx
  val data_start_idx   = io.peek.entry.data_start_idx
  val data_end_idx     = data_start_idx + req_useful_bytes - 1.U
  val beat_start_idx   = txn_start_idx + (beat_idx * DMA_BUS_BYTES.U)
  val beat_end_idx     = beat_start_idx + (DMA_BUS_BYTES - 1).U

  val req_data_lshift    = Wire(UInt(DMA_TXN_BITS_IDX.W))
  req_data_lshift       := data_start_idx * 8.U
  val shifted_req_data   = (cur_req.data << req_data_lshift)

  val beat_data_rshift   = Wire(UInt(log2Ceil(max_xfer_bits).W))
  beat_data_rshift      := (txn_start_idx * 8.U) + (beat_idx * DMA_BUS_BITS.U)
  val shifted_beat_data  = (shifted_req_data >> beat_data_rshift)

  val mask_start_offset  = Wire(UInt(DMA_BUS_BYTES_IDX.W))
  val data_starts_before = (data_start_idx < beat_start_idx)
  val data_starts_after  = (data_start_idx > beat_end_idx)
  mask_start_offset     := Mux(data_starts_before, 0.U, 
                            Mux(data_starts_after, DMA_BUS_BYTES.U, 
                                (data_start_idx - beat_start_idx)))

  val mask_end_offset  = Wire(UInt(DMA_BUS_BYTES_IDX.W))
  val data_ends_before = (data_end_idx < beat_start_idx)
  val data_ends_after  = (data_end_idx > beat_end_idx)
  mask_end_offset     := Mux(data_ends_before, DMA_BUS_BYTES.U,
                          Mux(data_ends_after, 0.U,
                              (beat_end_idx - data_end_idx)))

  val unshifted_mask = WireInit(~0.U(DMA_BUS_BYTES.W))
  val shifted_mask = ((((unshifted_mask >> mask_start_offset)
                                        << mask_start_offset)
                                        << mask_end_offset)
                                        >> mask_end_offset)

  //---------------------------------
  // outputs
  //---------------------------------
  io.tl_a.valid           := false.B
  io.tl_a.bits.is_full    := (txn_bytes === txn_useful_bytes)
  io.tl_a.bits.xactid     := cur_txn.xactid
  io.tl_a.bits.paddr      := paddr
  io.tl_a.bits.log2_bytes := txn_log2_bytes
  io.tl_a.bits.data       := shifted_beat_data
  io.tl_a.bits.mask       := shifted_mask

  //---------------------------------
  // FSM
  //---------------------------------
  val (s_PENDING_REQ :: s_PENDING_TXN :: s_SEND_BEAT :: Nil) = Enum(3)
  val state = RegInit(s_PENDING_REQ)

  def start_next_txn(dummy: Int = 0) = {
    state := s_PENDING_TXN
    beat_idx := 0.U
    txn.ready := true.B
    when (txn.fire()) {
      cur_txn := txn.bits
      state := s_SEND_BEAT
    }
  }

  def start_next_req(dummy: Int = 0) = {
    state := s_PENDING_REQ
    req.ready := true.B
    when (req.fire()) {
      cur_req := req.bits
      start_next_txn()
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
        beat_idx := beat_idx + 1.U
        when (bytes_sent_next === txn_bytes) {
          when (is_last_txn) {
            start_next_req()
          } .otherwise {
            start_next_txn()
          }
        }
      }
    }
  }
}
