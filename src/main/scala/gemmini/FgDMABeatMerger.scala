package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import Util._

//============================================================================
// FgDMABeatMerger
// - merge multiple beats in to a txn buffer and sends txn to MemUnit
// - multiple txns may make up a Load Request, but we don't merge txns here
//============================================================================
class FgDMABeatMerger[T <: Data]
  (config: FgGemminiArrayConfig[T], max_xfer_bytes:Int)
  (implicit p: Parameters) extends Module {
  import config._
  val max_xfer_bits = max_xfer_bytes*8
  //---------------------------------
  // I/O interface
  //---------------------------------
  val io = IO(new Bundle {
    val beat = Flipped(Decoupled(new Bundle {
      val xactid       = UInt(DMA_REQS_IDX.W)
      val data         = UInt(DMA_BUS_BITS.W)
      val beat_idx     = UInt(DMA_TXN_BEATS_IDX.W)
      val is_last_beat = Bool()
    }))
    val peek  = new FgDMATrackerPeekIO(config, max_xfer_bytes)
    val pop   = Output(Valid(UInt(DMA_REQS_IDX.W)))
    val decr  = Decoupled(UInt(DMA_REQS_IDX.W))
    val chunk = Decoupled(new FgDMALoadDataChunk(config, max_xfer_bytes))
  })
  //---------------------------------
  // internal state
  //---------------------------------
  val data = RegInit(0.U(DMA_TXN_BITS.W))
  val data_next = Wire(UInt(DMA_TXN_BITS.W))

  //---------------------------------
  // if (data_start_idx >= txn_start_idx)
  //   shift data right by (data_start_idx - txn_start_idx)
  //   set fg_col_start = 0
  // else 
  //   leave data starting at index 0
  //   set fg_col_start = (txn_start_idx - data_start_idx) / FG_DIM
  //---------------------------------
  val lrange           = io.peek.entry.lrange
  val elem_bytes       = Mux(lrange.is_acc, OTYPE_BYTES.U, ITYPE_BYTES.U)
  val txn_useful_bytes = io.peek.entry.txn_useful_bytes
  val txn_useful_elems = txn_useful_bytes / elem_bytes
  val data_start_idx   = io.peek.entry.data_start_idx
  val txn_start_idx    = io.peek.entry.txn_start_idx
  val beat_data        = io.beat.bits.data
  val beat_idx         = io.beat.bits.beat_idx
  val is_last_beat     = io.beat.bits.is_last_beat

  val is_first_txn          = data_start_idx >= txn_start_idx
  val first_rshift_bits     = (data_start_idx - txn_start_idx) * 8.U
  val bytes_per_fg          = FG_DIM.U * elem_bytes
  val nonfirst_fg_col_start = (txn_start_idx - data_start_idx) / bytes_per_fg
  // FIXME: major bug. data_start_idx must be aligned to bytes_per_fg!!!
  //        to fix this, we need to change all fg_col_start to just col_start
  //        which will make muxes larger everywhere.
  assert(
 
  val fg_col_start     = Mux(is_first_txn, 0.U, nonfirst_fg_col_start)
  val txn_rshift_bits  = Wire(UInt(log2Ceil(max_xfer_bits+1).W))
  val beat_lshift_bits = Wire(UInt(log2Ceil(max_xfer_bits+1).W))
  txn_rshift_bits     := Mux(is_first_txn, first_rshift_bits, 0.U)
  beat_lshift_bits    := beat_idx * DMA_BUS_BITS.U

  val shifted_data = beat_data << beat_lshift_bits
  data_next := data | Mux(is_first_txn, shifted_data >> txn_rshift_bits, 
                           shifted_data)

  //----------------------------------------
  // assign I/O/next state
  //----------------------------------------
  io.beat.ready  := io.decr.ready && io.chunk.ready
  io.peek.xactid := io.beat.bits.xactid
  io.pop.valid   := io.beat.fire() && is_last_beat
  io.pop.bits    := io.beat.bits.xactid
  io.decr.valid  := io.beat.fire() && is_last_beat
  io.decr.bits   := io.peek.entry.reqid
  io.chunk.valid                    := io.beat.fire() && is_last_beat
  io.chunk.bits.lrange              := lrange
  io.chunk.bits.lrange.cols         := txn_useful_elems
  io.chunk.bits.lrange.fg_col_start := fg_col_start
  io.chunk.bits.data                := data_next

  data := Mux(io.beat.fire(), Mux(is_last_beat, 0.U, data_next), data)
}
