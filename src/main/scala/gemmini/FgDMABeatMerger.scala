package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import Util._

//============================================================================
// FgDMABeatMerger
// - merges multiple beats from multiple tilelink txns into a single buffer 
//   for a DMA read (dram -> scratchpad)
// - achieves max throughput of 1 resp/cycle
//============================================================================
class FgDMABeatMerger[T <: Data]
  (config: GemminiArrayConfig[T], max_xfer_bytes:Int)
  (implicit p: Parameters) extends Module {
  import config._
  //---------------------------------
  // I/O interface
  //---------------------------------
  val io = IO(new Bundle {
    val peek = new FgDMATrackerPeekIO(config, max_xfer_bytes)
    val pop  = Output(Valid(UInt(DMA_REQS_IDX.W)))
    val beat = Flipped(Decoupled(new Bundle {
      val xactid       = UInt(DMA_REQS_IDX.W)
      val data         = UInt(DMA_BUS_BITS.W)
      val beat_idx     = UInt(DMA_TXN_BEATS_IDX.W)
      val is_last_beat = Bool()
    }))
    val resp = Decoupled(new FgDMALoadResponse(config, max_xfer_bytes))
    val busy = Output(Bool())
  })

  //---------------------------------
  // internal state
  //---------------------------------
  val data = RegInit(0.U((max_xfer_bytes*8).W))
  val useful_bytes_merged = RegInit(0.U(log2Ceil(max_xfer_bytes+1).W))

  //---------------------------------
  // output/next-state logic
  // - TODO: possibly pipeline this massive barrel shifter if its too slow
  //---------------------------------
  val lrange           = io.peek.entry.lrange
  val rob_id           = io.peek.entry.rob_id
  val req_useful_bytes = io.peek.entry.req_useful_bytes
  val data_start_idx   = io.peak.entry.data_start_idx
  val txn_useful_bytes = io.peek.entry.txn_useful_bytes
  val txn_bytes        = io.peek.entry.txn_bytes
  val txn_start_idx    = io.peek.entry.txn_start_idx

  val beat_start_idx = txn_start_idx + (io.beat.bits.beat_idx * DMA_BUS_BYTES)
  val beat_data = io.beat.bits.data
  val data_next = data | 
                  Mux(beat_start_idx > data_start_idx,
                    beat_data << ((beat_start_idx - data_start_idx)*8.U),
                    beat_data >> ((data_start_idx - beat_start_idx)*8.U))

  val is_last_beat_in_txn = io.beat.bits.is_last_beat
  val useful_bytes_merged_next = useful_bytes_merged + txn_useful_bytes

  //---------------------------------
  // output signals
  //---------------------------------
  io.peek.xactid := io.beat.bits.xactid

  io.pop.valid   := false.B
  io.pop.xactid  := io.beat.bits.xactid

  io.beat.ready  := false.B

  io.resp.valid  := false.B
  io.resp.data   := data_next
  io.resp.lrange := lrange
  io.resp.rob_id := rob_id

  io.busy := (useful_bytes_merged =/= 0.U)

  //---------------------------------
  // state update logic
  //---------------------------------
  when (io.resp.ready) {
    // we only continue if we are able to output a resp this cycle
    io.beat.ready := true.B
    when (io.beat.fire()) {
      data := data_next
      when (is_last_beat_in_txn) {
        io.pop.valid := true.B
        useful_bytes_merged := useful_bytes_merged_next
        when(useful_bytes_merged_next === req_useful_bytes) {
          io.resp.valid := true.B
          data := 0.U
          useful_bytes_merged := 0.U
        }
      }
    }
  }
}
