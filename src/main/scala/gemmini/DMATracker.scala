package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import gemmini.Util._

//===========================================================================
// tracked outstanding entry
// - data_start_idx and txn_start_idx are relative to total tilelink bytes
//   sent/recieved NOT useful bytes
//===========================================================================
class DMATrackerEntry[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  // same for all txns in a req
  val reqid            = UInt(DMA_REQS_IDX.W)
  val lrange           = new LocalRange(config)
  val req_useful_bytes = UInt(ACC_ROW_BYTES_CTR.W)
  val data_start_idx   = UInt(DMA_TXN_BYTES_IDX.W)
  // different for all txns in a req
  val txn_start_idx    = UInt(ACC_ROW_BYTES_CTR.W)
  val txn_useful_bytes = UInt(DMA_TXN_BYTES_CTR.W)
  val txn_bytes        = UInt(DMA_TXN_BYTES_CTR.W)
  val is_last_txn      = Bool()
}

//===========================================================================
// Tracker Interfaces
//===========================================================================
class DMATrackerPeekIO[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val xactid = Output(UInt(DMA_REQS_IDX.W))
  val entry  = Input(new DMATrackerEntry(config))
}

//===========================================================================
// DMATracker
// - track outstanding DMA requests with their offset/length
// - all outstanding DMA reqs are merged to/split from a single acc/sp row!
//===========================================================================
class DMATracker[T <: Data](config: GemminiArrayConfig[T], peeks: Int)
  (implicit p: Parameters) extends Module {
  import config._

  val io = IO(new Bundle {
    val nextid = Output(UInt(DMA_REQS_IDX.W))
    val alloc  = Flipped(Decoupled(new DMATrackerEntry(config)))
    val peek   = Vec(peeks, Flipped(new DMATrackerPeekIO(config)))
    val pop    = Input(Valid(UInt(DMA_REQS_IDX.W)))
  })

  // outstanding transaction registers
  val entries = Reg(Vec(DMA_REQS, UDValid(new DMATrackerEntry(config))))
  when (reset.toBool()) {
    entries.foreach(_.valid := false.B)
  }

  // interface to allocator
  val free_entry = MuxCase((DMA_REQS-1).U, entries.zipWithIndex.map {
    case (e, i) => !e.valid -> i.U
  })
  io.nextid := free_entry
  io.alloc.ready := !entries.map(_.valid).reduce(_ && _)
  when (io.alloc.fire()) {
    entries(free_entry).valid := true.B
    entries(free_entry).bits := io.alloc.bits
  }

  // interface to peekers
  for (i <- 0 until peeks) {
    io.peek(i).entry := entries(io.peek(i).xactid).bits
  }

  // interface to popper
  when (io.pop.fire()) {
    assert(entries(io.pop.bits).valid, "popping a non-valid DMA transaction")
    entries(io.pop.bits).valid := false.B
  }
}
