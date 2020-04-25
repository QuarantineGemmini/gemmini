package gemmini

import chisel3._
import chisel3.util._
import gemmini.Util.UDValid

//===========================================================================
// tracked outstanding entry
// - data_start_idx and txn_start_idx are relative to total tilelink bytes
//   sent/recieved NOT useful bytes
//===========================================================================
class FgDMATrackerEntry[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  // same for all txns in a req
  val lrange           = new FgLocalRange(config)
  val rob_id           = UInt(LOG2_ROB_ENTRIES.W)
  val req_useful_bytes = UInt(LOG2_MAX_TRANSFER_BYTES.W)
  val data_start_idx   = UInt(LOG2_MAX_DMA_BYTES.W)
  // different for all txns in a req
  val txn_useful_bytes = UInt(LOG2_MAX_DMA_BYTES.W)
  val txn_bytes        = UInt(LOG2_MAX_DMA_BYTES.W)
  val txn_log2_bytes   = UInt(log2Up(LOG2_MAX_DMA_BYTES).W)
  val txn_start_idx    = UInt(LOG2_MAX_DMA_BYTES.W)
  val paddr            = UInt(coreMaxAddrBits.W)
}

//===========================================================================
// Tracker Interfaces
//===========================================================================
class FgDMATrackerPeekIO[T <: Data]
  (config: GemminiArrayConfig[T], max_bytes: Int)(implicit p: Parameters) 
  extends CoreBundle { 
  import config._
  val xactid = Output(UInt(LOG2_MAX_DMA_REQS.W))
  val entry  = Input(new FgDMATrackerEntry(config))
}

//===========================================================================
// DMATracker 
// - track outstanding DMA requests with their offset/length
// - all outstanding DMA reqs are merged to/split from a single acc/sp row!
//===========================================================================
class FgDMATracker[T <: Data]
  (config: GemminiArrayConfig[T], max_bytes: Int, peeks: Int)
  (implicit p: Parameters) extends Module {
  import config._

  val io = IO(new Bundle {
    val nextid = Output(UInt(LOG2_MAX_DMA_REQS.W))
    val alloc = Flipped(Decoupled(new FgDMATrackerEntry(config)))
    val peek = Vec(peeks, Flipped(new FgDMATrackerPeekIO(config, max_bytes)))
    val pop = Input(Valid(UInt(LOG2_MAX_DMA_REQS.W)))
    val busy = Output(Bool())
  })

  // outstanding transaction registers
  val entries = Reg(Vec(MAX_DMA_REQS,UDValid(new FgDMATrackerEntry(config))))
  when (reset.toBool()) {
    entries.foreach(_.valid := false.B)
  }
  io.busy := entries.map(_.valid).reduce(_ || _)

  // interface to allocator
  val free_entry = MuxCase((MAX_DMA_REQS-1).U, entries.zipWithIndex.map { 
    case (e, i) => !e.valid -> i.U 
  })
  io.nextid := free_entry
  io.alloc.ready := !entries.map(_.valid).reduce(_ && _)
  when (io.alloc.fire()) {
    entries(free_entry).valid := true.B
    entries(free_entry).bits := io.alloc.bits
  }

  // interface to peekers
  for (i <- 0 to peeks) {
    io.peek(i).bits.entry := entries(io.peek(i).bits.xactid).bits
  }

  // interface to popper
  when (io.pop.fire()) {
    assert(entries(io.pop.bits).valid, "popping a non-valid DMA transaction")
    entries(io.pop.bits).valid := false.B
  }
}
