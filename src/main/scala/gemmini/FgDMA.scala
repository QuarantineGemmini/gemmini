package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.tile._
import testchipip.TLHelper

import Util._

//===========================================================================
// DMA read interface (dram -> scratchpad)
// - max_xfer_bytes: max bytes read from dram in a multi-tilelink-txn request
//===========================================================================
class FgDMALoadRequest[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange(config)
  val status = new MStatus
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

class FgDMALoadResponse[T <: Data]
  (val config: FgGemminiArrayConfig[T], val max_xfer_bytes: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val data   = UInt((max_xfer_bytes*8).W)
  val lrange = new FgLocalRange(config)
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

class FgDMALoad[T <: Data](config: FgGemminiArrayConfig[T], 
  name: String = "dma-load", max_xfer_bytes: Int)
  (implicit p: Parameters) extends LazyModule {
  import config._

  val node = TLHelper.makeClientNode(name, IdRange(0, DMA_REQS))

  lazy val module = new LazyModuleImp(this)
    with HasCoreParameters with MemoryOpConstants {
    // I/O interface
    val (tl, edge) = node.out(0)

    val io = IO(new Bundle {
      val req   = Flipped(Decoupled(new FgDMALoadRequest(config)))
      val resp  = Decoupled(new FgDMALoadResponse(config, max_xfer_bytes))
      val tlb   = new FrontendTLBIO
      val busy  = Output(Bool())
      val flush = Input(Bool())
    })

    val req = Wire(Flipped(Decoupled(new FgDMAControlRequest(config))))
    io.req.ready    := req.ready
    req.valid       := io.req.valid
    req.bits.vaddr  := io.req.bits.vaddr
    req.bits.lrange := io.req.bits.lrange
    req.bits.status := io.req.bits.status
    req.bits.rob_id := io.req.bits.rob_id

    //-----------------------------------------------
    // track outstanding transactions for each operation
    //-----------------------------------------------
    val tracker = Module(new FgDMATracker(config, max_xfer_bytes, 1))

    //-----------------------------------------------
    // tlb translations and tilelink txn dispatch
    //-----------------------------------------------
    val control = Module(new FgDMAControl(config, max_xfer_bytes, true))
    control.io.req    <> req
    control.io.nextid := tracker.io.nextid
    tracker.io.alloc  <> control.io.alloc
    io.tlb            <> control.io.tlb
    control.io.flush  := io.flush

    //-----------------------------------------------
    // merge response beats into buffer
    //-----------------------------------------------
    val merger = Module(new FgDMABeatMerger(config, max_xfer_bytes))
    tracker.io.peek(0) <> merger.io.peek
    tracker.io.pop     <> merger.io.pop
    io.resp            <> merger.io.resp

    //-----------------------------------------------
    // tile-link A-channel request
    //-----------------------------------------------
    tl.a.valid := control.io.dispatch.valid
    control.io.dispatch.ready := tl.a.ready
    tl.a.bits := edge.Get(
      fromSource = control.io.dispatch.bits.xactid,
      toAddress  = control.io.dispatch.bits.paddr,
      lgSize     = control.io.dispatch.bits.txn_log2_bytes,
    )._2

    //-----------------------------------------------
    // tile-link D-channel response (systolic)
    //-----------------------------------------------
    tl.d.ready                       := merger.io.beat.ready
    merger.io.beat.valid             := tl.d.valid
    merger.io.beat.bits.xactid       := tl.d.bits.source
    merger.io.beat.bits.data         := tl.d.bits.data
    merger.io.beat.bits.beat_idx     := edge.count(tl.d)._4 
    merger.io.beat.bits.is_last_beat := edge.last(tl.d)

    //-----------------------------------------------
    // busy signal
    //-----------------------------------------------
    io.busy := tracker.io.busy || control.io.busy || merger.io.busy
  }
}

//===========================================================================
// DMA write interface (scratchpad -> dram)
// - max_xfer_bytes: max bytes written to dram in a multi-tilelink-txn request
//===========================================================================
class FgDMAStoreRequest[T <: Data]
  (val config: FgGemminiArrayConfig[T], val max_xfer_bytes: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val data   = UInt((max_xfer_bytes*8).W)
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange(config)
  val status = new MStatus
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

class FgDMAStoreResponse[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

class FgDMAStore[T <: Data](config: FgGemminiArrayConfig[T], 
  name: String = "dma-store", max_xfer_bytes: Int)
  (implicit p: Parameters) extends LazyModule {
  import config._

  val node = TLHelper.makeClientNode(name, IdRange(0, DMA_REQS))

  lazy val module = new LazyModuleImp(this) 
    with HasCoreParameters {
    // I/O interface
    val (tl, edge) = node.out(0)
    
    val io = IO(new Bundle {
      val req  = Flipped(Decoupled(
                    new FgDMAStoreRequest(config, max_xfer_bytes)))
      val resp = Decoupled(new FgDMAStoreResponse(config))
      val tlb  = new FrontendTLBIO
      val busy = Output(Bool())
      val flush = Input(Bool())
    })

    val req = Wire(Flipped(Decoupled(new FgDMAControlRequest(config))))
    io.req.ready    := req.ready
    req.valid       := io.req.valid
    req.bits.vaddr  := io.req.bits.vaddr
    req.bits.lrange := io.req.bits.lrange
    req.bits.status := io.req.bits.status
    req.bits.rob_id := io.req.bits.rob_id

    //-----------------------------------------------
    // track outstanding requests
    //-----------------------------------------------
    val tracker = Module(new FgDMATracker(config, max_xfer_bytes, 2))

    //-----------------------------------------------
    // tlb translations and tilelink txn dispatch
    //-----------------------------------------------
    val control = Module(new FgDMAControl(config, max_xfer_bytes, false))
    control.io.req    <> req
    control.io.nextid := tracker.io.nextid
    tracker.io.alloc  <> control.io.alloc
    io.tlb            <> control.io.tlb
    control.io.flush  := io.flush

    //-----------------------------------------------
    // track outstanding requests
    //-----------------------------------------------
    val splitter = Module(new FgDMASplitter(config, max_xfer_bytes))
    assert(splitter.io.req.ready === true.B, "splitter blocked on req")
    splitter.io.req.valid := io.req.valid
    splitter.io.req.bits  := io.req.bits
    splitter.io.txn       <> control.io.dispatch
    tracker.io.peek(0)    <> splitter.io.peek

    //-----------------------------------------------
    // send response when all tilelink txns are complete
    //-----------------------------------------------
    val responder = Module(new FgDMAStoreResponder(config, max_xfer_bytes))
    tracker.io.peek(1) <> responder.io.peek
    tracker.io.pop     <> responder.io.pop
    io.resp            <> responder.io.resp

    //-----------------------------------------------
    // tile-link A-channel request
    //-----------------------------------------------
    splitter.io.tl_a.ready := tl.a.ready
    tl.a.valid := splitter.io.tl_a.valid
    tl.a.bits := Mux(splitter.io.tl_a.bits.is_full, 
      edge.Put(
        fromSource = splitter.io.tl_a.bits.xactid,
        toAddress  = splitter.io.tl_a.bits.paddr,
        lgSize     = splitter.io.tl_a.bits.log2_bytes,
        data       = splitter.io.tl_a.bits.data.asUInt(),
      )._2,
      edge.Put(
        fromSource = splitter.io.tl_a.bits.xactid,
        toAddress  = splitter.io.tl_a.bits.paddr,
        lgSize     = splitter.io.tl_a.bits.log2_bytes,
        data       = splitter.io.tl_a.bits.data.asUInt(),
        mask       = splitter.io.tl_a.bits.mask.asUInt(),
      )._2)

    //-----------------------------------------------
    // tile-link D-channel response
    //-----------------------------------------------
    responder.io.tl_d.valid       := tl.d.valid
    tl.d.ready                    := responder.io.tl_d.ready
    responder.io.tl_d.bits.xactid := tl.d.bits.source

    //-----------------------------------------------
    // busy tracker
    //-----------------------------------------------
    io.busy := control.io.busy || tracker.io.busy || responder.io.busy
  }
}
