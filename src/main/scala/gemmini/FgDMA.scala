package gemmini

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy.{IdRange, LazyModule, LazyModuleImp}
import freechips.rocketchip.tile.{CoreBundle, HasCoreParameters}
import testchipip.TLHelper
import freechips.rocketchip.rocket.MStatus
import freechips.rocketchip.rocket.constants.MemoryOpConstants

import Util._

class FgStreamCoreFsmReq extends Bundle {
  val req   = Flipped(Decoupled(new FgStreamReadRequest))
}

class FgLocalRange extends Bundle {
  val rows         = UInt(16.W)
  val cols         = UInt(16.W)
  val is_acc       = Bool()
  val is_accum     = Bool()
  val is_B_sp      = Bool()
  val garbage      = Bool()
  val sq_col_start = UInt(12.W)
  val row_start    = UInt(16.W)

  // utility functions
  def total_bytes(dummy: Int = 0) = rows * cols * 
                                    Mux(is_acc, OTYPE_BYTES.U, ITYPE_BYTES.U)
}

class FgStreamReadRequest(implicit p: Parameters) extends CoreBundle {
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange
  val status = new MStatus
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

class FgStreamReadResponse(max_bytes: Int)(implicit p: Parameters) 
  extends CoreBundle {
  val data   = UInt((max_bytes*8).W)
  val lrange = new FgLocalRange
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

class FgStreamWriteRequest(max_bytes: Int)(implicit p: Parameters) 
  extends CoreBundle {
  val data   = UInt((max_bytes*8).W)
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange
  val status = new MStatus
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

class FgStreamWriteResponse(implicit p: Parameters) extends CoreBundle {
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

// internal
class FgStreamBeatData[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val xactid       = UInt(LOG2_MAX_DMA_REQS.W)
  val data         = UInt(DMA_BUS_BITS.W)
  val beat_lshift  = UInt(LOG2_DMA_BUS_BYTES.W)
  val is_last_beat = Bool()
}

class FgDMAControlRequest[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange
  val status = new MStatus
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

class FgDmaTlTxn[T <: Data](config: GemminiArrayConfig[T], max_bytes: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val xactid    = UInt(LOG2_MAX_DMA_REQS.W)
  val paddr     = UInt(coreMaxAddrBits.W)
  val log2_size = UInt(log2Up(log2Up(max_bytes+1)).W)
}

class FgDMATlaRequest[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val xactid    = UInt(LOG2_MAX_DMA_REQS.W)
  val paddr     = UInt(DMA_BUS_BITS.W)
  val log2_size = ...
  val is_last_beat = Bool()
}

class FgTxnSplitterTxn[T <: Data]
  (config: GemminiArrayConfig[T], max_bytes: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val xactid    = UInt(LOG2_MAX_DMA_REQS.W)
  // just put everything in the entry-table
  //val paddr     = UInt(coreMaxAddrBits.W)
  //val log2_size = UInt(log2Up(log2Up(max_bytes+1)).W)
}

//===========================================================================
// tracked outstanding entry
//===========================================================================
class FgXactTrackerEntry[T <: Data]
  (config: GemminiArrayConfig[T], max_bytes: Int)(implicit p: Parameters) 
  extends CoreBundle {
  import config._
  val lrange         = new FgLocalRange
  val rob_id         = UInt(LOG2_ROB_ENTRIES.W)
  val useful_bytes   = UInt(LOG2_MAX_TRANSFER_BYTES.W)
  val data_start_idx = UInt(LOG2_MAX_DMA_BYTES.W)
  val txn_start_idx  = UInt(log2Up(max_bytes).W)
  val txn_bytes      = UInt(LOG2_MAX_TRANSFER_BYTES.W)
  val txn_log2_bytes = 
}

//=======================================================================
// StreamReader
//=======================================================================
class FgDMAReader[T <: Data](config: GemminiArrayConfig[T], 
  name: String = "stream-reader", max_bytes: Int)
  (implicit p: Parameters) extends LazyModule {
  import config._

  val node = TLHelper.makeClientNode(name, IdRange(0, MAX_DMA_REQS))

  lazy val module = new LazyModuleImp(this)
    with HasCoreParameters with MemoryOpConstants {
    // I/O interface
    val (tl, edge) = node.out(0)

    val io = IO(new Bundle {
      val req   = Flipped(Decoupled(new FgStreamReadRequest))
      val resp  = Decoupled(new FgDMAReadResponse(max_bytes))
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

    val control = Module(new FgDMAControl(config, max_bytes, true))
    control.io.req   <> req
    io.tlb           <> control.io.tlb
    io.busy          := control.io.busy
    control.io.flush := io.flush

    val merger = Module(new FgBeatMerger(config, max_bytes))
    val beat   = merger.io.beat
    control.io.peek <> merger.io.peek
    io.resp         <> merger.io.resp

    // tile-link A-channel request
    tl.a.valid := false.B
    tl.a.bits  := edge.Get(fromSource = control.io.tl_a.xactid,
                           toAddress  = control.io.tl_a.paddr,
                           lgSize     = control.io.tl_a.log2_size)._2

    // tile-link D-channel response (systolic)
    tl.d.ready             := merger.io.beat.ready
    beat.valid             := tl.d.valid
    beat.bits.xactid       := tl.d.bits.source
    beat.bits.data         := tl.d.bits.data
    beat.bits.beat_lshift  := edge.count(tl.d) << LOG2_DMA_BUS_BITWIDTH
    beat.bits.is_last_beat := edge.last(tl.d)
  }
}

//===========================================================================
// StreamWriter
//===========================================================================
class StreamWriter[T <: Data](config: GemminiArrayConfig[T], 
  name: String = "stream-writer", max_bytes: Int)
  (implicit p: Parameters) extends LazyModule {
  import config._

  val node = TLHelper.makeClientNode(name, IdRange(0, MAX_DMA_REQS))

  lazy val module = new LazyModuleImp(this) 
    with HasCoreParameters with MemoryOpConstants {
    // I/O interface
    val (tl, edge) = node.out(0)
    
    val io = IO(new Bundle {
      val req   = Flipped(Decoupled(new StreamWriteRequest(max_bytes)))
      val resp  = Decoupled(new FgDMAWriteResponse)
      val tlb   = new FrontendTLBIO
      val busy  = Output(Bool())
      val flush = Input(Bool())
    })

    val rob_id = RegInit(UInt(LOG2_ROB_ENTRIES.W))
    when(io.req.fire()) {
      rob_id := io.req.rob_id
    }
    io.resp.rob_id := rob_id

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
    val control = Module(new FgDMAControl(config, max_bytes, true))
    control.io.req   <> req
    io.tlb           <> control.io.tlb
    io.busy          := control.io.busy
    control.io.flush := io.flush

    //-----------------------------------------------
    // track outstanding requests
    //-----------------------------------------------
    val tracker = Module(new FgXactTracker(config, max_bytes))
    val next_xactid = tracker.io.next_xactid
    io.peek <> tracker.io.peek

    //-----------------------------------------------
    // track outstanding requests
    //-----------------------------------------------
    val splitter = Module(new FgBeatSplitter(config, max_bytes))
    splitter.io.req_fire := io.req.fire()
    splitter.io.req_data := io.req.bits
    splitter.io.tl_a <> control.io.tl_a

    //-----------------------------------------------
    // tile-link A-channel request
    //-----------------------------------------------
    splitter.io.tl_a.ready := tl.a.ready
    tl.a.valid := splitter.io.tl_a.valid
    tl.a.bits := Mux(splitter.io.tl_a.bits.is_full, 
      edge.Put(
        fromSource = splitter.io.tl_a.bits.xactid,
        toAddress  = splitter.io.tl_a.bits.paddr,
        lgSize     = splitter.io.tl_a.bits.log2_size,
        data       = splitter.io.tl_a.bits.data.asUInt(),
      )._2,
      edge.Put(
        fromSource = splitter.io.tl_a.bits.xactid,
        toAddress  = splitter.io.tl_a.bits.paddr,
        lgSize     = splitter.io.tl_a.bits.log2_size,
        data       = splitter.io.tl_a.bits.data.asUInt(),
        mask       = splitter.io.tl_a.bits.mask.asUInt(),
      )._2)

    //-----------------------------------------------
    // tile-link D-channel response
    //-----------------------------------------------
    tl.d.ready := true.B

    //-----------------------------------------------
    // send response when all tilelink txns are complete
    //-----------------------------------------------
    val responder = Module(new FgWriteResponder(config, max_bytes))
    responder.io.tl_d.valid       := tl.d.fire()
    responder.io.tl_d.bits.xactid := tl.d.bits.source
    tracker.io.peek               <> responder.io.peek
    io.resp                       <> responder.io.resp
  }
}

//===========================================================================
// Internal Guts
//===========================================================================
class FgDMAControl
  (config: GemminiArrayConfig[T], max_bytes: Int, is_read_mode: Boolean)
  (implicit p: Parameters) extends Module {
  import config._
  //-----------------------------------------------
  // I/O interface
  //-----------------------------------------------
  val io = IO(new Bundle {
    val req   = Flipped(Decoupled(new FgDMAControlRequest(config)))
    val alloc = Decoupled(new FgXactTrackerAllocIO(config))
    val tlb   = new FrontendTLBIO
    //val tl_a  = Output(Valid(new FgDMATlaRequest))
    val busy  = Output(Bool())
    val flush = Input(Bool())
  })

  //-----------------------------------------------
  // active request
  //-----------------------------------------------
  val req = Reg(new FgDMAControlRequest(config))
  val mstatus   = req.status
  val rob_id    = req.rob_id
  val lrange    = req.lrange
  val item_rows = lrange.rows
  val item_cols = lrange.cols
  val is_acc    = lrange.is_acc

  // initialized on io.req.fire()
  val total_useful_bytes     = RegInit(0.U(LOG2_MAX_TRANSFER_BYTES.W))
  val useful_bytes_left      = RegInit(0.U(LOG2_MAX_TRANSFER_BYTES.W))
  val useful_bytes_requested = total_useful_bytes - useful_bytes_left

  //-----------------------------------------------
  // TLB Address translation
  //-----------------------------------------------
  val cur_vaddr     = RegInit(0.U(coreMaxAddrBits.W))
  val cur_vpn       = cur_vaddr(coreMaxAddrBits-1, pgIdxBits)
  val cur_ppn       = RegInit(0.U((coreMaxAddrBits - pgIdxBits).W))
  val cur_ppn_valid = withReset(io.flush.toBool()) { RegInit(false.B) }
  val cur_paddr     = Cat(cur_ppn, cur_vaddr(pgIdxBits-1, 0))

  io.tlb.req.valid                    := false.B
  io.tlb.req.bits.tlb_req.vaddr       := Cat(cur_vpn, 0.U(pgIdxBits.W))
  io.tlb.req.bits.tlb_req.passthrough := false.B
  io.tlb.req.bits.tlb_req.size        := 0.U
  io.tlb.req.bits.tlb_req.cmd         := if(is_read_mode) M_XRD else M_XWR
  io.tlb.req.bits.status              := mstatus

  //-----------------------------------------------
  // (combinational) Select the size and mask of the TileLink request
  //-----------------------------------------------
  class Packet extends Bundle {
    val size         = UInt(log2Up(max_bytes+1).W)
    val log2_size    = UInt(log2Up(log2Up(max_bytes+1)).W)
    val useful_bytes = UInt(log2Up(max_bytes+1).W)
    val rshift       = UInt(log2Up(max_bytes).W)
    val paddr        = UInt(paddrBits.W)
  }

  val packets = (DMA_BUS_BYTES to MAX_DMA_BYTES by DMA_BUS_BYTES)
                .filter(size => isPow2(size)).map { size =>
    val log2_size     = log2Ceil(size)
    val paddr_aligned = Cat(cur_paddr(paddrBits-1, log2_size),
                            0.U(log2_size.W))
    val paddr_offset  = cur_paddr(log2_size-1, 0)

    val packet = Wire(new Packet())
    packet.size         := size.U
    packet.log2_size    := log2_size.U
    packet.useful_bytes := minOf(s.U - paddr_offset, useful_bytes_left)
    packet.rshift       := paddr_offset
    packet.paddr        := paddr_aligned

    packet
  }
  val best_packet = packets.reduce { (acc, p) =>
    Mux(p.useful_bytes > acc.useful_bytes, p, acc)
  }
  val cur_xactid       = next_xactid
  val cur_paddr        = best_packet.paddr
  val cur_log2_size    = best_packet.log2_size
  val cur_size         = best_packet.size
  val cur_rshift       = best_packet.rshift
  val cur_useful_bytes = best_packet.useful_bytes

  //-----------------------------------------------
  // allocate new tag for the tile-link request
  //-----------------------------------------------
  val start_rshift = RegInit(0.U(LOG2_MAX_TRANSFER_BYTES.W))
  val is_last_txn  = (cur_useful_bytes === useful_bytes_left)
  val is_first_txn = (useful_bytes_requested === 0.U)

  tracker.io.alloc.valid             := false.B
  tracker.io.alloc.bits.lrange       := lrange
  tracker.io.alloc.bits.rob_id       := rob_id
  tracker.io.alloc.bits.start_rshift := start_rshift // override on first txn
  tracker.io.alloc.bits.txn_lshift   := bytes_requested
  tracker.io.alloc.bits.paddr        := cur_paddr
  tracker.io.alloc.bits.log2_size    := cur_log2_size
  tracker.io.alloc.bits.useful_bytes := cur_useful_bytes

  // output transaction towards tile-link A-channel
  io.txn.valid       := false.B
  io.txn.bits.xactid := cur_xactid

  //-----------------------------------------------
  // FSM
  //-----------------------------------------------
  val (s_IDLE :: 
       s_START_TRANSLATE :: 
       s_FINISH_TRANSLATE :: 
       s_REQ_NEXT_CHUNK :: 
       Nil) = Enum(4)
  val state = RegInit(s_IDLE)

  def init_transfer(dummy : Int = 0) = {
    // TODO: remove this 1-row restriction, (a quite complicated task)
    assert(io.req.bits.lrange.item_rows === 1.U, 
      "cannot request more than 1 row at a time")
    val tmp_vpn = io.req.bits.vaddr(coreMaxAddrBits-1, pgIdxBits)
    val tmp_vpn_mapped  = cur_ppn_valid && (cur_vpn === tmp_vpn)
    val tmp_total_bytes = io.req.bits.lrange.item_cols * 
                          Mux(io.req.bits.lrange.is_acc,
                              OTYPE_BYTES, ITYPE_BYTES)
    req                := io.req.bits
    total_useful_bytes := tmp_total_bytes
    useful_bytes_left  := tmp_total_bytes
    cur_vaddr          := io.req.bits.vaddr
    state              := Mux(tmp_vpn_mapped, 
                              s_REQ_NEXT_CHUNK, s_START_TRANSLATE)
  }

  switch (state) {
    is (s_IDLE) {
      io.req.ready := true.B
      when (io.req.fire()) {
        init_transfer()
      }
    }
    is (s_START_TRANSLATE) {
      io.tlb.req.valid := true.B
      when (io.tlb.req.fire()) {
        state := s_FINISH_TRANSLATE
      }
    }
    is (s_FINISH_TRANSLATE) {
      io.tlb.resp.ready := true.B
      when (io.tlb.resp.fire()) {
        cur_ppn := io.tlb.resp.bits.paddr(paddrBits-1, pgIdxBits)
        cur_ppn_valid := true.B
        state := s_REQ_NEXT_CHUNK
      }
    }
    is (s_REQ_NEXT_CHUNK) {
      tracker.io.alloc.valid := io.txn.ready
      io.txn.valid           := tracker.io.alloc.ready
      when(io.txn.fire()) {
        val next_vaddr      = cur_vaddr + cur_useful_bytes
        val next_vpn        = next_vaddr(coreMaxAddrBits-1, pgIdxBits)
        val needs_translate = (next_vpn =/= cur_vpn)

        when (is_first_txn) {
          tracker.io.alloc.bits.start_rshift := cur_rshift
          start_rshift                       := cur_rshift
        }
        bytes_left := bytes_left - cur_useful_bytes
        cur_vaddr := next_vaddr

        when (tracker.io.alloc.bits.is_last_txn) {
          state := s_IDLE
          io.req.ready := true.B
          when (io.req.fire()) {
            init_transfer()
          }
        } .elsewhen (needs_translate) {
          state := s_START_TRANSLATE
        }
      }
    }
  }
  //-----------------------------------------------
  // busy signal
  //-----------------------------------------------
  io.busy := tracker.io.busy || (state =/= s_IDLE)
}
