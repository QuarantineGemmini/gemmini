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

class FgLocalRange extends Bundle {
  val rows         = UInt(16.W)
  val cols         = UInt(16.W)
  val is_acc       = Bool()
  val is_accum     = Bool()
  val is_B_sp      = Bool()
  val garbage      = Bool()
  val sq_col_start = UInt(12.W)
  val row_start    = UInt(16.W)
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
  val vaddr  = UInt(coreMaxAddrBits.W)
  val data   = UInt((max_bytes*8).W)
  val len    = UInt(log2Up(max_bytes+1).W)
  val status = new MStatus
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

class FgStreamBeatData[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val xactid       = UInt(LOG2_MAX_DMA_REQS.W)
  val data         = UInt(DMA_BUS_BITWIDTH.W)
  val beat_lshift  = UInt()
  val is_last_beat = Bool()
}

//=======================================================================
// StreamReader
//=======================================================================
class FgStreamReader[T <: Data](config: GemminiArrayConfig[T], 
  name: String = "stream-reader", max_bytes: Int)
  (implicit p: Parameters) extends LazyModule 
{
  val core = LazyModule(new FgStreamReaderCore(config, name, max_bytes))
  val node = core.node

  lazy val module = new LazyModuleImp(this) {
    val io = IO(new Bundle {
      val req   = Flipped(Decoupled(new FgStreamReadRequest))
      val resp  = Decoupled(new FgStreamReadResponse(max_bytes))
      val tlb   = new FrontendTLBIO
      val busy  = Output(Bool())
      val flush = Input(Bool())
    })

    val tracker = Module(new FgXactTracker(config))
    val packer = Module(new FgBeatMerger(config, max_bytes))

    // I/O connections
    io.req               <> core.module.io.req
    io.tlb               <> core.module.io.tlb
    io.busy              := tracker.io.busy
    core.module.io.flush := io.flush
    io.resp              <> packer.io.resp

    // internal connections
    core.module.io.next_xactid <> tracker.io.next_xactid
    tracker.io.alloc           <> core.module.io.alloc
    tracker.io.peek            <> packer.io.peek
    packer.io.beat             <> core.module.io.beat
  }
}

class FgStreamCoreFsmReq extends Bundle {
  val 
    val req   = Flipped(Decoupled(new FgStreamReadRequest))

}
class FgTxnCore extends Module {
  val io = IO(new Bundle {
    val req   = Flipped(Decoupled(new FgStreamReadRequest))
    val state
    val resp  = Decoupled(new FgStreamReadResponse(max_bytes))
    val tlb   = new FrontendTLBIO
    val busy  = Output(Bool())
    val flush = Input(Bool())
  })

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
  io.tlb.req.bits.tlb_req.cmd         := M_XRD
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
  val cur_xactid       = io.next_xactid
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

  io.alloc.valid        := false.B
  io.alloc.lrange       := lrange
  io.alloc.rob_id       := rob_id
  io.alloc.start_rshift := start_shift // overridden on first txn
  io.alloc.txn_lshift   := bytes_requested
  io.alloc.is_last_txn  := is_last_txn

  //-----------------------------------------------
  // tile-link A-channel request
  //-----------------------------------------------
  tl.a.valid := false.B
  tl.a.bits  := edge.Get(fromSource = cur_xactid,
                         toAddress  = cur_paddr,
                         lgSize     = cur_log2_size)._2

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
      io.alloc.valid := tl.a.ready
      tl.a.valid     := io.alloc.ready
      when(io.alloc.fire()) {
        val next_vaddr      = cur_vaddr + cur_useful_bytes
        val next_vpn        = next_vaddr(coreMaxAddrBits-1, pgIdxBits)
        val needs_translate = (next_vpn =/= cur_vpn)

        when (is_first_txn) {
          io.alloc.start_rshift := cur_rshift
          start_rshift := cur_rshift
        }
        bytes_left := bytes_left - cur_useful_bytes
        cur_vaddr := next_vaddr

        when (io.alloc.is_last_txn) {
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

}

//===========================================================================
// StreamReaderCore
//===========================================================================
class StreamReaderCore[T <: Data](config: GemminiArrayConfig[T], 
  name: String = "stream-reader", max_bytes: Int)
  (implicit p: Parameters) extends LazyModule {
  import config._

  val node = TLHelper.makeClientNode(name, IdRange(0, MAX_DMA_REQS))

  lazy val module = new LazyModuleImp(this) 
    with HasCoreParameters with MemoryOpConstants {
    //-----------------------------------------------
    // I/O interface
    //-----------------------------------------------
    val (tl, edge) = node.out(0)

    val io = IO(new Bundle {
      val req         = Flipped(Decoupled(new FgStreamReadRequest)
      val next_xactid = Input(UInt(LOG2_MAX_DMA_REQS.W))
      val alloc       = Decoupled(new XactTrackerAllocIO(config, max_bytes))
      val beat        = Decoupled(new FgStreamReadBeat(config, max_bytes))
      val tlb         = new FrontendTLBIO
      val flush       = Input(Bool())
    })
    io.req.ready      := false.B
    io.alloc.valid    := false.B
    io.beat.valid     := false.B
    io.tlb.req.valid  := false.B
    io.tlb.resp.ready := false.B

    //-----------------------------------------------
    // active request
    //-----------------------------------------------
    val req = Reg(new FgStreamReadRequest)
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
    io.tlb.req.bits.tlb_req.cmd         := M_XRD
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
    val cur_xactid       = io.next_xactid
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

    io.alloc.valid        := false.B
    io.alloc.lrange       := lrange
    io.alloc.rob_id       := rob_id
    io.alloc.start_rshift := start_shift // overridden on first txn
    io.alloc.txn_lshift   := bytes_requested
    io.alloc.is_last_txn  := is_last_txn

    //-----------------------------------------------
    // tile-link A-channel request
    //-----------------------------------------------
    tl.a.valid := false.B
    tl.a.bits  := edge.Get(fromSource = cur_xactid,
                           toAddress  = cur_paddr,
                           lgSize     = cur_log2_size)._2

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
        io.alloc.valid := tl.a.ready
        tl.a.valid     := io.alloc.ready
        when(io.alloc.fire()) {
          val next_vaddr      = cur_vaddr + cur_useful_bytes
          val next_vpn        = next_vaddr(coreMaxAddrBits-1, pgIdxBits)
          val needs_translate = (next_vpn =/= cur_vpn)

          when (is_first_txn) {
            io.alloc.start_rshift := cur_rshift
            start_rshift := cur_rshift
          }
          bytes_left := bytes_left - cur_useful_bytes
          cur_vaddr := next_vaddr

          when (io.alloc.is_last_txn) {
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
    // tile-link D-channel response (systolic)
    //-----------------------------------------------
    tl.d.ready                := io.beatData.ready
    io.beat.valid             := tl.d.valid
    io.beat.bits.xactid       := tl.d.bits.source
    io.beat.bits.data         := tl.d.bits.data
    io.beat.bits.beat_lshift  := edge.count(tl.d) << LOG2_DMA_BUS_BITWIDTH
    io.beat.bits.is_last_beat := edge.last(tl.d)
  }
}

class StreamWriter[T <: Data](config: GemminiArrayConfig[T], 
  name: String = "stream-writer", max_bytes: Int)
  (implicit p: Parameters) extends LazyModule {
  import config._

  val node = TLHelper.makeClientNode(name, IdRange(0, MAX_DMA_REQS))

  lazy val module = new LazyModuleImp(this) 
    with HasCoreParameters with MemoryOpConstants {
    //-----------------------------------------------
    // I/O interface
    //-----------------------------------------------
    val (tl, edge) = node.out(0)
    
    val io = IO(new Bundle {
      val req   = Flipped(Decoupled(new StreamWriteRequest(max_bytes)))
      val tlb   = new FrontendTLBIO
      val busy  = Output(Bool())
      val flush = Input(Bool())
    })
    io.req.ready      := false.B
    io.tlb.req.valid  := false.B
    io.tlb.resp.ready := false.B

    //-----------------------------------------------
    // active request
    //-----------------------------------------------
    val req = Reg(new FgStreamWriteRequest)
    val mstatus = req.status
    val rob_id  = req.rob_id
    val len     = req.len
    val data    = req.data

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
    io.tlb.req.bits.tlb_req.cmd         := M_XRD
    io.tlb.req.bits.status              := mstatus



    val vpn = req.vaddr(coreMaxAddrBits-1, pgIdxBits)

    // TODO this only needs to count up to (dataBytes/aligned_to), right?
    val bytesSent = Reg(UInt(log2Ceil(dataBytes).W))  
    // val bytesLeft = dataBytes.U - bytesSent
    val bytesLeft = req.len - bytesSent

    val xactBusy = RegInit(0.U(nXacts.W))
    val xactOnehot = PriorityEncoderOH(~xactBusy)
    val xactId = OHToUInt(xactOnehot)

    val xactBusy_add = Mux(tl.a.fire(), (1.U << xactId).asUInt(), 0.U)
    val xactBusy_remove = ~Mux(tl.d.fire(), (1.U << tl.d.bits.source).asUInt(), 0.U)
    xactBusy := (xactBusy | xactBusy_add) & xactBusy_remove.asUInt()

    io.busy := xactBusy.orR || (state =/= s_IDLE)

    //=======================================================================
    // Select the size and mask of the TileLink request
    //=======================================================================
    class Packet extends Bundle {
      val size = UInt(log2Ceil(maxBytes).W)
      val log2_size = UInt(log2Ceil(log2Ceil(maxBytes)).W)
      val mask = Vec(maxBeatsPerReq, Vec(beatBytes, Bool()))
      val paddr = UInt(paddrBits.W)
      val is_full = Bool()

      def bytes_written(dummy: Int = 0) = PopCount(mask.flatten)
      def total_beats(dummy: Int = 0) = Mux(size < beatBytes.U, 1.U, size / beatBytes.U)
    }

    val smallest_write_size = aligned_to max beatBytes
    val write_sizes = (smallest_write_size to maxBytes by aligned_to).
      filter(s => isPow2(s)).
      filter(s => s % beatBytes == 0).
      filter(s => s <= dataBytes*2 || s == smallest_write_size)
    val write_packets = write_sizes.map { s =>
      val log2_s = log2Ceil(s)
      val paddr_aligned_to_size = if (s == 1) paddr else Cat(paddr(paddrBits-1, log2_s), 0.U(log2_s.W))

      val mask = (0 until maxBytes).map { i =>
        if (s > 1) {
          val paddr_offset = paddr(log2_s-1, 0)

          i.U >= paddr_offset &&
            i.U < paddr_offset +& bytesLeft
        } else {
          true.B
        } && (i < s).B
      }

      val packet = Wire(new Packet())
      packet.size := s.U
      packet.log2_size := log2_s.U
      packet.mask := VecInit(mask.grouped(beatBytes).map(v => VecInit(v)).toSeq)
      packet.paddr := paddr_aligned_to_size
      packet.is_full := mask.take(s).reduce(_ && _)

      packet
    }
    val best_write_packet = write_packets.reduce { (acc, p) =>
      Mux(p.bytes_written() > acc.bytes_written(), p, acc)
    }
    val write_packet = RegEnableThru(best_write_packet, state === s_writing_new_block)

    val write_size = write_packet.size
    val log2_write_size = write_packet.log2_size
    val write_beats = write_packet.total_beats()
    val write_paddr = write_packet.paddr
    val write_full = write_packet.is_full

    val beatsLeft = Reg(UInt(log2Up(maxBytes/aligned_to).W))
    val beatsSent = Mux(state === s_writing_new_block, 0.U, write_beats - beatsLeft)

    val write_mask = write_packet.mask(beatsSent)
    val write_shift = PriorityEncoder(write_mask)

    val bytes_written_this_beat = PopCount(write_mask)

    // Firing off TileLink write requests
    val putFull = edge.Put(
      fromSource = RegEnableThru(xactId, state === s_writing_new_block),
      toAddress = write_paddr,
      lgSize = log2_write_size,
      data = (req.data >> (bytesSent * 8.U)).asUInt()
    )._2

    val putPartial = edge.Put(
      fromSource = RegEnableThru(xactId, state === s_writing_new_block),
      toAddress = write_paddr,
      lgSize = log2_write_size,
      data = ((req.data >> (bytesSent * 8.U)) << (write_shift * 8.U)).asUInt(),
      mask = write_mask.asUInt()
    )._2

    tl.a.valid := (state === s_writing_new_block || state === s_writing_beats) && !xactBusy.andR()
    tl.a.bits := Mux(write_full, putFull, putPartial)
    tl.d.ready := xactBusy.orR()

    when (tl.a.fire()) {
      when (state === s_writing_new_block) {
        beatsLeft := write_beats - 1.U

        val next_vaddr = req.vaddr + bytes_written_this_beat
        val new_page = next_vaddr(pgIdxBits-1, 0) === 0.U
        req.vaddr := next_vaddr

        bytesSent := bytesSent + bytes_written_this_beat

        when (write_beats === 1.U) {
          when (bytes_written_this_beat >= bytesLeft) {
            // We're done with this request at this point
            state_machine_ready_for_req := true.B
            state := s_idle
          }.elsewhen (new_page) {
            state := s_translate_req
          }
        }.otherwise {
          state := s_writing_beats
        }
      }.elsewhen(state === s_writing_beats) {
        beatsLeft := beatsLeft - 1.U
        bytesSent := bytesSent + bytes_written_this_beat

        when (beatsLeft === 0.U) {
          val new_page = req.vaddr(pgIdxBits-1, 0) === 0.U

          when (bytes_written_this_beat >= bytesLeft) {
            // We're done with this request at this point
            state_machine_ready_for_req := true.B
            state := s_idle
          }.elsewhen(new_page) {
            state := s_translate_req
          }.otherwise {
            state := s_writing_new_block
          }
        }
      }
    }

    // Accepting requests to kick-start the state machine
    when (io.req.fire()) {
      req := io.req.bits
      bytesSent := 0.U

      val vpn_translated = last_vpn_translated_valid &&
        last_vpn_translated === io.req.bits.vaddr(coreMaxAddrBits-1, pgIdxBits)
      state := Mux(vpn_translated, 
                   s_writing_new_block, 
                   s_translate_req)
    }
  }
}
