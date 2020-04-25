package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink.{TLIdentityNode, TLXbar}
import Util._

//===========================================================================
// Load/StoreController <-> Scratchpad Interfaces
//===========================================================================
class FgScratchpadMemRequest[T <: Data: Arithmetic]
  (config: GemminiArrayConfig[T])(implicit p: Parameters) extends CoreBundle 
  import config._
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange
  val status = new MStatus
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
  override def cloneType: this.type 
    = new FgScratchpadMemRequest(config).asInstanceOf[this.type]
}

class FgScratchpadMemResponse[T <: Data: Arithmetic]
  (config: GemminiArrayConfig[T])(implicit p: Parameters) extends CoreBundle 
  import config._
  val rob_id = Valid(UInt(LOG2_ROB_ENTRIES.W))
}

class FgScratchpadMemIO[T <: Data: Arithmetic]
  (config: GemminiArrayConfig[T])(implicit p: Parameters) extends CoreBundle {
  val req  = Decoupled(new FgScratchpadMemRequest(config))
  val resp = Flipped(new FgScratchpadMemResponse(config))
  override def cloneType: this.type 
    = new FgScratchpadMemIO(config).asInstanceOf[this.type]
}

//===========================================================================
// ExecController <-> Scratchpad Interfaces
//===========================================================================
class ScratchpadReadReq(val n: Int) extends Bundle {
  val addr = UInt(log2Ceil(n).W)
  val fromDMA = Bool()
}

class ScratchpadReadResp(val w: Int) extends Bundle {
  val data = UInt(w.W)
  val fromDMA = Bool()
}

class ScratchpadReadIO(val n: Int, val w: Int) extends Bundle {
  val req = Decoupled(new ScratchpadReadReq(n))
  val resp = Flipped(Decoupled(new ScratchpadReadResp(w)))
}

class ScratchpadWriteIO(val n: Int, val w: Int, val mask_len: Int) 
  extends Bundle {
  val en = Output(Bool())
  val addr = Output(UInt(log2Ceil(n).W))
  val mask = Output(Vec(mask_len, Bool()))
  val data = Output(UInt(w.W))
}

//============================================================================
// scratchpad bank
//============================================================================
class ScratchpadBank(n: Int, w: Int, mem_pipeline: Int, aligned_to: Int) 
  extends Module {
  // This is essentially a pipelined SRAM with the ability to 
  // stall pipeline stages
  require(w % aligned_to == 0 || w < aligned_to)
  // How many mask bits are there?
  val mask_len = (w / (aligned_to * 8)) max 1 
  // What datatype does each mask bit correspond to?
  val mask_elem = UInt((w min (aligned_to * 8)).W) 

  val io = IO(new Bundle {
    val read = Flipped(new ScratchpadReadIO(n, w))
    val write = Flipped(new ScratchpadWriteIO(n, w, mask_len))
  })

  val mem = SyncReadMem(n, Vec(mask_len, mask_elem))

  when (io.write.en) {
    if (aligned_to >= w)
      mem.write(io.write.addr, 
        io.write.data.asTypeOf(Vec(mask_len, mask_elem)))
    else
      mem.write(io.write.addr, 
        io.write.data.asTypeOf(Vec(mask_len, mask_elem)), io.write.mask)
  }

  val raddr = io.read.req.bits.addr
  val ren = io.read.req.fire()
  val rdata = mem.read(raddr, ren).asUInt()
  val fromDMA = io.read.req.bits.fromDMA

  // Make a queue which buffers the result of an SRAM read if it 
  // can't immediately be consumed
  val q = Module(new Queue(new ScratchpadReadResp(w), 1, true, true))
  q.io.enq.valid := RegNext(ren)
  q.io.enq.bits.data := rdata
  q.io.enq.bits.fromDMA := RegNext(fromDMA)

  val q_will_be_empty = 
    (q.io.count +& q.io.enq.fire()) - q.io.deq.fire() === 0.U
  io.read.req.ready := q_will_be_empty

  // Build the rest of the resp pipeline
  val rdata_p = Pipeline(q.io.deq, mem_pipeline)
  io.read.resp <> rdata_p
}

//============================================================================
// TODO find a more elegant way to move data into accumulator
// TODO replace the SRAM types with Vec[Vec[inputType]], rather than just 
//      simple UInts
// TODO support unaligned accesses, for both multiple and single matrix loads
// TODO scratchpad is currently broken when one row is larger than dataBits. 
//      The requests arrive out-of-order, meaning that half of one row might 
//      arrive after the first have of another row. Some kind of re-ordering 
//      buffer may be needed
//============================================================================
class FgScratchpad[T <: Data: Arithmetic](config: FgGemminiArrayConfig[T])
    (implicit p: Parameters) extends LazyModule {
  import config._

  val maxBytes = dma_maxbytes
  val dataBits = dma_buswidth

  val block_rows = meshRows * tileRows
  val block_cols = meshColumns * tileColumns
  val spad_w = inputType.getWidth *  block_cols
  val acc_w = accType.getWidth * block_cols

  val id_node = TLIdentityNode()
  val xbar_node = TLXbar()

  val reader_B = LazyModule(new StreamReader(
    config, max_in_flight_reqs, dataBits, maxBytes, spad_w, acc_w, aligned_to,
    SP_ROWS, ACC_ROWS, DIM))

  val writer = LazyModule(new StreamWriter(
    max_in_flight_reqs, dataBits, maxBytes, spad_w, aligned_to))

  xbar_node := reader_A.node
  xbar_node := reader_B.node
  xbar_node := reader_D.node
  xbar_node := writer_C.node
  id_node := xbar_node

  lazy val module = new LazyModuleImp(this) with HasCoreParameters {
    val io = IO(new Bundle {
      // DMA ports
      val dma = new Bundle {
        val read = Flipped(new FgScratchpadReadMemIO(local_addr_t))
        val write = Flipped(new FgScratchpadWriteMemIO(local_addr_t))
      }

      // B-tile SRAM ports
      val srams = new Bundle {
        val read = Flipped(Vec(2, new ScratchpadReadIO(
          fg_sp_bank_entries, spad_w)))
        val write = Flipped(Vec(2, new ScratchpadWriteIO(
          fg_sp_bank_entries, spad_w, (spad_w / (aligned_to * 8)) max 1)))
      }

      // SRAM ports
      val srams = new Bundle {
        val read = Flipped(Vec(sp_banks, new ScratchpadReadIO(
          sp_bank_entries, spad_w)))
        val write = Flipped(Vec(sp_banks, new ScratchpadWriteIO(
          sp_bank_entries, spad_w, (spad_w / (aligned_to * 8)) max 1)))
      }

      // Accumulator ports
      val acc = new Bundle {
        val read = Flipped(Vec(acc_banks, new AccumulatorReadIO(
          acc_bank_entries, log2Up(accType.getWidth), 
          Vec(meshColumns, Vec(tileColumns, inputType)))))
        val write = Flipped(Vec(acc_banks, new AccumulatorWriteIO(
          acc_bank_entries, Vec(meshColumns, Vec(tileColumns, accType)))))
      }

      // TLB ports
      val tlb = Vec(2, new FrontendTLBIO)

      // Misc. ports
      val busy = Output(Bool())
      val flush = Input(Bool())
    })

    val write_dispatch_q = Queue(io.dma.write.req)

    write_dispatch_q.ready := false.B

    val write_issue_q = Module(new Queue(
      new ScratchpadMemWriteRequest(local_addr_t), mem_pipeline+1, pipe=true))

    write_issue_q.io.enq.valid := false.B
    write_issue_q.io.enq.bits := write_dispatch_q.bits

    val writeq_deq = write_issue_q.io.deq
    val writer_req = writer.module.io.req

    val writeData = Wire(Valid(UInt((spad_w max acc_w).W)))
    writeData.valid := false.B
    writeData.bits  := DontCare

    writeq_deq.ready       := writer_req.ready && writeData.valid
    writer_req.valid       := writeq_deq.valid && writeData.valid
    writer_req.bits.vaddr  := writeq_deq.bits.vaddr
    writer_req.bits.len    := writeq_deq.bits.len * ITYPE_BYTES.U
    writer_req.bits.data   := writeData.bits
    writer_req.bits.status := writeq_deq.bits.status

    io.dma.write.resp.valid := false.B
    io.dma.write.resp.bits.cmd_id := write_dispatch_q.bits.cmd_id

    //========================================================================
    // LoadCmd datapath (dram->spad) 
    //========================================================================
    val read_issue_q = Module(new Queue(
      new ScratchpadMemReadRequest(local_addr_t), mem_pipeline+1, pipe=true))

    read_issue_q.io.enq <> io.dma.read.req

    val readq_deq   = read_issue_q.io.deq
    val reader_resp = reader.module.io.resp
    val reader_req  = reader.module.io.req

    read_issue_q.io.deq.ready := reader_req.ready
    reader_req.valid          := readq_deq.valid
    reader_req.bits.vaddr     := readq_deq.bits.vaddr
    reader_req.bits.spaddr    := Mux(readq_deq.bits.laddr.is_acc_addr,
                                     readq_deq.bits.laddr.full_acc_addr(), 
                                     readq_deq.bits.laddr.full_sp_addr())
    reader_req.bits.len       := readq_deq.bits.len
    reader_req.bits.is_acc    := readq_deq.bits.laddr.is_acc_addr
    reader_req.bits.status    := readq_deq.bits.status
    reader_req.bits.cmd_id    := readq_deq.bits.cmd_id

    reader_resp.ready := false.B

    io.dma.read.resp.valid          := reader_resp.fire() && 
                                       reader_resp.bits.last
    io.dma.read.resp.bits.cmd_id    := reader_resp.bits.cmd_id
    io.dma.read.resp.bits.bytesRead := reader_resp.bits.bytes_read

    io.tlb(0) <> writer.module.io.tlb
    io.tlb(1) <> reader.module.io.tlb

    //========================================================================
    writer.module.io.flush := io.flush
    reader.module.io.flush := io.flush

    io.busy := writer.module.io.busy || 
               reader.module.io.busy || 
               write_issue_q.io.deq.valid
  
    //=======================================================================
    // B-tile scratchpad banks
    // - NOTE: byte-aligned, so inputType % 8 must == 0 for exec-controller
    //   writes to work!
    //=======================================================================
    {
      val B_banks = Seq.fill(2) { Module(
        new ScratchpadBank(FG_DIM, SP_BITS_PER_ROW, mem_pipeline, aligned_to)) 
      }
      val bank_ios = VecInit(banks.map(_.io))

      // Getting the output of the bank that's about to be issued to the writer
      val bank_issued_io = bank_ios(write_issue_q.io.deq.bits.laddr.sp_bank())

      when (!write_issue_q.io.deq.bits.laddr.is_acc_addr) {
        writeData.valid := bank_issued_io.read.resp.valid && 
                           bank_issued_io.read.resp.bits.fromDMA
        writeData.bits  := bank_issued_io.read.resp.bits.data
      }

      // Reading from the SRAM banks
      bank_ios.zipWithIndex.foreach { case (bio, i) =>
        val ex_read_req = io.srams.read(i).req
        val exread = ex_read_req.valid

        // TODO we tie the write dispatch queue's, 
        // and write issue queue's, ready and valid signals together here
        val dmawrite = write_dispatch_q.valid && 
                       write_issue_q.io.enq.ready &&
                       !write_dispatch_q.bits.laddr.is_acc_addr && 
                       write_dispatch_q.bits.laddr.sp_bank() === i.U

        bio.read.req.valid := exread || dmawrite
        ex_read_req.ready := bio.read.req.ready

        // The ExecuteController gets priority when reading from SRAMs
        when (exread) {
          bio.read.req.bits.addr := ex_read_req.bits.addr
          bio.read.req.bits.fromDMA := false.B
        }.elsewhen (dmawrite) {
          bio.read.req.bits.addr := write_dispatch_q.bits.laddr.sp_row()
          bio.read.req.bits.fromDMA := true.B

          when (bio.read.req.fire()) {
            write_dispatch_q.ready := true.B
            write_issue_q.io.enq.valid := true.B

            io.dma.write.resp.valid := true.B
          }
        }.otherwise {
          bio.read.req.bits := DontCare
        }

        val ex_read_resp = io.srams.read(i).resp
        // I believe we don't need to check that write_issue_q is valid 
        // here, because if the SRAM's resp is valid, then that means that 
        // the write_issue_q's deq should also be valid
        val dma_resp_ready = 
          writer.module.io.req.ready &&
          !write_issue_q.io.deq.bits.laddr.is_acc_addr && 
          write_issue_q.io.deq.bits.laddr.sp_bank() === i.U 

        bio.read.resp.ready := Mux(bio.read.resp.bits.fromDMA, 
                                   dma_resp_ready, 
                                   ex_read_resp.ready)
        // TODO should we AND this with fromDMA?
        ex_read_resp.valid := bio.read.resp.valid 
        ex_read_resp.bits := bio.read.resp.bits
      }

      // Writing to the SRAM banks
      bank_ios.zipWithIndex.foreach { case (bio, i) =>
        val exwrite = io.srams.write(i).en
        val dmaread = reader.module.io.resp.valid &&
          !reader.module.io.resp.bits.is_acc && reader.module.io.resp.bits.addr.asTypeOf(local_addr_t).sp_bank() === i.U

        bio.write.en := exwrite || dmaread

        when (exwrite) {
          bio.write.addr := io.srams.write(i).addr
          bio.write.data := io.srams.write(i).data
          bio.write.mask := io.srams.write(i).mask
        }.elsewhen (dmaread) {
          bio.write.addr := reader.module.io.resp.bits.addr
          bio.write.data := reader.module.io.resp.bits.data
          bio.write.mask := reader.module.io.resp.bits.mask take ((spad_w / (aligned_to * 8)) max 1)

          reader.module.io.resp.ready := true.B // TODO we combinationally couple valid and ready signals
        }.otherwise {
          bio.write.addr := DontCare
          bio.write.data := DontCare
          bio.write.mask := DontCare
        }
      }
    }
    {
      val banks = Seq.fill(sp_banks) { Module(new ScratchpadBank(sp_bank_entries, spad_w, mem_pipeline, aligned_to)) }
      val bank_ios = VecInit(banks.map(_.io))

      // Getting the output of the bank that's about to be issued to the writer
      val bank_issued_io = bank_ios(write_issue_q.io.deq.bits.laddr.sp_bank())

      when (!write_issue_q.io.deq.bits.laddr.is_acc_addr) {
        writeData.valid := bank_issued_io.read.resp.valid && 
                           bank_issued_io.read.resp.bits.fromDMA
        writeData.bits  := bank_issued_io.read.resp.bits.data
      }

      // Reading from the SRAM banks
      bank_ios.zipWithIndex.foreach { case (bio, i) =>
        val ex_read_req = io.srams.read(i).req
        val exread = ex_read_req.valid

        // TODO we tie the write dispatch queue's, 
        // and write issue queue's, ready and valid signals together here
        val dmawrite = write_dispatch_q.valid && 
                       write_issue_q.io.enq.ready &&
                       !write_dispatch_q.bits.laddr.is_acc_addr && 
                       write_dispatch_q.bits.laddr.sp_bank() === i.U

        bio.read.req.valid := exread || dmawrite
        ex_read_req.ready := bio.read.req.ready

        // The ExecuteController gets priority when reading from SRAMs
        when (exread) {
          bio.read.req.bits.addr := ex_read_req.bits.addr
          bio.read.req.bits.fromDMA := false.B
        }.elsewhen (dmawrite) {
          bio.read.req.bits.addr := write_dispatch_q.bits.laddr.sp_row()
          bio.read.req.bits.fromDMA := true.B

          when (bio.read.req.fire()) {
            write_dispatch_q.ready := true.B
            write_issue_q.io.enq.valid := true.B

            io.dma.write.resp.valid := true.B
          }
        }.otherwise {
          bio.read.req.bits := DontCare
        }

        val ex_read_resp = io.srams.read(i).resp
        val dma_resp_ready = writer.module.io.req.ready &&
          !write_issue_q.io.deq.bits.laddr.is_acc_addr && write_issue_q.io.deq.bits.laddr.sp_bank() === i.U // I believe we don't need to check that write_issue_q is valid here, because if the SRAM's resp is valid, then that means that the write_issue_q's deq should also be valid

        bio.read.resp.ready := Mux(bio.read.resp.bits.fromDMA, dma_resp_ready, ex_read_resp.ready)
        ex_read_resp.valid := bio.read.resp.valid // TODO should we AND this with fromDMA?
        ex_read_resp.bits := bio.read.resp.bits
      }

      // Writing to the SRAM banks
      bank_ios.zipWithIndex.foreach { case (bio, i) =>
        val exwrite = io.srams.write(i).en
        val dmaread = reader.module.io.resp.valid &&
          !reader.module.io.resp.bits.is_acc && reader.module.io.resp.bits.addr.asTypeOf(local_addr_t).sp_bank() === i.U

        bio.write.en := exwrite || dmaread

        when (exwrite) {
          bio.write.addr := io.srams.write(i).addr
          bio.write.data := io.srams.write(i).data
          bio.write.mask := io.srams.write(i).mask
        }
        .elsewhen (dmaread) {
          bio.write.addr := reader.module.io.resp.bits.addr
          bio.write.data := reader.module.io.resp.bits.data
          bio.write.mask := reader.module.io.resp.bits.mask take 
                            ((spad_w / (aligned_to * 8)) max 1)

          // TODO we combinationally couple valid and ready signals
          reader.module.io.resp.ready := true.B 
        }.otherwise {
          bio.write.addr := DontCare
          bio.write.data := DontCare
          bio.write.mask := DontCare
        }
      }
    }

    {
      val acc_row_t  = Vec(meshColumns, Vec(tileColumns, accType))
      val spad_row_t = Vec(meshColumns, Vec(tileColumns, inputType))

      val banks = Seq.fill(acc_banks) { Module(new AccumulatorMem(config)) }
      val bank_ios = VecInit(banks.map(_.io))

      // Getting the output of the bank that's about to be issued to the 
      // writer
      val bank_issued_io = 
        bank_ios(write_issue_q.io.deq.bits.laddr.acc_bank())

      when (write_issue_q.io.deq.bits.laddr.is_acc_addr) {
        writeData.valid := bank_issued_io.read.resp.valid && 
                           bank_issued_io.read.resp.bits.fromDMA
        writeData.bits  := bank_issued_io.read.resp.bits.data.asUInt()
      }

      //---------------------------------------------------------------------
      // Reading from the Accumulator banks
      //---------------------------------------------------------------------
      bank_ios.zipWithIndex.foreach { case (bio, i) =>
        val ex_read_req = io.acc.read(i).req
        val exread = ex_read_req.valid

        // TODO we tie the write dispatch queue's, and write issue queue's, 
        // ready and valid signals together here
        val dmawrite = 
          write_dispatch_q.valid && 
          write_issue_q.io.enq.ready &&
          write_dispatch_q.bits.laddr.is_acc_addr && 
          write_dispatch_q.bits.laddr.acc_bank() === i.U

        bio.read.req.valid := exread || dmawrite
        bio.read.req.bits.shift := ex_read_req.bits.shift
        bio.read.req.bits.relu6_shift := ex_read_req.bits.relu6_shift
        bio.read.req.bits.act := ex_read_req.bits.act
        ex_read_req.ready := bio.read.req.ready

        // The ExecuteController gets priority when reading from 
        // accumulator banks
        when (exread) {
          bio.read.req.bits.addr := ex_read_req.bits.addr
          bio.read.req.bits.fromDMA := false.B
        }
        .elsewhen (dmawrite) {
          bio.read.req.bits.addr := write_dispatch_q.bits.laddr.acc_row()
          bio.read.req.bits.fromDMA := true.B

          when (bio.read.req.fire()) {
            write_dispatch_q.ready := true.B
            write_issue_q.io.enq.valid := true.B

            io.dma.write.resp.valid := true.B
          }
        }.otherwise {
          bio.read.req.bits := DontCare
        }

        val ex_read_resp = io.acc.read(i).resp
        // I believe we don't need to check that write_issue_q is valid here, 
        // because if the accumulator bank's resp is valid, then that means 
        // that the write_issue_q's deq should also be valid
        val dma_resp_ready = 
          writer.module.io.req.ready &&
          write_issue_q.io.deq.bits.laddr.is_acc_addr && 
          write_issue_q.io.deq.bits.laddr.acc_bank() === i.U 

        bio.read.resp.ready := Mux(bio.read.resp.bits.fromDMA, 
                                   dma_resp_ready, 
                                   ex_read_resp.ready)
        // TODO should we AND this with fromDMA?
        ex_read_resp.valid := bio.read.resp.valid
        ex_read_resp.bits := bio.read.resp.bits
      }

      //---------------------------------------------------------------------
      // Writing to the accumulator banks
      //---------------------------------------------------------------------
      bank_ios.zipWithIndex.foreach { case (bio, i) =>
        val exwrite = io.acc.write(i).en
        val dmaread = 
          reader_resp.valid &&
          reader_resp.bits.is_acc && 
          reader_resp.bits.addr.asTypeOf(local_addr_t).acc_bank() === i.U

        bio.write.en := exwrite || dmaread

        bio.write.addr := DontCare
        bio.write.data := DontCare
        bio.write.acc  := DontCare
        bio.write.mask := DontCare 
        when (exwrite) {
          bio.write.addr := io.acc.write(i).addr
          bio.write.data := io.acc.write(i).data
          bio.write.acc  := io.acc.write(i).acc
          // TODO add wmask to AccumulatorMem as well, so that we support 
          //      non-aligned accesses
          bio.write.mask := io.acc.write(i).mask //each bit == 1-byte
        }
        .elsewhen (dmaread) {
          bio.write.addr := reader_resp.bits.addr
          bio.write.data := reader_resp.bits.data.asTypeOf(acc_row_t)
          bio.write.acc  := false.B
          // TODO add wmask to eccumulatorMem as well, so that we support 
          //      non-aligned accesses
          bio.write.mask := reader_resp.bits.mask 
          // TODO we combinationally couple valid and ready signals
          reader_resp.ready := true.B 
        }
      }
    }
  }
}