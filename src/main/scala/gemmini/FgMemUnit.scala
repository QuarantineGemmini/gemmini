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
// MemOpController <-> MemUnit Interfaces
//===========================================================================
class FgMemUnitDMAReadReq[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val bank         = UInt(LOG2_FG_NUM.W))
  val row          = UInt(LOG2_FG_DIM.W))
  val fg_col_start = UInt(LOG2_FG_NUM.W))
  val cols         = UInt(LOG2_SP_ROW_ELEMS.W))
}

class FgMemUnitDMAReq[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange(config)
  val status = new MStatus
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

class FgMemUnitDMAResp[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

class FgMemUnitMemIO[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val req  = Decoupled(new FgMemUnitDMAReq(config))
  val resp = Flipped(Decoupled(new FgMemUnitDMAResp(config)))
}

//===========================================================================
// ExecController <-> MemUnit
//===========================================================================
class FgMemUnitExecReadReq[T <: Data]
  (config: GemminiArrayConfig[T], fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en           = Output(Bool())
  val row          = Output(UInt(LOG2_FG_DIM.W))
  val fg_col_start = Output(UInt(log2Up(fg_cols).W))
  val bank_start   = Output(UInt(LOG2_FG_NUM.W))
  val banks        = Output(UInt(LOG2_FG_NUM.W))
}

class FgMemUnitExecReadResp[T <: Data]
  (config: GemminiArrayConfig[T], port_fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val PORT_ELEMS = port_fg_cols * FG_DIM
  val PORT_BITS = PORT_ELEMS * ITYPE_BITS
  val data = Output(UInt(PORT_BITS.W))
}

class FgMemUnitExecReadIO[T <: Data]
  (config: GemminiArrayConfig[T], fg_cols: Int, port_fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  val req  = new FgMemUnitExecReadReq(config, fg_cols)
  val resp = Flipped(new FgMemUnitExecReadResp(config, port_fg_cols))
}

class FgMemUnitExecWriteReq[T <: Data]
  (config: GemminiArrayConfig[T], fg_cols: Int, port_fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val PORT_ELEMS = port_fg_cols * FG_DIM
  val PORT_BITS = PORT_ELEMS * OTYPE_BITS
  val en           = Output(Bool())
  val is_accum     = Output(Bool())
  val row          = Output(UInt(LOG2_FG_DIM.W))
  val fg_col_start = Output(UInt(LOG2_FG_NUM.W))
  val cols         = Output(UInt(log2Up(PORT_ELEMS + 1).W))
  val bank_start   = Output(UInt(LOG2_FG_NUM.W))
  val banks        = Output(UInt(LOG2_FG_NUM.W))
  val data         = Output(UInt(PORT_BITS.W))
}

//============================================================================
// FgMemUnit
// - supports multiple concurrent fine-grained systolic-array reads/writes
//============================================================================
class FgMemUnit[T <: Data: Arithmetic](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends LazyModule with HasCoreParameters {
  import config._

  val id_node = TLIdentityNode()
  val xbar_node = TLXbar()

  val readerA = LazyModule(new FgDMAReader(config,"readerA",A_SP_ROW_BYTES))
  val readerB = LazyModule(new FgDMAReader(config,"readerB",B_SP_ROW_BYTES))
  val readerD = LazyModule(new FgDMAReader(config,"readerD",D_ACC_ROW_BYTES))
  val writerC = LazyModule(new FgDMAWriter(config,"writerC",C_SP_ROW_BYTES))

  xbar_node := readerA.node
  xbar_node := readerB.node
  xbar_node := readerD.node
  xbar_node := writerC.node
  id_node   := xbar_node

  override lazy val module = new FgMemUnitModule(this, config)
}

class FgMemUnitModule[T <: Data: Arithmetic](outer: FgMemUnit[T])
  (implicit p: Parameters)
  extends LazyModuleImp(outer) with HasCoreParameters {
  import outer.{config,readerA,readerB,readerD,writerC}
  import config._
  //------------------------------------------
  // I/O interface
  //------------------------------------------
  val io = IO(new Bundle {
    val dma = new Bundle {
      val readA  = Flipped(new FgMemUnitMemIO(config))
      val readB  = Flipped(new FgMemUnitMemIO(config))
      val readD  = Flipped(new FgMemUnitMemIO(config))
      val writeC = Flipped(new FgMemUnitMemIO(config))
    }
    val tlb = new Bundle {
      val readA  = new FrontendTLBIO
      val readB  = new FrontendTLBIO
      val readD  = new FrontendTLBIO
      val writeC = new FrontendTLBIO
    }
    val exec = new Bundle {
      val readA  = Flipped(new FgMemUnitExecReadIO(config, SQRT_FG_NUM, 1))
      val readB  = Flipped(new FgMemUnitExecReadIO(config, FG_NUM, FG_NUM))
      val writeC = Flipped(new FgMemUnitExecWriteReq(config, FG_NUM, FG_NUM))
    }
    val busy = Output(Bool())
    val flush = Input(Bool())
  })

  //--------------------------------------------
  // setup tile-link DMA reader/writers
  //--------------------------------------------
  readerA.io.req <> io.dma.readA.req
  readerB.io.req <> io.dma.readB.req
  readerD.io.req <> io.dma.readD.req
  writerC.io.req <> io.dma.writeC.req

  Seq(readerA.io.resp, readerB.io.resp, readerD.io.resp).zip(
    Seq(io.dma.readA, io.dma.readB, io.dma.readD).foreach {case(pin, port) =>
      pin.ready := port.ready
      port.valid := port.valid
      port.bits.rob_id := pin.bits.rob_id
    })

  io.tlb.readA  <> readerA.io.tlb
  io.tlb.readB  <> readerB.io.tlb
  io.tlb.readD  <> readerD.io.tlb
  io.tlb.writeD <> writerC.io.tlb

  readerA.io.flush := io.flush
  readerB.io.flush := io.flush
  readerD.io.flush := io.flush
  writerC.io.flush := io.flush

  io.busy := readerA.io.busy || readerB.io.busy || 
             readerD.io.busy || writerC.io.busy

  //========================================================================
  {
    // A-banks
    val banks = Seq.fill(FG_DIM) { Module( new FgScratchpadBank(config)) }
    val bank_ios = VecInit(banks.map(_.io))

    //======================================================================
    // read-datapath
    //======================================================================
    // exec reads can read from 1+ banks at a time
    val ex_rd_req     = io.exec.readA.req
    val rd_row_idx    = ex_rd_req.bits.row_idx    
    val rd_bank_start = ex_rd_req.bits.bank_start 
    val rd_banks      = ex_rd_req.bits.banks      
    val rd_sq_col_idx = ex_rd_req.bits.sq_col_idx 
    val rd_cols       = ex_rd_req.bits.cols       
    assert((bank_start & (bank_start-1.U)) === 0.U, "bank_start not aligned")

    // the queue to maintain back-pressure
    val memq = RegNext(io.exec.readA.req)
    val outq = RegNext(new FgMemUnitReadResp(config))

    // datapath into scratchpads
    val mem_blocked = !outq.enq.ready
    bank_ios.zipWithIndex.foreach { case (bio, i) =>
      bio.read.req.en := !mem_blocked
      bio.read.req.row := row
    }

    // read datapath out of scratchpads: 2-level mux
    val rd_data_shifted = bank_ios.map { bio => 
      bio.read.resp.data >> (sp_col_idx * SQ_COL_IBITS)
    }

    // finish hooking up the read path
    memq.deq.ready     := outq.enq.ready
    outq.enq.valid     := memq.deq.valid
    outq.enq.bits.data := rd_data

    //======================================================================
    // write-datapath
    //======================================================================
    // when 'fire', this MUST NOT BLOCK when writing to the scratchpad!
    // dma-writes only do 1 row max (so it won't write to 2+ banks at once
    val dma_wr_fire   = readerA.io.resp.fire()
    val dma_wr_data   = readerA.io.resp.data
    val dma_wr_lrange = readerA.io.resp.lrange
    val wr_rows       = dma_wr_lrange.rows
    val wr_cols       = dma_wr_lrange.cols
    val wr_sq_col_start = dma_wr_lrange.sq_col_start
    val wr_row_start    = dma_wr_lrange.row_start
    assert(wr_rows === 1.U, "dma cannot write >1 row at a time")

    val wr_real_bank = wr_row_start(15,LOG2_FG_DIM)
    val wr_real_row  = wr_row_start(LOG2_FG_DIM-1,0)
    val mask = ((wr_cols*ITYPE_BITS.U) - 1.U) << (sp_col_idx * SQ_COL_IBITS)
    bank_ios.zipWithIndex.foreach { case (bio, i) =>
      bio.write.req.en           := dma_wr_fire
      bio.write.req.row          := 1.U
      bio.write.req.cols         := wr_cols
      bio.write.req.sq_col_start := wr_sq_col_start
      bio.write.req.data         := dma_wr_data
    }
  }

  {
    // A-banks
    val banks = Seq.fill(FG_NUM) { 
      Module(new FgScratchpadBank(config, FG_NUM, FG_NUM)) 
    }
    val bank_ios = VecInit(banks.map(_.io))

    //======================================================================
    // read-datapath
    //======================================================================
    // exec reads can read from 1+ banks at a time
    val ex_rd_req       = io.exec.readA.req
    val rd_en           = ex_rd_req.en
    val rd_row          = ex_rd_req.row
    val rd_fg_col_start = ex_rd_req.fg_col_start
    val rd_bank_start   = ex_rd_req.bank_start 
    val rd_banks        = ex_rd_req.banks      
    assert((rd_bank_start & (rd_bank_start-1.U)) === 0.U, 
      "bank_start not aligned")

    // the queue to maintain back-pressure
    val memq = RegNext(io.exec.readA.req)
    val outq = RegNext(new FgMemUnitExecReadResp(config, FG_NUM))

    // datapath into scratchpads
    bank_ios.zipWithIndex.foreach { case (bio, i) =>
      bio.read.req.en           := rd_en
      bio.read.req.row          := row
      bio.read.req.fg_col_start := rd_fg_col_start
    }

    // read datapath out of scratchpads and output data (2 cycle latency)
    val rd_datas = bank_ios.map { bio => bio.read.resp.data }
    outq.data = Mux1H((0 to log2Up(FG_NUM)).map { log2banks => 
      val my_banks = scala.math.pow(2, log2banks)
      val is_selected = (my_banks === banks)
      (is_selected -> Mux1H((0 to (FG_NUM-1) by my_banks).map{my_bank_start =>
        val my_bank_end = my_bank_start + my_banks - 1
        val is_selected = (my_bank_start === bank_start)
        (is_selected -> Cat((my_bank_start to my_bank_end).map{bank_idx =>
          rd_datas(bank_idx)
        }))
      }))
    })

    //======================================================================
    // write-datapath
    //======================================================================
    // when 'fire', this MUST NOT BLOCK when writing to the scratchpad!
    // dma-writes only do 1 row max (so it won't write to 2+ banks at once
    val dma_wr_fire     = readerA.io.resp.fire()
    val dma_wr_data     = readerA.io.resp.data
    val dma_wr_lrange   = readerA.io.resp.lrange
    val wr_rows         = dma_wr_lrange.rows
    val wr_cols         = dma_wr_lrange.cols
    val wr_fg_col_start = dma_wr_lrange.fg_col_start
    val wr_row_start    = dma_wr_lrange.row_start
    assert(wr_rows === 1.U, "dma cannot write >1 row at a time")

    val wr_real_bank = wr_row_start(15,LOG2_FG_DIM)
    val wr_real_row  = wr_row_start(LOG2_FG_DIM-1,0)
    assert(wr_real_bank < 2.U)

    bank_ios.zipWithIndex.foreach { case (bio, i) =>
      bio.write.req.en           := (dma_wr_fire && (wr_real_bank === i))
      bio.write.req.row          := wr_real_row
      bio.write.req.cols         := wr_cols
      bio.write.req.fg_col_start := wr_fg_col_start
      bio.write.req.data         := dma_wr_data
    }
  }

  {
    // B-banks
    val banks = Seq.fill(2) { 
      Module(new FgScratchpadBank(config, SQRT_FG_NUM, 1)) 
    }
    val bank_ios = VecInit(banks.map(_.io))

    //======================================================================
    // read-datapath
    //======================================================================
    // exec reads can read from 1+ banks at a time
    val ex_rd_req       = io.exec.readB.req
    val rd_en           = ex_rd_req.en
    val rd_row          = ex_rd_req.row
    val rd_fg_col_start = ex_rd_req.fg_col_start
    val rd_bank_start   = ex_rd_req.bank_start 
    val rd_banks        = ex_rd_req.banks      
    assert(rd_banks === 1.U && rd_bank_start < 2.U)

    // the queue to maintain back-pressure
    val memq = RegNext(io.exec.readB.req)
    val outq = RegNext(new FgMemUnitExecReadResp(config, 1))

    // datapath into scratchpads
    bank_ios.zipWithIndex.foreach { case (bio, i) =>
      bio.read.req.en           := rd_en
      bio.read.req.row          := row
      bio.read.req.fg_col_start := rd_fg_col_start
    }

    // read datapath out of scratchpads and output data (2 cycle latency)
    val rd_datas = bank_ios.map { bio => bio.read.resp.data }
    outq.data = rd_datas(memq.bank_start)

    //======================================================================
    // write-datapath
    //======================================================================
    // when 'fire', this MUST NOT BLOCK when writing to the scratchpad!
    // dma-writes only do 1 row max (so it won't write to 2+ banks at once
    val dma_wr_fire     = readerB.io.resp.fire()
    val dma_wr_data     = readerB.io.resp.data
    val dma_wr_lrange   = readerB.io.resp.lrange
    val wr_rows         = dma_wr_lrange.rows
    val wr_cols         = dma_wr_lrange.cols
    val wr_fg_col_start = dma_wr_lrange.fg_col_start
    val wr_row_start    = dma_wr_lrange.row_start
    assert(wr_rows === 1.U, "dma cannot write >1 row at a time")

    val wr_real_bank = wr_row_start(15,LOG2_FG_DIM)
    val wr_real_row  = wr_row_start(LOG2_FG_DIM-1,0)
    assert(wr_real_bank < 2.U)

    bank_ios.zipWithIndex.foreach { case (bio, i) =>
      bio.write.req.en           := (dma_wr_fire && (wr_real_bank === i))
      bio.write.req.row          := wr_real_row
      bio.write.req.cols         := wr_cols
      bio.write.req.fg_col_start := wr_fg_col_start
      bio.write.req.data         := dma_wr_data
    }
  }

  {
    // C/D-banks
    val banks = Seq.fill(FG_DIM) { Module( new FgScratchpadBank(config)) }
    val bank_ios = VecInit(banks.map(_.io))

    //======================================================================
    // read-datapath
    //======================================================================
    // exec reads can read from 1+ banks at a time
    val ex_rd_req     = io.exec.readA.req
    val rd_row_idx    = ex_rd_req.bits.row_idx    
    val rd_bank_start = ex_rd_req.bits.bank_start 
    val rd_banks      = ex_rd_req.bits.banks      
    val rd_sq_col_idx = ex_rd_req.bits.sq_col_idx 
    val rd_cols       = ex_rd_req.bits.cols       
    assert((bank_start & (bank_start-1.U)) === 0.U, "bank_start not aligned")

    // the queue to maintain back-pressure
    val memq = RegNext(io.exec.readA.req)
    val outq = RegNext(new FgMemUnitReadResp(config))

    // datapath into scratchpads
    val mem_blocked = !outq.enq.ready
    bank_ios.zipWithIndex.foreach { case (bio, i) =>
      bio.read.req.en := !mem_blocked
      bio.read.req.row := row
    }

    // read datapath out of scratchpads: 2-level mux
    val rd_data_shifted = bank_ios.map { bio => 
      bio.read.resp.data >> (sp_col_idx * SQ_COL_IBITS)
    }
    val rd_data = Mux1H((0 to log2Up(FG_NUM)).map { log2banks => 
      val my_banks = scala.math.pow(2, log2banks)
      val is_selected = (my_banks === banks)
      (is_selected -> {
        val bit_per_bank = ITYPE_BITS * FG_DIM * (FG_NUM >> log2banks)
        Mux1H((0 to (FG_NUM-1) by my_banks).map { my_bank_start =>
          val my_bank_end = my_bank_start + my_banks - 1
          val is_selected = (my_bank_start === bank_start)
          (is_selected -> Cat((my_bank_start to my_bank_end).map {bank_idx =>
            rd_data_shifted(bank_idx)((bits_per_bank-1).U, 0.U)
          }))
        })
      })
    })

    // finish hooking up the read path
    memq.deq.ready     := outq.enq.ready
    outq.enq.valid     := memq.deq.valid
    outq.enq.bits.data := rd_data

    //======================================================================
    // write-datapath
    //======================================================================
    // when 'fire', this MUST NOT BLOCK when writing to the scratchpad!
    // dma-writes only do 1 row max (so it won't write to 2+ banks at once
    val dma_wr_fire   = readerA.io.resp.fire()
    val dma_wr_data   = readerA.io.resp.data
    val dma_wr_lrange = readerA.io.resp.lrange
    val wr_rows       = dma_wr_lrange.rows
    val wr_cols       = dma_wr_lrange.cols
    val wr_sq_col_start = dma_wr_lrange.sq_col_start
    val wr_row_start    = dma_wr_lrange.row_start
    assert(wr_rows === 1.U, "dma cannot write >1 row at a time")

    val wr_real_bank = wr_row_start(15,LOG2_FG_DIM)
    val wr_real_row  = wr_row_start(LOG2_FG_DIM-1,0)
    val mask = ((wr_cols*ITYPE_BITS.U) - 1.U) << (sp_col_idx * SQ_COL_IBITS)
    bank_ios.zipWithIndex.foreach { case (bio, i) =>
      bio.write.req.en           := dma_wr_fire
      bio.write.req.row          := 1.U
      bio.write.req.cols         := wr_cols
      bio.write.req.sq_col_start := wr_sq_col_start
      bio.write.req.data         := dma_wr_data
    }
  }
  //=======================================================================
  // B-tile scratchpad banks
  // - NOTE: byte-aligned, so inputType % 8 must == 0 for exec-controller
  //   writes to work!
  //=======================================================================
  {
    // when 'fire', this MUST NOT BLOCK when writing to the scratchpad!
    // dma-writes only do 1 row max (so it won't write to 2+ banks at once
    val dma_wr_fire   = readerB.io.resp.fire()
    val dma_wr_data   = readerB.io.resp.data
    val dma_wr_lrange = readerB.io.resp.lrange
    assert(dma_wr_lrange.bank < 2, "dma writing to bank >= 2")

    // exec reads can read from 1+ banks at a time
    val ex_rd_req  = io.exec.readB.req
    val row_idx    = ex_rd_req.bits.row_idx    
    val bank_start = ex_rd_req.bits.bank_start 
    val banks      = ex_rd_req.bits.banks      
    val sq_col_idx = ex_rd_req.bits.sq_col_idx 
    val cols       = ex_rd_req.bits.cols       
    assert(banks === 1, "can't read more than 1 B-bank at a time")

    val banks = Seq.fill(2) { Module( new FgScratchpadBank(config) }
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

      bio.read.req.valid := exread
      bio.read.resp.ready := ex_read_resp.ready
      ex_read_req.ready := bio.read.req.ready

      // The ExecuteController gets priority when reading from SRAMs
      bio.read.req.bits.addr := ex_read_req.bits.addr

      val ex_read_resp = io.srams.read(i).resp


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

    val banks = Seq.fill(FG_NUM) { Module(new AccumulatorMem(config)) }
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
