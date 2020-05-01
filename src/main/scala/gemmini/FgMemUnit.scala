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
//class FgMemUnitDMAReadReq[T <: Data](config: FgGemminiArrayConfig[T])
//  (implicit p: Parameters) extends CoreBundle {
//  import config._
//  val bank         = UInt(LOG2_FG_NUM.W))
//  val row          = UInt(LOG2_FG_DIM.W))
//  val fg_col_start = UInt(LOG2_FG_NUM.W))
//  val cols         = UInt(LOG2_SP_ROW_ELEMS.W))
//}

class FgMemUnitMemOpReq[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val vaddr  = UInt(coreMaxAddrBits.W)
  val lrange = new FgLocalRange(config)
  val status = new MStatus
  val rob_id = UInt(LOG2_ROB_ENTRIES.W)
}

class FgMemUnitMemOpResp[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

class FgMemUnitMemOpIO[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val req  = Decoupled(new FgMemUnitMemOpReq(config))
  val resp = Flipped(Decoupled(new FgMemUnitMemOpResp(config)))
}

//===========================================================================
// ExecController <-> MemUnit
//===========================================================================
class FgMemUnitExecReadReq[T <: Data]
  (config: FgGemminiArrayConfig[T], fg_cols: Int)
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en           = Output(Bool())
  val row          = Output(UInt(LOG2_FG_DIM.W))
  val fg_col_start = Output(UInt(log2Up(fg_cols).W))
  val bank_start   = Output(UInt(LOG2_FG_NUM.W))
  val banks        = Output(UInt(LOG2_FG_NUM.W))
}

class FgMemUnitExecReadResp[T <: Data]
  (config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val data = Output(UInt(FG_DIM * FG_NUM * ITYPE_BITS.W))
}

class FgMemUnitExecReadIO[T <: Data](config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  val req  = new FgMemUnitExecReadReq(config)
  val resp = Flipped(new FgMemUnitExecReadResp(config))
}

class FgMemUnitExecWriteReq[T <: Data](config: FgGemminiArrayConfig[T]) 
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val en           = Output(Bool())
  val row          = Output(UInt(FG_DIM_IDX.W))
  val cols         = Output(UInt(ACC_ROW_ELEMS_CTR.W))
  val fg_col_start = Output(UInt(FG_NUM_IDX.W))
  val bank_start   = Output(UInt(FG_NUM_IDX.W))
  val banks        = Output(UInt(FG_NUM_CTR.W))
  val accum        = Output(Bool())
  val data         = Output(UInt(ACC_ROW_BITS.W))
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
      val loadA  = Flipped(new FgMemUnitMemOpIO(config))
      val loadB  = Flipped(new FgMemUnitMemOpIO(config))
      val loadD  = Flipped(new FgMemUnitMemOpIO(config))
      val storeC = Flipped(new FgMemUnitMemOpIO(config))
    }
    val tlb = new Bundle {
      val loadA  = new FrontendTLBIO
      val loadB  = new FrontendTLBIO
      val loadD  = new FrontendTLBIO
      val storeC = new FrontendTLBIO
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
  //writerC.io.req <> io.dma.writeC.req

  Seq(readerA.io.resp, readerB.io.resp, readerD.io.resp).zip(
    Seq(io.dma.readA, io.dma.readB, io.dma.readD).foreach {case(pin, port) =>
      pin.ready := port.ready
      port.valid := port.valid
      port.bits.rob_id := pin.bits.rob_id
    })

  io.tlb.readA  <> readerA.io.tlb
  io.tlb.readB  <> readerB.io.tlb
  io.tlb.readD  <> readerD.io.tlb
  io.tlb.writeC <> writerC.io.tlb

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
    val banks = Seq.fill(FG_NUM) { 
      Module(new FgAccumulatorBank(config, FG_NUM, FG_NUM)) 
    }
    val bank_ios = VecInit(banks.map(_.io))
 
    //======================================================================
    // write-datapath
    //======================================================================
    val ex_wr_req          = io.exec.writeC.req
    val ex_wr_en           = ex_wr_req.en
    val ex_wr_row          = ex_wr_req.row
    val ex_wr_cols         = ex_wr_req.cols
    val ex_wr_fg_col_start = ex_wr_req.fg_col_start
    val ex_wr_bank_start   = ex_wr_req.bank_start
    val ex_wr_bank_end     = ex_wr_req.bank_end
    val ex_wr_accum        = ex_wr_req.accum
    val ex_wr_data         = ex_wr_req.data
    assert((bank_start & (bank_start-1.U)) === 0.U, "bank_start not aligned")

    // when 'fire', this MUST NOT BLOCK when writing to the scratchpad!
    // dma-writes only do 1 row max (so it won't write to 2+ banks at once
    readerD.io.req.ready := !ex_wr_en
    val dma_wr_fire         = readerD.io.resp.fire()
    val dma_wr_data         = readerD.io.resp.data
    val dma_wr_lrange       = readerD.io.resp.lrange
    val dma_wr_row          = dma_wr_lrange.row
    val dma_wr_cols         = dma_wr_lrange.cols
    val dma_wr_fg_col_start = dma_wr_lrange.fg_col_start
    val dma_wr_bank_start   = dma_wr_lrange.bank_start
    val dma_wr_banks        = dma_wr_lrange.total_banks
    val dma_wr_accum        = false.B
    assert(dma_wr_rows === 1.U, "dma cannot write >1 row at a time")
    assert(dma_wr_banks === 1.U, "dma cannot write >1 row at a time")

    val is_writing = ex_wr_en || dma_wr_fire
    bank_ios.zipWithIndex.foreach { case (bio, i) =>
      when (ex_wr_en) {
        val is_active     = (ex_wr_bank_start <= i) && (i <= ex_wr_bank_end)
        val bank_offset   = i.U - ex_wr_bank_start
        val fg_per_bank   = (FG_NUM / ex_wr_banks)
        val bits_per_bank = fg_per_bank * FG_DIM * OTYPE_BITS
        val shifted_data  = ex_wr_data >> (i.U * bits_per_bank)

        bio.write.req.en           := is_active
        bio.write.req.row          := ex_wr_row
        bio.write.req.cols         := ex_wr_cols
        bio.write.req.fg_col_start := ex_wr_fg_col_start
        bio.write.req.data         := shifted_data
        bio.write.req.accum        := ex_wr_accum
      } 
      .elsewhen (dma_wr_fire) {
        bio.write.req.en           := (dma_wr_bank_start === i)
        bio.write.req.row          := dma_wr_row
        bio.write.req.cols         := dma_wr_cols
        bio.write.req.fg_col_start := dma_wr_fg_col_start
        bio.write.req.data         := dma_wr_data
        bio.write.req.accum        := dma_wr_accum
      }
    }
 
    //======================================================================
    // read-datapath
    //======================================================================
    val dma_rd_req          = io.dma.writeC.req
    val dma_rd_vaddr        = dma_rd_req.bits.vaddr
    val dma_rd_lrange       = dma_rd_req.bits.lrange
    val dma_rd_status       = dma_rd_req.bits.status
    val dma_rd_row          = dma_rd_lrange.row
    val dma_rd_cols         = dma_rd_lrange.cols
    val dma_rd_fg_col_start = dma_rd_lrange.fg_col_start
    val dma_rd_bank_start   = dma_rd_lrange.bank_start 
    val dma_rd_banks        = dma_rd_lrange.banks      
    assert(dma_rd_rows === 1.U, "dma cannot read >1 acc rows at a time")
    assert((dma_rd_banks === 1.U,"dma cannot read >1 acc banks at a time") 

    io.dma.writeC.req.ready := !bio.write.req.en(dma_rd_bank_start)
    val dma_rd_fire = io.dma.writeC.fire()

    // accept read req if we know a spot is reserved for it 2 cycles later

    // acc bank read behavior:
    // - fg_col_start must be self-aligned
    // - total fg_cols per row written === FG_NUM / total_banks
    // - dma_store only reads 1 row of 1 bank at a time
    // - dma_load only writes 1 row of 1 bank at a time
    // - ex_write writes 1 row of multiple consecutive banks at a time
    // - simulatenously read banks are concatenated
    // datapath into scratchpads
    bank_ios.zipWithIndex.foreach { case (bio, i) =>
      bio.read.req.en           := dma_rd_fire && (dma_rd_bank_start === i.U)
      bio.read.req.row          := dma_rd_row
      bio.read.req.fg_col_start := dma_rd_fg_col_start
    }
    val dma_rd_en_buf     = ShiftRegister(dma_rd_fire,       2)
    val dma_rd_cols_buf   = ShiftRegister(dma_rd_cols,       2)
    val dma_rd_bank_buf   = ShiftRegister(dma_rd_bank_start, 2)
    val dma_rd_vaddr_buf  = ShiftRegister(dma_rd_vaddr,      2)
    val dma_rd_status_buf = ShiftRegister(dma_rd_status,     2)
    val dma_rd_rob_id_buf = ShiftRegister(dma_rd_rob_id,     2)

    // read datapath out of scratchpads and output data (2 cycle latency)
    val dma_rd_datas = bank_ios.map { bio => bio.read.resp.data }
    val dma_rd_data  = rd_datas(dma_rd_bank_buf)

    val dma_rd_q_type = new FgDMAWriteRequest(config, ACC_ROW_BYTES)
    val dma_rd_q = Module(new Queue(dma_rd_q_type))

    // queue accumulator read if can't send to dma-store unit this cycle
    when (dma_rd_en_buf && (!writerC.io.req.ready || dma_rd_q.valid)) {
      dma_rd_q.enq.valid       := true.B
      dma_rd_q.enq.bits.data   := dma_rd_data
      dma_rd_q.enq.bits.vaddr  := dma_rd_vaddr_buf
      dma_rd_q.enq.bits.len    := (dma_rd_cols_buf * ITYPE_BYTES.U)
      dma_rd_q.enq.bits.status := dma_rd_status_buf
      dma_rd_q.enq.bits.rob_id := dma_rd_rob_id_buf
      assert(dma_rd_q.enq.fire())
    }

    // send the accumulator read data or queued data to dma-store if ready
    when (dma_rd_q.deq.valid) {
      writerC.io.req <> dma_rd_q.deq
    } 
    .elsewhen (dma_rd_en_buf && writerC.io.req.ready) {
      writerC.io.req.valid       := true.B
      writerC.io.req.bits.data   := dma_rd_data.asUInt()
      writerC.io.req.bits.vaddr  := dma_rd_vaddr_buf
      writerC.io.req.bits.len    := (dma_rd_cols_buf * ITYPE_BYTES.U)
      writerC.io.req.bits.status := dma_rd_status_buf
      writerC.io.req.bits.rob_id := dma_rd_rob_id_buf
    }

    // internal dma-writer decoupled response to external store-controller
    writerC.io.resp.ready     := io.dma.writeC.resp.ready
    io.dma.writeC.resp.valid  := writerC.io.resp.valid
    io.dma.writeC.resp.rob_id := writerC.io.resp.rob_id
  }
}
