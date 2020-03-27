package gemmini

import chisel3._
import chisel3.util._

import freechips.rocketchip.tile.RoCCCommand

import GemminiISA._
import Util._

import midas.targetutils


// TODO unify this class with GemminiCmdWithDeps
class ROBIssue[T <: Data](cmd_t: T, nEntries: Int) extends Bundle {
  val valid = Output(Bool())
  val ready = Input(Bool())
  val cmd = Output(cmd_t.cloneType)
  val rob_id = Output(UInt(log2Up(nEntries).W))

  def fire(dummy: Int=0) = valid && ready

  override def cloneType: this.type = new ROBIssue(cmd_t, nEntries).asInstanceOf[this.type]
}

// class ROB[T <: RoCCCommand](cmd_t: T, nEntries: Int, sprows: Int, block_rows: Int) extends Module {
class ROB(cmd_t: RoCCCommand, nEntries: Int, local_addr_t: LocalAddr, block_rows: Int) extends Module {
  val io = IO(new Bundle {
    val alloc = Flipped(Decoupled(cmd_t.cloneType))

    val completed = Flipped(Valid(UInt(log2Up(nEntries).W)))

    val issue = new Bundle {
      val ld = new ROBIssue(cmd_t, nEntries)
      val st = new ROBIssue(cmd_t, nEntries)
      val ex = new ROBIssue(cmd_t, nEntries)
    }

    val busy = Output(Bool())
  })

  val ldq :: stq :: exq :: Nil = Enum(3)
  val q_t = ldq.cloneType

  val MAX_CMD_ID   = 1000000            // FOR DEBUGGING ONLY
  val debug_cycle  = RegInit(0.U(32.W)) // FOR DEBUGGING ONLY

  debug_cycle := debug_cycle + 1.U      // FOR DEBUGGING ONLY
  val cmd_id = Counter(MAX_CMD_ID)      // FOR DEBUGGING ONLY

  // scratchpad range, used for tracking RAW, WAR, and WAW deps
  class SPRange extends Bundle {
    val valid = Bool()
    val is_sp = Bool()
    val start = UInt(30.W) // TODO magic number
    val end   = UInt(30.W) // TODO magic number
    def overlaps(other: SPRange) = valid && other.valid && 
                                   (is_sp === other.is_sp) &&
                                   (start < other.end) && 
                                   (end > other.start)
  }

  class Entry extends Bundle {
    val q = q_t.cloneType

    val is_config = Bool()

    val cmd_id   = UInt(log2Up(MAX_CMD_ID).W)   // FOR DEBUGGING ONLY
    val is_load  = Bool()    // true on config_load.  FOR DEBUGGING ONLY
    val is_store = Bool()    // true on config_store. FOR DEBUGGING ONLY
    val is_ex    = Bool()    // true on config_ex.    FOR DEBUGGING ONLY

    val op1 = new SPRange()
    val op2 = new SPRange()
    val op3 = new SPRange()
    val dst = new SPRange()

    val issued = Bool()

    val complete_on_issue = Bool()

    val cmd = cmd_t.cloneType

    val deps = Vec(nEntries, Bool())
    def ready(dummy: Int = 0): Bool = !deps.reduce(_ || _)
  }

  val entries = Reg(Vec(nEntries, UDValid(new Entry)))

  val empty = !entries.map(_.valid).reduce(_ || _)
  val full = entries.map(_.valid).reduce(_ && _)

  io.busy := !empty

  // Read in commands to the buffer
  io.alloc.ready := !full

  val last_allocated = Reg(UInt(log2Up(nEntries).W))

  val new_entry = Wire(new Entry)
  new_entry := DontCare
  val new_entry_id = MuxCase((nEntries-1).U, entries.zipWithIndex.map { 
                                        case (e, i) => !e.valid -> i.U })
  val alloc_fire = io.alloc.fire()

  when (io.alloc.fire()) {
    val spAddrBits = 32
    val cmd = io.alloc.bits
    val funct = cmd.inst.funct
    val funct_is_compute = funct === COMPUTE_AND_STAY_CMD || funct === COMPUTE_AND_FLIP_CMD
    val funct_is_compute_preload = funct === COMPUTE_AND_FLIP_CMD
    val config_cmd_type = cmd.rs1(1,0) // TODO magic numbers

    new_entry.issued := false.B
    new_entry.cmd := cmd

    new_entry.is_config := funct === CONFIG_CMD

    new_entry.op1.valid := funct_is_compute
    new_entry.op1.is_sp := cmd.rs1(31)
    new_entry.op1.start := cmd.rs1(29,0)
    new_entry.op1.end   := cmd.rs1(29,0) + block_rows.U

    new_entry.op2.valid := funct_is_compute || funct === STORE_CMD
    new_entry.op2.is_sp := cmd.rs2(31)
    new_entry.op2.start := cmd.rs2(29,0)
    new_entry.op2.end   := cmd.rs2(29,0) + block_rows.U

    new_entry.op3.valid := funct_is_compute
    new_entry.op3.is_sp := cmd.rs1(63)
    new_entry.op3.start := cmd.rs1(61,32)
    new_entry.op3.end   := cmd.rs1(61,32) + block_rows.U

    new_entry.dst.valid := funct_is_compute || funct === LOAD_CMD
    new_entry.dst.is_sp := Mux(funct_is_compute, cmd.rs2(63),    cmd.rs2(31))
    new_entry.dst.start := Mux(funct_is_compute, cmd.rs2(61,32), cmd.rs2(29,0))
    new_entry.dst.end   := new_entry.dst.start + 
                           (Mux(funct_is_compute, 1.U, cmd.rs2(63, 32)) * block_rows.U)

    val is_load  = (funct === LOAD_CMD) || 
                   (funct === CONFIG_CMD && config_cmd_type === CONFIG_LOAD)
    val is_store = (funct === STORE_CMD) || 
                   (funct === CONFIG_CMD && config_cmd_type === CONFIG_STORE)
    val is_ex    = funct_is_compute || 
                   (funct === CONFIG_CMD && config_cmd_type === CONFIG_EX)

    // never allow this to wrap.FOR DEBUGGING ONLY
    assert(!cmd_id.inc())
    new_entry.cmd_id   := cmd_id.value // FOR DEBUGGING ONLY
    new_entry.is_load  := is_load      // FOR DEBUGGING ONLY
    new_entry.is_store := is_store     // FOR DEBUGGING ONLY
    new_entry.is_ex    := is_ex        // FOR DEBUGGING ONLY
    //======================================================================
    // debug
    //======================================================================
    //printf(midas.targetutils.SynthesizePrintf("ISSUE config_mvin: stride=%x\n", 
    //  new_entry.bits.cmd.rs2))
    when(new_entry.is_config) {
      when (new_entry.is_load) {
        printf(
          "cycle[%d], entry[%d], accept[%d], config_mvin[stride=%x]\n", 
          debug_cycle, new_entry_id, cmd_id.value, 
          new_entry.cmd.rs2)
      }
      .elsewhen (new_entry.is_store) {
        printf(
          "cycle[%d], entry[%d], accept[%d], config_mvout[stride=%x]\n", 
          debug_cycle, new_entry_id, cmd_id.value, 
          new_entry.cmd.rs2)
      }
      .otherwise {
        assert(new_entry.is_ex)
        printf(
          "cycle[%d], entry[%d], accept[%d], config_ex[matmul_rshift=%x, acc_rshift=%x, relu6_lshift=%x]\n", 
          debug_cycle, new_entry_id, cmd_id.value, 
          cmd.rs1(63,32), cmd.rs2(31,0), cmd.rs2(63,32))
      }
    }
    .elsewhen (new_entry.is_load) {
      printf(
        "cycle[%d], entry[%d], accept[%d], mvin[dram=%x, spad=%x, tiles=%x]\n",
        debug_cycle, new_entry_id, cmd_id.value, 
        cmd.rs1, cmd.rs2(31,0), cmd.rs2(63,32))
    }
    .elsewhen (new_entry.is_store) {
      printf(
        "cycle[%d], entry[%d], accept[%d], mvout[dram=%x, spad=%x, tiles=%x]\n",
        debug_cycle, new_entry_id, cmd_id.value, 
        cmd.rs1, cmd.rs2(31,0), cmd.rs2(63,32))
    }
    .otherwise {
      assert(new_entry.is_ex)
      when (funct_is_compute_preload) {
        printf(
          "cycle[%d], entry[%d], accept[%d], ex.pre[A=%x, B=%x, D=%x, C=%x]\n",
          debug_cycle, new_entry_id, cmd_id.value, 
          cmd.rs1(31,0), cmd.rs1(63,32), cmd.rs2(31,0), cmd.rs2(63,32))
      }
      .otherwise {
        printf(
          "cycle[%d], entry[%d], accept[%d], ex.acc[A=%x, B=%x, D=%x, C=%x]\n",
          debug_cycle, new_entry_id, cmd_id.value, 
          cmd.rs1(31,0), cmd.rs1(63,32), cmd.rs2(31,0), cmd.rs2(63,32))
      }
    }

    //======================================================================

    new_entry.q := Mux1H(Seq(
      is_load -> ldq,
      is_store -> stq,
      is_ex -> exq
    ))

    // We search for all entries which write to an address which we read from
    val raws = entries.map { e => e.valid && (
      e.bits.dst.overlaps(new_entry.op1) ||
      e.bits.dst.overlaps(new_entry.op2) ||
      e.bits.dst.overlaps(new_entry.op3)
    )}

    // We search for all entries which read from an address that we write to
    val wars = entries.map { e => e.valid && (
      new_entry.dst.overlaps(e.bits.op1) ||
      new_entry.dst.overlaps(e.bits.op2) ||
      new_entry.dst.overlaps(e.bits.op3)
    )}

    // We search for all entries which write to an address that we write to
    val waws = entries.map { e => e.valid && 
      new_entry.dst.overlaps(e.bits.dst)
    }

    val older_in_same_q = entries.map { e => e.valid && 
      e.bits.q === new_entry.q && 
      !e.bits.issued
    }

    val is_st_and_must_wait_for_prior_ex_config = entries.map { e =>
      (e.valid && e.bits.q === exq && e.bits.is_config) &&
      (new_entry.q === stq && !new_entry.is_config)
    }

    val is_ex_config_and_must_wait_for_prior_st = entries.map { e =>
      (e.valid && e.bits.q === stq && !e.bits.is_config) &&
      (new_entry.q === exq && new_entry.is_config)
    }

    new_entry.deps := (Cat(raws) | 
                       Cat(wars) | 
                       Cat(waws) | 
                       Cat(older_in_same_q) |
                       Cat(is_st_and_must_wait_for_prior_ex_config) | 
                       Cat(is_ex_config_and_must_wait_for_prior_st)
                      ).asBools().reverse

    new_entry.complete_on_issue := new_entry.is_config && new_entry.q =/= exq

    entries(new_entry_id).valid := true.B
    entries(new_entry_id).bits := new_entry

    last_allocated := new_entry_id
  }

  // Issue commands which are ready to be issued
  Seq((ldq, io.issue.ld), (stq, io.issue.st), (exq, io.issue.ex)).foreach { case (q, io) =>
    val issue_id = MuxCase((nEntries-1).U, entries.zipWithIndex.map { case (e, i) =>
      (e.valid && e.bits.ready() && !e.bits.issued && e.bits.q === q) -> i.U
    })

    io.valid := entries.map(e => e.valid && e.bits.ready() && 
                                !e.bits.issued && e.bits.q === q).reduce(_ || _)
    io.cmd := entries(issue_id).bits.cmd
    io.rob_id := issue_id

    // ssteff: added for debug
    when(io.fire()) {
      //======================================================================
      // debug
      //======================================================================
      when(entries(issue_id).bits.is_config) {
        when (entries(issue_id).bits.is_load) {
          printf(
            "cycle[%d], entry[%d],  issue[%d], config_mvin\n",
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
          printf(
            "cycle[%d], entry[%d],  final[%d], config_mvin\n", 
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
        }
        .elsewhen (entries(issue_id).bits.is_store) {
          printf(
            "cycle[%d], entry[%d],  issue[%d], config_mvout\n",
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
          printf(
            "cycle[%d], entry[%d],  final[%d], config_mvout\n", 
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
        }
        .otherwise {
          assert(entries(issue_id).bits.is_ex)
          printf(
            "cycle[%d], entry[%d],  issue[%d], config_ex\n",
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
        }
      }
      .elsewhen (entries(issue_id).bits.is_load) {
        printf(
          "cycle[%d], entry[%d],  issue[%d], mvin\n",
          debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
      }
      .elsewhen (entries(issue_id).bits.is_store) {
        printf(
          "cycle[%d], entry[%d],  issue[%d], mvout\n",
          debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
      }
      .otherwise {
        assert(entries(issue_id).bits.is_ex)
        printf(
          "cycle[%d], entry[%d],  issue[%d], ex\n",
          debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
      }
      //======================================================================

      entries(issue_id).bits.issued := true.B

      // Clear out all the dependency bits for instructions which depend on the same queue
      entries.zipWithIndex.foreach { case (e, i) =>
        val is_same_q = Mux(alloc_fire && new_entry_id === i.U,
          new_entry.q === entries(issue_id).bits.q,
          e.bits.q === entries(issue_id).bits.q)

        when (is_same_q || entries(issue_id).bits.complete_on_issue) {
          e.bits.deps(issue_id) := false.B
        }
      }

      entries(issue_id).valid := !entries(issue_id).bits.complete_on_issue
    }
  }

  // Mark entries as completed once they've returned
  when (io.completed.fire()) {
    //======================================================================
    // debug
    //======================================================================
    //printf(midas.targetutils.SynthesizePrintf("ISSUE config_mvin: stride=%x\n", 
    //  entries(io.completed.bits).bits.cmd.rs2))
    when (entries(io.completed.bits).bits.is_config) {
      assert(entries(io.completed.bits).bits.is_ex)
      printf(
        "cycle[%d], entry[%d],  final[%d], config_ex\n",
        debug_cycle, io.completed.bits, entries(io.completed.bits).bits.cmd_id)
    }
    .elsewhen (entries(io.completed.bits).bits.is_load) {
      printf(
        "cycle[%d], entry[%d],  final[%d], mvin\n",
        debug_cycle, io.completed.bits, entries(io.completed.bits).bits.cmd_id)
    }
    .elsewhen (entries(io.completed.bits).bits.is_store) {
      printf(
        "cycle[%d], entry[%d],  final[%d], mvout\n",
        debug_cycle, io.completed.bits, entries(io.completed.bits).bits.cmd_id)
    }
    .otherwise {
      assert(entries(io.completed.bits).bits.is_ex)
      printf(
        "cycle[%d], entry[%d],  final[%d], ex\n",
        debug_cycle, io.completed.bits, entries(io.completed.bits).bits.cmd_id)
    }
    //======================================================================

    entries.foreach(_.bits.deps(io.completed.bits) := false.B)

    entries(io.completed.bits).valid := false.B
    assert(entries(io.completed.bits).valid)
  }

  val utilization = PopCount(entries.map(e => e.valid))
  val utilization_ld_q_unissued = PopCount(entries.map(e => e.valid && 
                                                            !e.bits.issued && 
                                                            e.bits.q === ldq))
  val utilization_st_q_unissued = PopCount(entries.map(e => e.valid && 
                                                            !e.bits.issued && 
                                                            e.bits.q === stq))
  val utilization_ex_q_unissued = PopCount(entries.map(e => e.valid && 
                                                            !e.bits.issued && 
                                                            e.bits.q === exq))
  val utilization_ld_q = PopCount(entries.map(e => e.valid && e.bits.q === ldq))
  val utilization_st_q = PopCount(entries.map(e => e.valid && e.bits.q === stq))
  val utilization_ex_q = PopCount(entries.map(e => e.valid && e.bits.q === exq))

  val packed_deps = VecInit(entries.map(e => Cat(e.bits.deps)))
  dontTouch(packed_deps)

  val pop_count_packed_deps = VecInit(entries.map(e => Mux(e.valid, PopCount(e.bits.deps), 0.U)))
  val min_pop_count = pop_count_packed_deps.reduce((acc, d) => minOf(acc, d))
  // assert(min_pop_count < 2.U)
  dontTouch(pop_count_packed_deps)
  dontTouch(min_pop_count)

  val cycles_since_issue = RegInit(0.U(32.W))

  when (io.issue.ld.fire() || io.issue.st.fire() || io.issue.ex.fire() || !io.busy) {
    cycles_since_issue := 0.U
  }.elsewhen(io.busy) {
    cycles_since_issue := cycles_since_issue + 1.U
  }
  assert(cycles_since_issue < 10000.U, "pipeline stall")

  val cntr = Counter(10000000)
  when (cntr.inc()) {
    printf(p"Utilization: $utilization\n")
    printf(p"Utilization ld q (incomplete): $utilization_ld_q_unissued\n")
    printf(p"Utilization st q (incomplete): $utilization_st_q_unissued\n")
    printf(p"Utilization ex q (incomplete): $utilization_ex_q_unissued\n")
    printf(p"Utilization ld q: $utilization_ld_q\n")
    printf(p"Utilization st q: $utilization_st_q\n")
    printf(p"Utilization ex q: $utilization_ex_q\n")
    printf(p"Packed deps: $packed_deps\n")
    printf(p"Last allocated: $last_allocated\n\n")
  }

  when (reset.toBool()) {
    entries.foreach(_.valid := false.B)
  }
}
