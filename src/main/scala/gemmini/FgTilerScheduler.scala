//===========================================================================
// TilerController's Internal Scheduler implementation
//===========================================================================
package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import GemminiISA._
import Util._

class TilerScheduler[T <: Data: Arithmetic]
  (config: GemminiArrayConfig[T])(implicit val p: Parameters) 
  extends Module with HasCoreParameters {
  import config._

  //=========================================================================
  // interface
  //=========================================================================
  val io = IO(new Bundle {
    val cmd_in = Flipped(Decoupled(new RoCCCommand))
    val issue = new Bundle {
      val loadA  = Decoupled(new GemminiCmd(ROB_ENTRIES))
      val loadB  = Decoupled(new GemminiCmd(ROB_ENTRIES))
      val loadD  = Decoupled(new GemminiCmd(ROB_ENTRIES))
      val storeC = Decoupled(new GemminiCmd(ROB_ENTRIES))
      val exec   = Decoupled(new GemminiCmd(ROB_ENTRIES))
    }
    val completed = Flipped(Decoupled(UInt(LOG2_ROB_ENTRIES.W)))
    val busy = Output(Bool())
  })
  io.completed.ready := true.B

  //=========================================================================
  // ...
  //=========================================================================
  val (ldaq :: ldbq :: lddq :: stcq :: exq :: Nil) = Enum(5)
  val q_t = ldaq.cloneType

  // FOR DEBUGGING ONLY
  val MAX_DEBUG_ID = 1000000            
  val cmd_id = Counter(MAX_DEBUG_ID)
  val debug_cycle = RegInit(0.U(32.W))
  debug_cycle := debug_cycle + 1.U
  //---------------------

  class Entry extends Bundle {
    val q = q_t.cloneType
    val cmd = new RoCCCommand

    // FOR DEBUGGING ONLY
    val cmd_id      = UInt(log2Up(MAX_DEBUG_ID).W)
    val is_config   = Bool()
    val is_loadA    = Bool()
    val is_loadB    = Bool()
    val is_loadD    = Bool()
    val is_storeC   = Bool()
    val is_exec     = Bool()
    val is_execflip = Bool()
    val is_preload  = Bool()
    //-----------------------

    // dependency tracking fields
    val op1 = UDValid(new FgLocalRange(config))
    val op2 = UDValid(new FgLocalRange(config))
    val dst = UDValid(new FgLocalRange(config))
    val issued = Bool()
    val complete_on_issue = Bool()
    val deps = Vec(ROB_ENTRIES, Bool())
    def ready(dummy: Int = 0): Bool = !deps.reduce(_ || _)
  }
  val entries = Reg(Vec(ROB_ENTRIES, UDValid(new Entry)))
  val empty = !entries.map(_.valid).reduce(_ || _)
  val full = entries.map(_.valid).reduce(_ && _)

  io.busy := !empty

  //=========================================================================
  // accept new commands
  //=========================================================================
  val last_allocated = Reg(0.U(log2Up(ROB_ENTRIES).W))

  val new_entry = Wire(new Entry)
  new_entry := DontCare
  val new_entry_id = MuxCase((ROB_ENTRIES-1).U, entries.zipWithIndex.map { 
    case (e, i) => !e.valid -> i.U 
  })

  io.cmd_in.ready := !full
  val alloc_fire = io.cmd_in.fire()
  when (io.cmd_in.fire()) {
    val cmd           = io.cmd_in.bits
    val funct         = cmd.inst.funct
    // what major-type of cmd is it
    val is_cfg        = (funct === CONFIG_CMD) 
    val is_load       = (funct === LOAD_CMD)
    val is_store      = (funct === STORE_CMD)
    val is_preload    = (funct === PRELOAD_CMD)
    val is_exec       = (funct === COMPUTE_AND_STAY_CMD) || 
                        (funct === COMPUTE_AND_FLIP_CMD)
    val is_execflip   = (funct === COMPUTE_AND_FLIP_CMD)
    // if config cmd, what subcmd is it
    val rs1_cfg       = cmd.rs1.asTypeOf(new FgConfigRs1)
    val is_cfg_load   = is_cfg && (rs1_cfg.cfgtype === CONFIG_LOAD)
    val is_cfg_store  = is_cfg && (rs1_cfg.cfgtype === CONFIG_STORE)
    val is_cfg_exec   = is_cfg && (rs1_cfg.cfgtype === CONFIG_EX)
    val is_cfg_loadA  = is_cfg_load  && !rs1_cfg.is_acc && !rs1_cfg.is_B_sp
    val is_cfg_loadB  = is_cfg_load  && !rs1_cfg.is_acc && rs1_cfg.is_B_sp
    val is_cfg_loadD  = is_cfg_load  && rs1_cfg.is_acc
    val is_cfg_storeC = is_cfg_store
    // if load cmd, what subcmd is it
    val rs1_laddr     = cmd.rs1.asTypeOf(new FgLocalAddr(config))
    val rs2_laddr     = cmd.rs2.asTypeOf(new FgLocalAddr(config))
    val is_loadA      = is_load && !rs2_laddr.is_acc && !rs2_laddr.is_B_sp
    val is_loadB      = is_load && !rs2_laddr.is_acc && rs2_laddr.is_B_sp
    val is_loadD      = is_load && rs2_laddr.is_acc
    val is_storeC     = is_store

    assert(!cmd_id.inc(), "cmd_id should never wrap!")
    new_entry.issued      := false.B
    new_entry.cmd         := cmd

    new_entry.op1.valid   := is_preload || is_exec
    new_entry.op1.bits    := rs1_laddr
    new_entry.op2.valid   := is_exec || is_store
    new_entry.op2.bits    := rs2_laddr
    new_entry.dst.valid   := is_preload || is_load
    new_entry.dst.bits    := rs2_laddr

    // ALL THE FOLLOWING FOR DEBUGGING ONLY
    new_entry.cmd_id      := cmd_id.value                
    new_entry.is_config   := is_config
    new_entry.is_loadA    := is_loadA   || is_cfg_loadA
    new_entry.is_loadB    := is_loadB   || is_cfg_loadB
    new_entry.is_loadD    := is_loadD   || is_cfg_loadD
    new_entry.is_storeC   := is_storeC  || is_cfg_storeC
    new_entry.is_exec     := is_exec    || is_cfg_exec || is_preload
    new_entry.is_execflip := is_execflip
    new_entry.is_preload  := is_preload
    //======================================================================
    // debug
    //======================================================================
    when(new_entry.is_config) {
      when (new_entry.is_loadA) {
        printf("cycle[%d], entry[%d], accept[%d], config_mvinA[stride=%x]\n", 
          debug_cycle, new_entry_id, cmd_id.value, new_entry.cmd.rs2)
      }
      .elsewhen (new_entry.is_loadB) {
        printf("cycle[%d], entry[%d], accept[%d], config_mvinB[stride=%x]\n", 
          debug_cycle, new_entry_id, cmd_id.value, new_entry.cmd.rs2)
      }
      .elsewhen (new_entry.is_loadD) {
        printf("cycle[%d], entry[%d], accept[%d], config_mvinD[stride=%x]\n", 
          debug_cycle, new_entry_id, cmd_id.value, new_entry.cmd.rs2)
      }
      .elsewhen (new_entry.is_storeC) {
        printf("cycle[%d], entry[%d], accept[%d], config_mvoutC[stride=%x]\n", 
          debug_cycle, new_entry_id, cmd_id.value, new_entry.cmd.rs2)
      }
      .otherwise {
        assert(new_entry.is_exec)
        printf(
          "cycle[%d], entry[%d], accept[%d], " +
          "config_ex[matmul_rshift=%x, acc_rshift=%x, relu6_lshift=%x]\n", 
          debug_cycle, new_entry_id, cmd_id.value, 
          cmd.rs1(63,32), cmd.rs2(31,0), cmd.rs2(63,32))
      }
    }
    .elsewhen (new_entry.is_loadA) {
      printf("cycle[%d], entry[%d], accept[%d], " +
        "mvinA[dram=%x, spad=(%x,%x), rows=%x, cols=%x]\n",
        debug_cycle, new_entry_id, cmd_id.value, 
        cmd.rs1, dst.row_start, dst.fg_col_start, dst.rows, dst.cols)
    }
    .elsewhen (new_entry.is_loadB) {
      printf("cycle[%d], entry[%d], accept[%d], " +
        "mvinB[dram=%x, spad=(%x,%x), rows=%x, cols=%x]\n",
        debug_cycle, new_entry_id, cmd_id.value, 
        cmd.rs1, dst.row_start, dst.fg_col_start, dst.rows, dst.cols)
    }
    .elsewhen (new_entry.is_loadD) {
      printf("cycle[%d], entry[%d], accept[%d], " +
        "mvinD[dram=%x, acc=(%x,%x), rows=%x, cols=%x]\n",
        debug_cycle, new_entry_id, cmd_id.value, 
        cmd.rs1, dst.row_start, dst.fg_col_start, dst.rows, dst.cols)
    }
    .elsewhen (new_entry.is_storeC) {
      printf("cycle[%d], entry[%d], accept[%d], " + 
        "mvoutC[dram=%x, acc=(%x,%x), rows=%x, cols=%x]\n",
        debug_cycle, new_entry_id, cmd_id.value, 
        cmd.rs1, op2.row_start, op2.fg_col_start, op2.rows, op2.cols)
    }
    .elsewhen (new_entry.is_preload) {
      printf("cycle[%d], entry[%d], accept[%d], preload[B=%x, C=%x]\n",
        debug_cycle, new_entry_id, cmd_id.value, 
        cmd.rs1(31,0), cmd.rs2(31,0))
    }
    .elsewhen (new_entry.is_execflip) {
      printf(
        "cycle[%d], entry[%d], accept[%d], " +
        "ex.pre[A=(%x,%x), D=(%x,%x), rows=%x, cols=%x]\n",
        debug_cycle, new_entry_id, cmd_id.value, 
        op1.row_start, op1.fg_col_start, 
        op2.row_start, op2.fg_col_start, op2.rows, op2.cols)
    }
    .otherwise {
      assert(new_entry.is_exec)
      printf(
        "cycle[%d], entry[%d], accept[%d], " +
        "ex.acc[A=(%x,%x), D=(%x,%x), rows=%x, cols=%x]\n",
        debug_cycle, new_entry_id, cmd_id.value, 
        op1.row_start, op1.fg_col_start, 
        op2.row_start, op2.fg_col_start, op2.rows, op2.cols)
    }

    //======================================================================
    new_entry.q := Mux1H(Seq(
      new_entry.is_loadA   -> ldaq,
      new_entry.is_loadB   -> ldbq,
      new_entry.is_loadD   -> lddq,
      new_entry.is_storeC  -> stcq,
      new_entry.is_exec    -> exq,
    ))

    // We search for all entries which write to an address which we read from
    val raws = entries.map { e => 
      e.valid && 
      e.bits.dst.valid && (
      (e.bits.dst.bits.overlaps(new_entry.op1.bits) && new_entry.op1.valid) ||
      (e.bits.dst.bits.overlaps(new_entry.op2.bits) && new_entry.op2.valid)
    )}

    // We search for all entries which read from an address that we write to
    val wars = entries.map { e => 
      e.valid && 
      new_entry.dst.valid &&
      (new_entry.dst.bits.overlaps(e.bits.op1.bits) && e.bits.op1.valid) ||
      (new_entry.dst.bits.overlaps(e.bits.op2.bits) && e.bits.op2.valid)
    )}

    // We search for all entries which write to an address that we write to
    val waws = entries.map { e => 
      e.valid && 
      e.bits.dst.valid &&
      new_entry.dst.valid &&
      new_entry.dst.bits.overlaps(e.bits.dst.bits)
    )}

    val older_in_same_q = entries.map { e => 
      e.valid && 
      e.bits.q === new_entry.q && 
      !e.bits.issued
    }

    // because ex-config modifies the relu/relu6/shift out of accumulator
    val is_st_and_must_wait_for_prior_ex_config = entries.map { e =>
      (e.valid && e.bits.q === exq && e.bits.is_config) &&
      (new_entry.q === stcq && !new_entry.is_config)
    }
    val is_ex_config_and_must_wait_for_prior_st = entries.map { e =>
      (e.valid && e.bits.q === stcq && !e.bits.is_config) &&
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

  //=========================================================================
  // Issue commands which are ready to be issued
  //=========================================================================
  Seq((ldaq, io.issue.loadA), 
      (ldbq, io.issue.loadB), 
      (lddq, io.issue.loadD), 
      (stcq, io.issue.storeC), 
      (exq,  io.issue.exec)).foreach { case (q, io) =>
    val issue_id = MuxCase((ROB_ENTRIES-1).U, entries.zipWithIndex.map {
      case (e, i) => (e.valid && e.bits.ready() && 
                      !e.bits.issued && e.bits.q === q) -> i.U
    })
    io.valid := entries.map(e => e.valid && e.bits.ready() && !e.bits.issued 
                                 && e.bits.q === q).reduce(_ || _)
    io.bits.cmd := entries(issue_id).bits.cmd
    io.bits.rob_id := issue_id

    // ssteff: added for debug
    when(io.fire()) {
      //======================================================================
      // debug
      //======================================================================
      when(entries(issue_id).bits.is_config) {
        when (entries(issue_id).bits.is_loadA) {
          printf("cycle[%d], entry[%d],  issue[%d], config_mvinA\n",
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
          printf("cycle[%d], entry[%d],  final[%d], config_mvinA\n",
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
        }
        .elsewhen (entries(issue_id).bits.is_loadB) {
          printf("cycle[%d], entry[%d],  issue[%d], config_mvinB\n",
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
          printf("cycle[%d], entry[%d],  final[%d], config_mvinB\n",
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
        }
        .elsewhen (entries(issue_id).bits.is_loadD) {
          printf("cycle[%d], entry[%d],  issue[%d], config_mvinD\n",
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
          printf("cycle[%d], entry[%d],  final[%d], config_mvinD\n",
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
        }
        .elsewhen (entries(issue_id).bits.is_storeC) {
          printf("cycle[%d], entry[%d],  issue[%d], config_mvoutC\n",
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
          printf("cycle[%d], entry[%d],  final[%d], config_mvoutC\n", 
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
        }
        .otherwise {
          assert(entries(issue_id).bits.is_exec)
          printf("cycle[%d], entry[%d],  issue[%d], config_ex\n",
            debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
        }
      }
      .elsewhen (entries(issue_id).bits.is_loadA) {
        printf("cycle[%d], entry[%d],  issue[%d], mvinA\n",
          debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
      }
      .elsewhen (entries(issue_id).bits.is_loadB) {
        printf("cycle[%d], entry[%d],  issue[%d], mvinB\n",
          debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
      }
      .elsewhen (entries(issue_id).bits.is_loadD) {
        printf("cycle[%d], entry[%d],  issue[%d], mvinD\n",
          debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
      }
      .elsewhen (entries(issue_id).bits.is_storeC) {
        printf("cycle[%d], entry[%d],  issue[%d], mvoutC\n",
          debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
      }
      .elsewhen (entries(issue_id).bits.is_preload) {
        printf("cycle[%d], entry[%d],  issue[%d], preload\n",
          debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
      }
      .elsewhen (entries(issue_id).bits.is_execflip) {
        printf("cycle[%d], entry[%d],  issue[%d], ex.pre\n",
          debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
      }
      .otherwise {
        assert(entries(issue_id).bits.is_exec)
        printf("cycle[%d], entry[%d],  issue[%d], ex.acc\n",
          debug_cycle, issue_id, entries(issue_id).bits.cmd_id)
      }
      //--------------------------------------------------------
      entries(issue_id).bits.issued := true.B

      // Clear out all the dependency bits for instructions which 
      // depend on the same queue
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
 
  //========================================================================
  // Mark entries as completed once they've returned
  //========================================================================
  when (io.completed.fire()) {
    // debug
    when (entries(io.completed.bits).bits.is_config) {
      assert(entries(io.completed.bits).bits.is_exec)
      printf("cycle[%d], entry[%d],  final[%d], config_ex\n",
        debug_cycle, io.completed.bits, 
        entries(io.completed.bits).bits.cmd_id)
    }
    .elsewhen (entries(io.completed.bits).bits.is_loadA) {
      printf("cycle[%d], entry[%d],  final[%d], mvinA\n",
        debug_cycle, io.completed.bits, 
        entries(io.completed.bits).bits.cmd_id)
    }
    .elsewhen (entries(io.completed.bits).bits.is_loadB) {
      printf("cycle[%d], entry[%d],  final[%d], mvinB\n",
        debug_cycle, io.completed.bits, 
        entries(io.completed.bits).bits.cmd_id)
    }
    .elsewhen (entries(io.completed.bits).bits.is_loadD) {
      printf("cycle[%d], entry[%d],  final[%d], mvinD\n",
        debug_cycle, io.completed.bits, 
        entries(io.completed.bits).bits.cmd_id)
    }
    .elsewhen (entries(io.completed.bits).bits.is_storeC) {
      printf("cycle[%d], entry[%d],  final[%d], mvoutC\n",
        debug_cycle, io.completed.bits, 
        entries(io.completed.bits).bits.cmd_id)
    }
    .elsewhen (entries(io.completed.bits).bits.is_preload) {
      printf("cycle[%d], entry[%d],  final[%d], preload\n",
        debug_cycle, io.completed.bits, 
        entries(io.completed.bits).bits.cmd_id)
    }
    .elsewhen (entries(io.completed.bits).bits.is_execflip) {
      printf("cycle[%d], entry[%d],  final[%d], ex.pre\n",
        debug_cycle, io.completed.bits, 
        entries(io.completed.bits).bits.cmd_id)
    }
    .otherwise {
      assert(entries(io.completed.bits).bits.is_exec)
      printf("cycle[%d], entry[%d],  final[%d], ex.acc\n",
        debug_cycle, io.completed.bits, 
        entries(io.completed.bits).bits.cmd_id)
    }
    //-----------------------------------------------------------
    entries.foreach(_.bits.deps(io.completed.bits) := false.B)
    entries(io.completed.bits).valid := false.B
    assert(entries(io.completed.bits).valid)
  }

  //========================================================================
  // debugging/utilization report
  //========================================================================
  val util = PopCount(entries.map(e => e.valid))
  val util_ld_q_unissued = PopCount(entries.map(e => e.valid && 
                                                     !e.bits.issued && 
                                                     e.bits.q === ldq))
  val util_st_q_unissued = PopCount(entries.map(e => e.valid && 
                                                     !e.bits.issued && 
                                                     e.bits.q === stq))
  val util_ex_q_unissued = PopCount(entries.map(e => e.valid && 
                                                     !e.bits.issued && 
                                                     e.bits.q === exq))
  val util_ld_q = PopCount(entries.map(e => e.valid && e.bits.q === ldq))
  val util_st_q = PopCount(entries.map(e => e.valid && e.bits.q === stq))
  val util_ex_q = PopCount(entries.map(e => e.valid && e.bits.q === exq))

  val packed_deps = VecInit(entries.map(e => Cat(e.bits.deps)))
  dontTouch(packed_deps)

  val pop_count_packed_deps = VecInit(entries.map(e => Mux(e.valid, PopCount(e.bits.deps), 0.U)))
  val min_pop_count = pop_count_packed_deps.reduce((acc, d) => minOf(acc, d))
  // assert(min_pop_count < 2.U)
  dontTouch(pop_count_packed_deps)
  dontTouch(min_pop_count)

  val cycles_since_issue = RegInit(0.U(32.W))

  when (io.issue.load.fire() || 
        io.issue.store.fire() || 
        io.issue.exec.fire() || 
        !io.busy) {
    cycles_since_issue := 0.U
  } .elsewhen (io.busy) {
    cycles_since_issue := cycles_since_issue + 1.U
  }
  assert(cycles_since_issue < 10000.U, "pipeline stall")

  val cntr = Counter(10000000)
  when (cntr.inc()) {
    printf(p"Utilization: $util\n")
    printf(p"Utilization ld q (incomplete): $util_ld_q_unissued\n")
    printf(p"Utilization st q (incomplete): $util_st_q_unissued\n")
    printf(p"Utilization ex q (incomplete): $util_ex_q_unissued\n")
    printf(p"Utilization ld q: $util_ld_q\n")
    printf(p"Utilization st q: $util_st_q\n")
    printf(p"Utilization ex q: $util_ex_q\n")
    printf(p"Packed deps: $packed_deps\n")
    printf(p"Last allocated: $last_allocated\n\n")
  }

  when (reset.toBool()) {
    entries.foreach(_.valid := false.B)
  }
}

object TilerScheduler {
  def apply[T <: Data: Arithmetic]
    (config: GemminiArrayConfig[T])(implicit p: Parameters)
      = Module(new TilerScheduler(config))
}
