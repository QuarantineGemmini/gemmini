//============================================================================
// this file contains the top-level Gemmini2 controller
// - it uses a CmdController to parse the input RoCC commands
// - it uses a TilerController to drive tiled matmul in hardware
// - it has DMA engines that do im2col on the fly
//============================================================================
package gemmini

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths}

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import GemminiISA._

class FgGemmini2[T <: Data : Arithmetic]
  (opcodes: OpcodeSet, val config: GemminiArrayConfig[T])
  (implicit p: Parameters)
  extends LazyRoCC(opcodes=OpcodeSet.custom3, nPTWPorts=1) {

  Files.write(Paths.get(config.headerFilePath),
              config.generateHeader().getBytes(StandardCharsets.UTF_8))

  override lazy val module = new FgGemminiModule2(this)

  val mem = LazyModule(new FgMemUnit(config))
  override val tlNode = mem.id_node
}

class FgGemminiModule2[T <: Data: Arithmetic](outer: FgGemmini2[T])
  (implicit p: Parameters)
  extends LazyRoCCModuleImp(outer) with HasCoreParameters {
  import outer.config
  import config._
  import outer.mem

  //=========================================================================
  // TLB (2-clients, 4 entries)
  //=========================================================================
  val tlb = FrontendTLB(4, 8, dma_maxbytes, outer.tlNode.edges.out.head)
  tlb.io.clients(0)   <> mem.module.io.tlb.readA
  tlb.io.clients(1)   <> mem.module.io.tlb.readB
  tlb.io.clients(2)   <> mem.module.io.tlb.readD
  tlb.io.clients(3)   <> mem.module.io.tlb.writeC
  io.ptw.head         <> tlb.io.ptw
  io.interrupt        := tlb.io.exp.interrupt
  mem.module.io.flush := tlb.io.exp.flush()

  //=========================================================================
  // Dispatch/Issue
  //=========================================================================
  val raw_cmd = Queue(io.cmd)

  val cmd_fsm = CmdFSM(outer.config)
  cmd_fsm.io.cmd         <> raw_cmd
  tlb.io.exp.flush_retry := cmd_fsm.io.flush_retry
  tlb.io.exp.flush_skip  := cmd_fsm.io.flush_skip

  val tiler = TilerController(outer.config)
  tiler.io.cmd_in <> cmd_fsm.io.tiler

  //=========================================================================
  // Execution
  //=========================================================================
  val exec = FgExecuteController(outer.config)
  exec.io.cmd               <> tiler.io.issue.exec
  tiler.io.completed.exec   := exec.io.completed
  mem.module.io.exec.readA  <> exec.io.readA
  mem.module.io.exec.readB  <> exec.io.readB
  mem.module.io.exec.readD  <> exec.io.readD
  mem.module.io.exec.writeC <> exec.io.writeC

  val loadA = FgMemTransferController(outer.config)
  loadA.io.cmd             <> tiler.io.issue.loadA
  tiler.io.completed.loadA <> loadA.io.completed
  mem.module.io.dma.readA  <> loadA.io.dma

  val loadB = FgMemTransferController(outer.config)
  loadB.io.cmd             <> tiler.io.issue.loadB
  tiler.io.completed.loadB <> loadB.io.completed
  mem.module.io.dma.readB  <> loadB.io.dma

  val loadD = FgMemTransferController(outer.config)
  loadD.io.cmd             <> tiler.io.issue.loadD
  tiler.io.completed.loadD <> loadD.io.completed
  mem.module.io.dma.readD  <> loadD.io.dma

  val storeC = FgMemTransferController(outer.config)
  storeC.io.cmd             <> tiler.io.issue.storeC
  tiler.io.completed.storeC <> storeC.io.completed
  mem.module.io.dma.writeC  <> storeC.io.dma

  //=========================================================================
  // Busy Signal (used by RocketCore during fence insn)
  //=========================================================================
  val is_computing = cmd_fsm.io.busy || tiler.io.busy ||
                     mem.module.io.busy ||
                     loadA.io.busy || loadB.io.busy || loadD.io.busy || 
                     storeC.io.busy || exec.io.busy

  io.busy := raw_cmd.valid || is_computing

  //=========================================================================
  // instrumentation for profiling counters
  // - don't use raw_cmd.valid, since it causes spurious prof_starts when
  //   the cmd_fsm is being configured
  //=========================================================================
  val busy_last  = RegNext(is_computing)
  val prof_start = WireDefault(~busy_last & is_computing)
  val prof_end   = WireDefault(busy_last & ~is_computing)

  val prof_cycle = RegInit(0.U(32.W))
  prof_cycle := Mux(prof_start, 0.U, prof_cycle + 1.U)

  val debug_cycle = RegInit(0.U(40.W))
  debug_cycle := debug_cycle + 1.U

  val prof = Wire(new Profiling)
  prof.start       := prof_start
  prof.end         := prof_end
  prof.cycle       := prof_cycle
  prof.debug_cycle := debug_cycle

  exec.io.prof := prof

  when(prof.end) {
    printf(s"G2-PERF[%d]: total-cycles: %d\n", prof.debug_cycle, prof.cycle)
  }
}
