//============================================================================
// this file contains the top-level Gemmini2 controller
// - it uses a CmdController to parse the input RoCC commands
// - it uses a TilerController to drive tiled matmul in hardware
// - it has DMA engines that do im2col on the fly
//============================================================================
package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import GemminiISA._

class Gemmini2[T <: Data : Arithmetic]
  (opcodes: OpcodeSet, val config: GemminiArrayConfig[T])
  (implicit p: Parameters)
  extends LazyRoCC(opcodes=OpcodeSet.custom3, nPTWPorts=1)
{
  def this(tileParams: TileParams, crossing: ClockCrossingType, lookup: LookupByHartIdImpl, p: Parameter
) = {
    this(crossing, p.alterMap(Map(
      TileKey -> tileParams,
      TileVisibilityNodeKey -> TLEphemeralNode()(ValName("tile_master")),
      LookupByHartId -> lookup
    )))
  }





  val spad = LazyModule(new Scratchpad(config))
  override lazy val module = new GemminiModule2(this)
  override val tlNode = spad.id_node
}

class GemminiModule2[T <: Data: Arithmetic](outer: Gemmini2[T])
  extends LazyRoCCModuleImp(outer) with HasCoreParameters
{
  import outer.config._
  //import outer.spad

  //=========================================================================
  // OoO Issuing
  //=========================================================================
  val raw_cmd = Queue(io.cmd)

  val parser = CmdParser(outer.config))
  parser.io.in <> raw_cmd

  val tiler = TilerController(outer.config)
  tiler.io.cmd_in <> parser.io.cmd_out

  //=========================================================================
  // TLB (2-clients, 4 entries)
  //=========================================================================
  val tlb = FrontendTLB(2, 4, dma_maxbytes, outer.tlNode.edges.out.head)
  tlb.io.clients(0)    <> outer.spad.module.io.tlb(0)
  tlb.io.clients(1)    <> outer.spad.module.io.tlb(1)
  io.ptw.head          <> tlb.io.ptw
  io.interrupt         := tlb.io.exp.interrupt
  spad.module.io.flush := tlb.io.exp.flush()

  dontTouch(outer.spad.module.io.tlb)

  //=========================================================================
  // OoO Execution
  //=========================================================================
  val exec = ExecuteController(outer.config)
  exec.io.cmd                <> tiler.io.issue.exec
  tiler.io.completed.exec    := exec.io.completed
  spad.module.io.srams.read  <> exec.io.srams.read
  spad.module.io.srams.write <> exec.io.srams.write
  spad.module.io.acc.read    <> exec.io.acc.read
  spad.module.io.acc.write   <> exec.io.acc.write

  val load = LoadController(outer.config)
  load.io.cmd             <> tiler.io.issue.load
  tiler.io.completed.load <> load.io.completed
  spad.module.io.dma.read <> load.io.dma

  val store = StoreController(outer.config)
  store.io.cmd             <> tiler.io.issue.store
  tiler.io.completed.store <> store.io.completed
  spad.module.io.dma.write <> store.io.dma

  val flush = FlushController(outer.config)
  flush.io.cmd             <> tiler.io.issue.flush
  tiler.io.completed.flush <> flush.io.completed
  tlb.io.exp.flush_retry   := flush.io.flush_retry
  tlb.io.exp.flush_skip    := flush.io.flush_skip

  //=========================================================================
  // Power Management Unit
  //=========================================================================
  //val pmu  = PowerManagement(outer.config)
  //pmu.io.tiler_hint <> tiler.io.
  //tiler.io.in <> cmd_parser.io.out

  //=========================================================================
  // Busy Signal (used by RocketCore during fence insn)
  //=========================================================================
  io.busy := raw_cmd.valid || cmd_parser.io.busy || tiler.io.busy ||
             spad.module.io.busy ||
             load.io.busy || store.io.busy || exec.io.busy || flush.io.busy
}
