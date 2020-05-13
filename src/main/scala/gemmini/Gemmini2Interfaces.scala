//============================================================================
// this contains interfaces between the Gemmini2 components
//============================================================================
package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import GemminiISA._

//===========================================================================
// From ROB to {exec,load,store,flush} units
//===========================================================================
class GemminiCmd(val ROB_ENTRIES_IDX: Int)
  (implicit p: Parameters) extends CoreBundle {
  val cmd = new RoCCCommand
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

//===========================================================================
// Per-Matrix Addressing Mode
//===========================================================================
class TilerCmdAddrCfg(implicit p: Parameters) extends CoreBundle {
  val mode        = UInt(1.W) // AddressMode
  val in_rows     = UInt(32.W)
  val in_cols     = UInt(32.W)
  val stride      = UInt(16.W)
  val padding     = UInt(8.W)
  val in_channels = UInt(16.W)
  val kernel_size = UInt(16.W)
}

//===========================================================================
// TilerController Interface (gemmini2 mode only)
// - NOTE: all gemminis support im2col interface, even if they don't use it.
//         this includes the fg-array implementation too!
//===========================================================================
class TilerCmd(OTYPE_BITS_IDX: Int)
  (implicit p: Parameters) extends CoreBundle {
  val m              = UInt(32.W)
  val n              = UInt(32.W)
  val k              = UInt(32.W)
  val addr_a         = UInt(xLen.W)
  val addr_b         = UInt(xLen.W)
  val addr_c         = UInt(xLen.W)
  val addr_d         = UInt(xLen.W)
  val addr_a_cfg     = new TilerCmdAddrCfg
  val addr_b_cfg     = new TilerCmdAddrCfg
  val addr_c_cfg     = new TilerCmdAddrCfg
  val addr_d_cfg     = new TilerCmdAddrCfg
  val in_rshift      = UInt(OTYPE_BITS_IDX.W)
  val acc_rshift     = UInt(OTYPE_BITS_IDX.W)
  val relu6_lshift   = UInt(OTYPE_BITS_IDX.W)
  val activation     = UInt(2.W)
  val repeating_bias = Bool()
  val status         = new MStatus
}

//===========================================================================
// Instrumentation for profile counters
//===========================================================================
class Profiling extends Bundle {
  val start       = Bool()
  val end         = Bool()
  val cycle       = UInt(32.W)
  val debug_cycle = UInt(40.W)
}

