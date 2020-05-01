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
class GemminiCmd(ROB_ENTRIES_IDX: Int)
  (implicit p: Parameters) extends Bundle {
  val cmd = new RoCCCommand
  val rob_id = UInt(ROB_ENTRIES_IDX.W)

  override def cloneType: this.type 
    = (new GemminiCmd(ROB_ENTRIES_IDX)).asInstanceOf[this.type]
}

//===========================================================================
// TilerController Interface (gemmini2 mode only)
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
  val in_rshift      = UInt(OTYPE_BITS_IDX.W)
  val acc_rshift     = UInt(OTYPE_BITS_IDX.W)
  val relu6_lshift   = UInt(OTYPE_BITS_IDX.W)
  val activation     = UInt(2.W)
  val repeating_bias = Bool()
  val status         = new MStatus

  override def cloneType: this.type =
    (new TilerCmd(OTYPE_BITS_IDX)).asInstanceOf[this.type]
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

