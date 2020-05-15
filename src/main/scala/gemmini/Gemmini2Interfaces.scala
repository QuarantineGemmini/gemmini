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
// From ROB to {exec,load,store} units
//===========================================================================
class GemminiCmd(val ROB_ENTRIES_IDX: Int)
  (implicit p: Parameters) extends CoreBundle {
  val funct  = UInt(7.W)
  val rs1    = UInt(xLen.W)
  val rs2    = UInt(xLen.W)
  val rob_id = UInt(ROB_ENTRIES_IDX.W)
}

//===========================================================================
// Gemmini Internal Control Status Registers
//===========================================================================
class GemminiCSRAddr(implicit p: Parameters) extends CoreBundle {
  val vaddr         = UInt(xLen.W)
  val mode          = UInt(1.W)
  val normal_stride = Input(UInt(32.W)) // normal-mode byte stride
  val in_rows       = UInt(32.W)        // im2col-mode configs
  val in_cols       = UInt(32.W)
  val stride        = UInt(16.W)
  val padding       = UInt(8.W)
  val in_channels   = UInt(16.W)
  val kernel_size   = UInt(16.W)
}

// TODO: change the fields in this depending on if orig_tiler or hw_tiler
class GemminiCSR(OTYPE_BITS_IDX: Int)
  (implicit p: Parameters) extends CoreBundle {
  val m              = UInt(32.W)
  val n              = UInt(32.W)
  val k              = UInt(32.W)
  val addr_a_cfg     = new GemminiCSRAddr
  val addr_b_cfg     = new GemminiCSRAddr
  val addr_c_cfg     = new GemminiCSRAddr
  val addr_d_cfg     = new GemminiCSRAddr
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

