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
class GemminiCmd(rob_entries: Int)(implicit p: Parameters) extends Bundle {
  val cmd = new RoCCCommand
  val rob_id = UInt(log2Up(rob_entries).W)
  override def cloneType: this.type 
    = (new GemminiCmd(rob_entries)).asInstanceOf[this.type]
}

//===========================================================================
// Per-Matrix Addressing Mode
// Supported Modes:
// * Row-Major (default)
// * Im2Col
//===========================================================================
class MatrixAddrConfig[T <: Data: Arithmetic]
  (config: GemminiArrayConfig[T])(implicit p: Parameters) extends CoreBundle {
  import config._
  val base_addr      = UInt(xLen.W)
  val addr_mode      = UInt(3.W)

  // Im2Col-Mode Params
  val rows = UInt(32.W);
  val cols = UInt(32.W);
  val batch_size = UInt(16.W);
  val kernel_size = UInt(16.W);
  val channels = UInt(8.W);
  val padding = UInt(8.W);
  val stride = UInt(8.W);

  override def cloneType: this.type =
    (new MatrixAddrConfig(config)).asInstanceOf[this.type]
}

//===========================================================================
// TilerController Interface (gemmini2 mode only)
//===========================================================================
class TilerCmd[T <: Data: Arithmetic]
  (config: GemminiArrayConfig[T])(implicit p: Parameters) extends CoreBundle {
  import config._

  val m              = UInt(32.W)
  val n              = UInt(32.W)
  val k              = UInt(32.W)

  val a              = new MatrixAddrConfig(config)
  val b              = new MatrixAddrConfig(config)
  val c              = new MatrixAddrConfig(config)
  val d              = new MatrixAddrConfig(config)

  val in_rshift      = UInt(log2Up(accType.getWidth).W)
  val acc_rshift     = UInt(log2Up(accType.getWidth).W)
  val relu6_lshift   = UInt(log2Up(accType.getWidth).W)
  val activation     = UInt(2.W)
  val repeating_bias = Bool()
  val status         = new MStatus

  override def cloneType: this.type =
    (new TilerCmd(config)).asInstanceOf[this.type]
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

