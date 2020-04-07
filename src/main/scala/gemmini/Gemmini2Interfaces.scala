//============================================================================
// this contains interfaces between the Gemmini2 components
//============================================================================
package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
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
// TilerController Interface (gemmini2 mode only)
//===========================================================================
class TilerCmd(config: GemminiArrayConfig[T])(implicit p: Parameters)
  extends CoreBundle {
  import config._
  val M              = UInt(32.U)
  val N              = UInt(32.U)
  val K              = UInt(32.U)
  val addr_a         = UInt(xLen.W)
  val addr_b         = UInt(xLen.W)
  val addr_c         = UInt(xLen.W)
  val addr_d         = UInt(xLen.W)
  val in_rshift      = UInt(log2Up(accType.getWidth).W)
  val acc_rshift     = UInt(log2Up(accType.getWidth).W)
  val relu6_lshift   = UInt(log2Up(accType.getWidth).W)
  val activation     = UInt(2.W)
  val repeating_bias = Bool()

  override def cloneType: this.type =
    new TilerCmd(config).asInstanceOf[this.type]
}
