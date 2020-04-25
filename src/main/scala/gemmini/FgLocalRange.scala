package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import Util._

//===========================================================================
// represents a 2-D array of rows/cols in a scratchpad/accumulator
//===========================================================================
class FgLocalRange[T <: Data](config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val rows         = UInt(16.W)
  val cols         = UInt(16.W)
  val is_acc       = Bool()
  val is_accum     = Bool()
  val is_B_sp      = Bool()
  val garbage      = Bool()
  val sq_col_start = UInt(12.W)
  val row_start    = UInt(16.W)

  // total useful bytes
  def total_bytes(dummy: Int = 0) = rows * cols * 
                                    Mux(is_acc, OTYPE_BYTES.U, ITYPE_BYTES.U)
}

