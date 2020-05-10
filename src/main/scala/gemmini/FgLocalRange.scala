package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import Util._

//===========================================================================
// represents a 2-D array of rows/cols in a scratchpad/accumulator
//===========================================================================
class FgLocalRange[T <: Data](val config: FgGemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  val is_acc    = Bool()
  val is_accum  = Bool()
  val is_B_sp   = Bool()
  val garbage   = Bool()
  val rows      = UInt(15.W)
  val cols      = UInt(15.W)
  val row_start = UInt(15.W)
  val col_start = UInt(15.W)

  def total_read_bytes(dummy: Int = 0) = rows * cols * ITYPE_BYTES.U
  def total_write_bytes(dummy: Int = 0) 
    = rows * cols * Mux(is_acc, OTYPE_BYTES.U, ITYPE_BYTES.U)

  // inclusive
  def col_end(dummy: Int = 0)   = col_start + cols - 1.U

  // which bank does this start/end read/write to
  // for example, if FG_DIM is 4, rows=16, and row_start is 0, then row_end
  // is 12, since when you read/write to 4 banks at a time starting at row 0,
  // the last row is 12
  def row_end(dummy: Int = 0)   = row_start + (((rows-1.U)/FG_DIM.U)*FG_DIM.U)
  def bank_start(dummy: Int = 0)  = row_start(14, FG_DIM_IDX)
  def bank_end(dummy: Int = 0)    = row_end()(14, FG_DIM_IDX)
  def total_banks(dummy: Int = 0) = bank_end() - bank_start() + 1.U

  // NOTE: this should always be 0...
  def row_start_within_bank(dummy: Int = 0) = row_start(FG_DIM_IDX-1,0)

  // do two ranges have data range overlap (one read and one write)
  def overlaps(other: FgLocalRange[T])
    = (is_acc === other.is_acc) && 
      (is_acc || (is_B_sp === other.is_B_sp)) &&
      (bank_start() <= other.bank_end()) &&
      (bank_end()   >= other.bank_start()) &&
      (col_start    <= other.col_end()) &&
      (col_end()    >= other.col_start)

  // do two ranges have bank conflicts (both writes or both reads)
  def conflicts(other: FgLocalRange[T]) 
    = (is_acc === other.is_acc) && 
      (is_acc || (is_B_sp === other.is_B_sp)) &&
      (bank_start() <= other.bank_end()) &&
      (bank_end()   >= other.bank_start())
}

//===========================================================================
// when CONFIG_LOAD is used, specifies if loadA, loadB, or loadD
//===========================================================================
class FgConfigRs1 extends Bundle {
  val garbage      = UInt(60.W)
  val is_acc       = Bool()
  val is_B_sp      = Bool()
  val cfgtype      = UInt(2.W)
}
