package gemmini

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths}

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import GemminiISA._

//===========================================================================
// represents a 2-D array of rows/cols in a scratchpad/accumulator
//===========================================================================
class LocalRange[T <: Data](val config: GemminiArrayConfig[T])
  (implicit p: Parameters) extends CoreBundle {
  import config._
  // which local memory/bank is this data range
  val is_acc    = Bool()
  val is_accum  = Bool()
  val garbage   = Bool()
  val rows      = UInt(32.W)
  val row_start = UInt(32.W)

  // normal-mode cols
  val cols = UInt(DIM.W)

  // im2col-mode fields
  val is_im2col = Bool()
  val krows     = UInt(16.U)
  val kcols     = UInt(16.U)
  val in_chans  = UInt(16.U)
  val krow      = UInt(16.U)
  val kcol      = UInt(16.U)
  val in_chan   = UInt(16.U)

  //---------------------------------------------------------------
  // methods
  //---------------------------------------------------------------
  def total_elems(dummy: Int = 0) = Mux(is_im2col, in_chans * kcols, cols)
  def total_read_bytes(dummy: Int = 0) = total_elems() * ITYPE_BYTES.U
  def total_write_bytes(dummy: Int = 0) 
    = total_elems() * Mux(is_acc, OTYPE_BYTES.U, ITYPE_BYTES.U)


  // used to calculate bank conflicts
  def row_end(dummy: Int = 0)   = row_start + rows - 1.U
  def bank_rows(dummy: Int = 0) = Mux(is_acc, ACC_BANK_ROWS, SP_BANK_ROWS) 

  def bank_start(dummy: Int = 0)
    = Mux(is_acc, row_start(31, ACC_BANK_ROWS_IDX)
                  row_start(31, SP_BANK_ROWS_IDX))

  def bank_end(dummy: Int = 0)
    = Mux(is_acc, row_end()(31, ACC_BANK_ROWS_IDX)
                  row_end()(31, SP_BANK_ROWS_IDX))

  def total_banks(dummy: Int = 0) = bank_end() - bank_start() + 1.U

  def row_start_within_bank(dummy: Int = 0) 
    = Mux(is_acc, row_start(ACC_BANK_ROWS_IDX-1,0)
                  row_start(SP_BANK_ROWS_IDX-1,0))

  // do two ranges have data range overlap (one read and one write)
  def overlaps(other: LocalRange[T])
    = (is_acc === other.is_acc) &&
      (bank_start() <= other.bank_end()) &&
      (bank_end()   >= other.bank_start())
}
