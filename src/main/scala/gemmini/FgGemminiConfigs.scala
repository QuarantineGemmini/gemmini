//==========================================================================
// NOTE on constant terminology
// - {foo}_IDX means the number of bits to index {foo} items. if {foo} === 0,
//   then {foo}_IDX will be 0 bits!
// - {foo}_CTR means the number of bits to count {foo} items
//==========================================================================
package gemmini

import scala.math.{pow,sqrt}
import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.system._
import freechips.rocketchip.tile._
import GemminiISA._

object WithFgGemmini2Config {
  def apply[T <: Data : Arithmetic](config: FgGemminiArrayConfig[T]) = {
    new Config((site, here, up) => {
      case BuildRoCC => Seq((p: Parameters) => {
        implicit val q = p
        implicit val v = implicitly[ValName]
        LazyModule(new FgGemmini2(OpcodeSet.custom3, config))
      })
      case SystemBusKey => 
        up(SystemBusKey).copy(beatBytes = (config.dma_beat_bits/8))
    })
  }
}

case class FgGemminiArrayConfig[T <: Data : Arithmetic](
  dim: Int,
  fg_dim: Int,
  //------------------------
  mem_op_queue_length: Int,
  ex_queue_length: Int,
  rob_entries: Int,
  //------------------------
  dma_max_req_bytes: Int,
  dma_beat_bits: Int,
  //------------------------
  inputType: T,
  outputType: T,
  accType: T,
) {
  //==========================================================================
  // mesh size/fg-mesh size
  //==========================================================================
  def DIM     = dim
  def DIM_IDX = log2Up(DIM)
  def DIM_CTR = log2Up(DIM+1)
  require(DIM >= 2)

  // fg-array rows/cols
  def SQRT_FG_NUM     = dim / fg_dim
  def SQRT_FG_NUM_IDX = log2Up(SQRT_FG_NUM)
  def SQRT_FG_NUM_CTR = log2Up(SQRT_FG_NUM+1)
  def FG_NUM          = SQRT_FG_NUM * SQRT_FG_NUM
  def FG_NUM_IDX      = log2Up(FG_NUM)
  def FG_NUM_CTR      = log2Up(FG_NUM+1)

  // element rows/cols per fg-tile
  def FG_DIM     = fg_dim
  def FG_DIM_IDX = log2Up(FG_DIM)
  def FG_DIM_CTR = log2Up(FG_DIM+1)
  require(FG_DIM >= 1)

  //==========================================================================
  // ROB tag entries
  //==========================================================================
  def ROB_ENTRIES     = rob_entries
  def ROB_ENTRIES_IDX = log2Up(rob_entries)

  //==========================================================================
  // input/output element sizes
  //==========================================================================
  def ITYPE_BITS   = inputType.getWidth
  def ITYPE_BYTES  = (inputType.getWidth+7) / 8
  def OTYPE_BITS   = accType.getWidth
  def OTYPE_BYTES  = (accType.getWidth+7) / 8

  def ITYPE_BITS_IDX  = log2Up(ITYPE_BITS)
  def OTYPE_BITS_IDX  = log2Up(OTYPE_BITS)
  def ITYPE_BYTES_IDX = log2Ceil(ITYPE_BYTES) // can be 0
  def OTYPE_BYTES_IDX = log2Ceil(OTYPE_BYTES) // can be 0

  //==========================================================================
  // scratchpad/accumulator banks/rows
  //==========================================================================
  def BANK_ROWS               = FG_DIM
  def BANK_ROWS_IDX           = log2Up(BANK_ROWS)
  def BANK_ROWS_CTR           = log2Up(BANK_ROWS+1)
  //------------------------------------------------------
  def A_SP_BANKS              = FG_NUM
  def A_SP_BANK_IDX           = log2Up(A_SP_BANKS)
  def A_SP_BANK_CTR           = log2Up(A_SP_BANKS+1)

  def A_SP_ROWS               = BANK_ROWS * A_SP_BANKS
  def A_SP_ROWS_IDX           = log2Up(A_SP_ROWS)
  def A_SP_ROWS_CTR           = log2Up(A_SP_ROWS+1)
  //------------------------------------------------------
  def B_SP_BANKS              = 2
  def B_SP_BANK_IDX           = log2Up(B_SP_BANKS)
  def B_SP_BANK_CTR           = log2Up(B_SP_BANKS+1)

  def B_SP_ROWS               = BANK_ROWS * B_SP_BANKS
  def B_SP_ROWS_IDX           = log2Up(B_SP_ROWS)
  def B_SP_ROWS_CTR           = log2Up(B_SP_ROWS+1)
  //------------------------------------------------------
  def CD_ACC_BANKS            = FG_NUM
  def CD_ACC_BANK_IDX         = log2Up(CD_ACC_BANKS)
  def CD_ACC_BANK_CTR         = log2Up(CD_ACC_BANKS+1)

  def CD_ACC_ROWS             = BANK_ROWS * CD_ACC_BANKS
  def CD_ACC_ROWS_IDX         = log2Up(CD_ACC_ROWS)
  def CD_ACC_ROWS_CTR         = log2Up(CD_ACC_ROWS+1)

  //==========================================================================
  // scratchpad/accumulator bank row widths
  //==========================================================================
  def A_SP_ROW_ELEMS          = SQRT_FG_NUM * FG_DIM
  def A_SP_ROW_ELEMS_IDX      = log2Up(A_SP_ROW_ELEMS)
  def A_SP_ROW_ELEMS_CTR      = log2Up(A_SP_ROW_ELEMS+1)

  def B_SP_ROW_ELEMS          = FG_NUM * FG_DIM
  def B_SP_ROW_ELEMS_IDX      = log2Up(B_SP_ROW_ELEMS)
  def B_SP_ROW_ELEMS_CTR      = log2Up(B_SP_ROW_ELEMS+1)

  def CD_ACC_ROW_ELEMS        = FG_NUM * FG_DIM
  def CD_ACC_ROW_ELEMS_IDX    = log2Up(CD_ACC_ROW_ELEMS)
  def CD_ACC_ROW_ELEMS_CTR    = log2Up(CD_ACC_ROW_ELEMS+1)
  //------------------------------------------------------
  def A_SP_ROW_BITS           = A_SP_ROW_ELEMS * ITYPE_BITS
  def A_SP_ROW_BYTES          = A_SP_ROW_ELEMS * ITYPE_BYTES
  def A_SP_ROW_BYTES_IDX      = log2Up(A_SP_ROW_BYTES)
  def A_SP_ROW_BYTES_CTR      = log2Up(A_SP_ROW_BYTES+1)

  def B_SP_ROW_BITS           = B_SP_ROW_ELEMS * ITYPE_BITS
  def B_SP_ROW_BYTES          = B_SP_ROW_ELEMS * ITYPE_BYTES
  def B_SP_ROW_BYTES_IDX      = log2Up(B_SP_ROW_BYTES)
  def B_SP_ROW_BYTES_CTR      = log2Up(B_SP_ROW_BYTES+1)

  def D_LOAD_ROW_BITS         = CD_ACC_ROW_ELEMS * OTYPE_BITS
  def D_LOAD_ROW_BYTES        = CD_ACC_ROW_ELEMS * OTYPE_BYTES
  def D_LOAD_ROW_BYTES_IDX    = log2Up(D_LOAD_ROW_BYTES)
  def D_LOAD_ROW_BYTES_CTR    = log2Up(D_LOAD_ROW_BYTES+1)

  def C_STORE_ROW_BITS        = CD_ACC_ROW_ELEMS * ITYPE_BITS
  def C_STORE_ROW_BYTES       = CD_ACC_ROW_ELEMS * ITYPE_BYTES
  def C_STORE_ROW_BYTES_IDX   = log2Up(C_STORE_ROW_BYTES)
  def C_STORE_ROW_BYTES_CTR   = log2Up(C_STORE_ROW_BYTES+1)
  //------------------------------------------------------
  def A_SP_FG_COLS            = A_SP_ROW_ELEMS / FG_DIM
  def A_SP_FG_COLS_IDX        = log2Up(A_SP_FG_COLS)
  def A_SP_FG_COLS_CTR        = log2Up(A_SP_FG_COLS+1)

  def B_SP_FG_COLS            = B_SP_ROW_ELEMS / FG_DIM
  def B_SP_FG_COLS_IDX        = log2Up(B_SP_FG_COLS)
  def B_SP_FG_COLS_CTR        = log2Up(B_SP_FG_COLS+1)

  def CD_ACC_FG_COLS          = CD_ACC_ROW_ELEMS / FG_DIM
  def CD_ACC_FG_COLS_IDX      = log2Up(CD_ACC_FG_COLS)
  def CD_ACC_FG_COLS_CTR      = log2Up(CD_ACC_FG_COLS+1)
  //==========================================================================
  // scratchpad/accumulator bank port widths
  //==========================================================================
  def A_SP_PORT_FG_COLS     = 1
  def A_SP_PORT_FG_COLS_IDX = log2Up(A_SP_PORT_FG_COLS)
  def A_SP_PORT_FG_COLS_CTR = log2Up(A_SP_PORT_FG_COLS+1)

  def B_SP_PORT_FG_COLS     = FG_NUM
  def B_SP_PORT_FG_COLS_IDX = log2Up(B_SP_PORT_FG_COLS)
  def B_SP_PORT_FG_COLS_CTR = log2Up(B_SP_PORT_FG_COLS+1)

  def AB_EXEC_PORT_FG_COLS  = FG_NUM
  def AB_EXEC_PORT_BITS     = AB_EXEC_PORT_FG_COLS * FG_DIM * ITYPE_BITS
  //==========================================================================
  // DMA-related constants
  //==========================================================================
  def DMA_REQS          = 16 // max outstanding tilelink reqs per client
  def DMA_REQS_IDX      = log2Up(DMA_REQS)
  def DMA_REQS_CTR      = log2Up(DMA_REQS+1)

  def DMA_BUS_BITS      = dma_beat_bits
  def DMA_BUS_BITS_IDX  = log2Ceil(DMA_BUS_BITS)
  def DMA_BUS_BITS_CTR  = log2Ceil(DMA_BUS_BITS+1)

  def DMA_BUS_BYTES     = DMA_BUS_BITS / 8
  def DMA_BUS_BYTES_IDX = log2Ceil(DMA_BUS_BYTES)
  def DMA_BUS_BYTES_CTR = log2Ceil(DMA_BUS_BYTES+1)

  def DMA_TXN_BYTES         = dma_max_req_bytes // in single TileLink txn
  def DMA_TXN_BYTES_IDX     = log2Ceil(DMA_TXN_BYTES)
  def DMA_TXN_BYTES_CTR     = log2Ceil(DMA_TXN_BYTES+1)
  def DMA_TXN_BYTES_CTR_IDX = log2Ceil(DMA_TXN_BYTES_CTR) // TL-A log2_size

  def DMA_TXN_BEATS     = DMA_TXN_BYTES / DMA_BUS_BYTES
  def DMA_TXN_BEATS_IDX = log2Ceil(DMA_TXN_BEATS)
  def DMA_TXN_BEATS_CTR = log2Ceil(DMA_TXN_BEATS+1)

  //==========================================================================
  // Memory-Operation constants (mem-op === multiple related DMA ops)
  //==========================================================================
  def MEM_OP_QUEUE_LENGTH = mem_op_queue_length

  def MEM_OP_ROWS     = FG_NUM * FG_DIM
  def MEM_OP_ROWS_IDX = log2Up(MEM_OP_ROWS)
  def MEM_OP_ROWS_CTR = log2Up(MEM_OP_ROWS+1)
}

