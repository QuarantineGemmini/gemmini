package gemmini

import scala.math.{pow,sqrt}
import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import GemminiISA._

sealed abstract trait GemminiMemCapacity
case class CapacityInKilobytes(kilobytes: Int) extends GemminiMemCapacity
case class CapacityInMatrices(matrices: Int) extends GemminiMemCapacity

case class FgGemminiArrayConfig[T <: Data : Arithmetic](
  fg_tile_rows: Int,  // rows/cols in a fine-grained systolic array
  fg_tile_cols: Int,
  meshRows: Int,
  meshColumns: Int,
  ld_queue_length: Int,
  st_queue_length: Int,
  ex_queue_length: Int,
  rob_entries: Int,
  sp_banks: Int, // TODO support one-bank designs
  sp_capacity: GemminiMemCapacity,
  acc_banks: Int,
  acc_capacity: GemminiMemCapacity,
  shifter_banks: Int,
  dataflow: Dataflow.Value,
  mem_pipeline: Int,
  dma_maxbytes: Int,
  dma_buswidth: Int,
  aligned_to: Int, // TODO we should align to inputType/outputType instead
  inputType: T,
  outputType: T,
  accType: T,
  fg_sa_div: Int,
) {
  val meshCols = meshColumns
  val tileCols = tileColumns

  //==========================================================================
  // sanity check mesh size
  //==========================================================================
  def BLOCK_ROWS = tileRows * meshRows
  def BLOCK_COLS = tileColumns * meshColumns
  require(BLOCK_ROWS == BLOCK_COLS, "BLOCK_ROWS != BLOCK_COLS!")

  def DIM             = BLOCK_ROWS
  def LOG2_DIM        = log2Up(DIM)
  def LOG2_DIM_COUNT  = log2Up(DIM + 1)
  require(DIM >= 2, "the systolic array must have DIM of at least 2")

  //==========================================================================
  // gemmini1 hardware-specific global constants
  //==========================================================================
  def sp_width = DIM * inputType.getWidth
  def sp_bank_entries = sp_capacity match {
    case CapacityInKilobytes(kb) => kb * 1024 * 8 / (sp_banks * sp_width)
    case CapacityInMatrices(ms) => ms * DIM / sp_banks
  }
  def acc_width = DIM * accType.getWidth
  def acc_bank_entries = acc_capacity match {
    case CapacityInKilobytes(kb) => kb * 1024 * 8 / (acc_banks * acc_width)
    case CapacityInMatrices(ms) => ms * DIM / acc_banks
  }

  def local_addr_t = new LocalAddr(sp_banks, sp_bank_entries, 
                                   acc_banks, acc_bank_entries)

  def max_in_flight_reqs    = 16 // TODO calculate this somehow

  def MAX_DMA_REQS            = 16
  def DMA_BUS_BITS            = dma_buswidth
  def DMA_BUS_BYTES           = dma_buswidth / 8
  def MAX_DMA_BYTES           = dma_maxbytes
  def LOG2_MAX_DMA_REQS       = log2Up(MAX_DMA_REQS) // used as index
  def LOG2_DMA_BUS_BITS       = log2Up(DMA_BUS_BITS) // used as index
  def LOG2_DMA_BUS_BYTES      = log2Up(DMA_BUS_BYTES) // used as ???
  def LOG2_MAX_DMA_BYTES      = log2Up(MAX_DMA_BYTES+1) // used as counter

  def mvin_len_bits   = log2Up(((dma_maxbytes / (inputType.getWidth / 8)) max 
                               DIM) + 1)
  def mvin_rows_bits  = log2Up(DIM + 1)
  def mvout_len_bits  = log2Up(DIM + 1)
  def mvout_rows_bits = log2Up(DIM + 1)

  def BANK_IDX_BITS = 16
  def ROW_COUNT_BITS = 16
  def SQ_COL_START_BITS = 12

  // TODO: move this. was originally in Controller.scala
  def tagWidth = 32

  //==========================================================================
  // gemmini2 fine-grained-SA miscellaneous constants
  //==========================================================================
  def fg_sp_width = DIM * inputType.getWidth

  //==========================================================================
  // gemmini2 miscellaneous constants (some redundant with above)
  //==========================================================================
  def ROB_ENTRIES      = rob_entries
  def LOG2_ROB_ENTRIES = log2Up(rob_entries)

  //==========================================================================
  // gemmini2 hardware-specific compile-time global constants
  //==========================================================================

  def ITYPE_BITS       = inputType.getWidth
  def ITYPE_BYTES      = (inputType.getWidth+7) / 8
  def LOG2_ITYPE_BYTES = if(ITYPE_BYTES <= 1) 0 else log2Up(ITYPE_BYTES)

  def OTYPE_BITS       = accType.getWidth
  def LOG2_OTYPE_BITS  = log2Up(OTYPE_BITS)
  def OTYPE_BYTES      = (accType.getWidth+7) / 8
  def LOG2_OTYPE_BYTES = if(OTYPE_BYTES <= 1) 0 else log2Up(OTYPE_BYTES)

  def SP_BANKS        = sp_banks
  def SP_BANK_ROWS    = sp_bank_entries
  def SP_ROWS         = SP_BANKS * SP_BANK_ROWS
  def LOG2_SP_ROWS    = log2Up(SP_ROWS)

  def SP_ROW_BITS        = FG_NUM * FG_DIM * ITYPE_BITS
  def SP_ROW_BYTES       = (SP_ROW_BITS +7 ) / 8
  def ACC_ROW_BITS       = FG_NUM * FG_DIM * OTYPE_BITS
  def ACC_ROW_BYTES      = (ACC_ROW_BITS +7 ) / 8
  def LOG2_SP_ROW_BITS   = log2Up(SP_ROW_BITS)     // used as index
  def LOG2_ACC_ROW_BITS  = log2Up(ACC_ROW_BITS)
  def LOG2_SP_ROW_BYTES  = log2Up(SP_ROW_BYTES+1)  // used as counters
  def LOG2_ACC_ROW_BYTES = log2Up(ACC_ROW_BYTES+1)

  def MAX_MEM_OP_BYTES      = ACC_ROW_BYTES * FG_DIM
  def LOG2_MAX_MEM_OP_BYTES = log2Up(MAX_MEM_OP_BYTES+1)

  def ACC_BANKS       = acc_banks
  def ACC_BANK_ROWS   = acc_bank_entries
  def ACC_ROWS        = ACC_BANKS * ACC_BANK_ROWS
  def LOG2_ACC_ROWS   = log2Up(ACC_ROWS)

  def MAX_TRANSFER_BYTES      = FG_NUM * FG_DIM * OTYPE_BYTES
  def MAX_TRANSFER_BITS       = MAX_TRANSFER_BYTES * 8
  def LOG2_MAX_TRANSFER_BYTES = log2Up(MAX_TRANSFER_BYTES+1) // count

  def MAX_TRANSFER_ELEMS      = FG_NUM * FG_DIM
  def MAX_TRANSFER_ROWS       = FG_NUM * FG_DIM
  def LOG2_MAX_TRANSFER_ROWS  = log2Up(MAX_TRANSFER_ROWS)

  def MNK_BYTES                   = Int.MaxValue / DIM  // TODO: upper bound?
  def LOG2_MNK_BYTES              = log2Up(MNK_BYTES)
  def MNK_BYTES_PER_TILE_ROW      = MNK_BYTES * DIM
  def LOG2_MNK_BYTES_PER_TILE_ROW = log2Up(MNK_BYTES_PER_TILE_ROW)
  def TILE_IDX                    = MNK_BYTES / (DIM / 8)
  def LOG2_TILE_IDX               = log2Up(TILE_IDX)

  // this just makes some internal logic simpler
  require(ACC_ROW_BYTES >= MAX_DMA_BYTES, "ACC_ROW_BYTES < MAX_DMA_BYTES")

  //--------------------------------------------------------------------------
  def I_TILE_BYTE_WIDTH = DIM * ((inputType.getWidth+7) / 8)
  def O_TILE_BYTE_WIDTH = DIM * ((accType.getWidth+7) / 8)
  def I_TILE_BYTE_WIDTH_LOG2 = log2Up(I_TILE_BYTE_WIDTH)
  def O_TILE_BYTE_WIDTH_LOG2 = log2Up(O_TILE_BYTE_WIDTH)
  require(pow(2,I_TILE_BYTE_WIDTH_LOG2) == I_TILE_BYTE_WIDTH,
    s"I_TILE_BYTE_WIDTH is not power of 2: $I_TILE_BYTE_WIDTH")
  require(pow(2,O_TILE_BYTE_WIDTH_LOG2) == O_TILE_BYTE_WIDTH,
    s"O_TILE_BYTE_WIDTH is not power of 2: $O_TILE_BYTE_WIDTH")

  def GBL_B_SP_ROW_ADDR_1 = (SP_BANKS * SP_BANK_ROWS) - 2*DIM
  def GBL_B_SP_ROW_ADDR_2 = (SP_BANKS * SP_BANK_ROWS) - 1*DIM

  def USABLE_SP_TILES = (SP_ROWS / DIM) - 2
  def TOTAL_ACC_TILES = (ACC_ROWS / DIM)
  def SQRT_ACC_TILES = sqrt(TOTAL_ACC_TILES).toInt
  assert(USABLE_SP_TILES >= TOTAL_ACC_TILES, 
    s"SP_TILES($USABLE_SP_TILES) + 2 < ACC_TILES($TOTAL_ACC_TILES)")

  // prioritize sizes that cause the output-group to be further from square
  def OG_HEIGHT_MAP = (1 to TOTAL_ACC_TILES).sortWith((h1, h2) => {
    (h1 - SQRT_ACC_TILES).abs > (h2 - SQRT_ACC_TILES).abs
  })

  def FG_NUM = fs_sa_div
  def FG_DIM = DIM / FG_NUM

  def OG__MAP = (1 to TOTAL_ACC_TILES).sortWith((h1, h2) => {
    (h1 - SQRT_ACC_TILES).abs > (h2 - SQRT_ACC_TILES).abs
  })

  def BYTE_ROWS_PER_TILE = DIM

  //==========================================================================
  // other stuff
  //==========================================================================
  // TODO remove these requirements
  require(isPow2(sp_bank_entries), 
    "each SRAM bank must have a power-of-2 rows, " + 
    "to simplify address calculations") 
  require(sp_bank_entries % DIM == 0, 
    "the number of rows in a bank must be a multiple of " +
    "the dimensions of the systolic array")
  require(acc_bank_entries % DIM == 0, 
    "the number of rows in an accumulator bank must be a " +
    "multiple of the dimensions of the systolic array")

  def generateHeader(guard: String = "GEMMINI_PARAMS_H"): String = {
    // Returns the (min,max) values for a dataType
    def limitsOfDataType(dataType: Data): (String, String) = {
      assert(dataType.getWidth <= 32) // Above 32 bits, we need to append UL to the number, which isn't done yet

      dataType match {
        case dt: UInt => ("0", BigInt(2).pow(dt.getWidth).-(1).toString)
        case dt: SInt => ("-" + BigInt(2).pow(dt.getWidth - 1).toString, BigInt(2).pow(dt.getWidth - 1).-(1).toString)
        case dt: Float =>
          (dt.expWidth, dt.sigWidth) match {
            case (8, 24) => (scala.Float.MinValue.toString, scala.Float.MaxValue.toString)
            case (11, 53) => (scala.Double.MinValue.toString, scala.Double.MaxValue.toString)
            case _ => throw new IllegalArgumentException(s"Only single- and double-precision IEEE754 floating point types are currently supported")
          }
        case _ => throw new IllegalArgumentException(s"Data type $dataType is unknown")
      }
    }

    def c_type(dataType: Data): String = {
      dataType match {
        case dt: UInt => s"uint${dt.getWidth}_t"
        case dt: SInt => s"int${dt.getWidth}_t"
        case dt: Float =>
          (dt.expWidth, dt.sigWidth) match {
            case (8, 24) => "float"
            case (11, 53) => "double"
            case _ => throw new IllegalArgumentException(s"Only single- and double-precision IEEE754 floating point types are currently supported")
          }
        case _ => throw new IllegalArgumentException(s"Data type $dataType is unknown")
      }
    }

    def full_c_type(dataType: Data): String = {
      dataType match {
        case dt: UInt => "uint64_t"
        case dt: SInt => "int64_t"
        case dt: Float => "double"
        case _ => throw new IllegalArgumentException(s"Data type $dataType is unknown")
      }
    }

    assert(tileColumns*meshColumns == tileRows*meshRows)
    assert(Set(8, 16, 32, 64).contains(inputType.getWidth))
    // assert(Set(8, 16, 32, 64).contains(outputType.getWidth))
    assert(Set(8, 16, 32, 64).contains(accType.getWidth))

    val header = new StringBuilder()
    header ++= s"#ifndef $guard\n"
    header ++= s"#define $guard\n\n"

    header ++= s"#include <stdint.h>\n"
    header ++= s"#include <limits.h>\n\n"

    header ++= s"#define DIM ${tileColumns*meshColumns}\n"
    header ++= s"#define ADDR_LEN 32\n"
    header ++= s"#define BANK_NUM $sp_banks\n"
    header ++= s"#define BANK_ROWS $sp_bank_entries\n"
    header ++= s"#define ACC_ROWS ${ACC_ROWS}\n"
    header ++= 
      s"#define ACC_ROWS_SQRT ${sqrt(ACC_ROWS / DIM).toInt}\n"

    val max_bytes = 64
    header ++= s"#define MAX_BYTES $max_bytes\n"

    if (tileColumns*meshColumns*inputType.getWidth/8 <= max_bytes) {
      header ++= s"#define MAX_BLOCK_LEN (MAX_BYTES/(DIM*${inputType.getWidth/8}))\n"
    } else {
      header ++= s"#define MAX_BLOCK_LEN 1\n"
    }

    if (tileColumns*meshColumns*accType.getWidth/8 <= max_bytes) {
      header ++= s"#define MAX_BLOCK_LEN_ACC (MAX_BYTES/(DIM*${accType.getWidth / 8}))\n\n"
    } else {
      header ++= s"#define MAX_BLOCK_LEN_ACC 1\n\n"
    }

    // Datatype of the systolic array
    val limits = limitsOfDataType(inputType)
    header ++= s"typedef ${c_type(inputType)} elem_t;\n"
    header ++= s"elem_t elem_t_max = ${limits._2};\n"
    header ++= s"elem_t elem_t_min = ${limits._1};\n"
    header ++= s"typedef ${c_type(accType)} acc_t;\n"
    header ++= s"typedef ${full_c_type(inputType)} full_t;\n\n"

    if (inputType.isInstanceOf[Float]) {
      header ++= "#define ELEM_T_IS_FLOAT\n\n"
    }

    header ++= s"#define row_align(blocks) __attribute__((aligned(blocks*DIM*sizeof(elem_t))))\n"
    header ++= s"#define row_align_acc(blocks) __attribute__((aligned(blocks*DIM*sizeof(acc_t))))\n\n"

    header ++= s"#endif // $guard"
    header.toString()
  }

  def headerFilePath: String = {
    val chipyard_directory = "./generators/gemmini/software/gemmini-rocc-tests/include"
    val project_template_directory = "./gemmini-rocc-tests/include" // Old root directory; rendered obsolete by Chipyard
    val default_directory = "."

    val in_chipyard = {
      val dir = new java.io.File(chipyard_directory)
      dir.exists() && dir.isDirectory
    }

    val in_project_template = {
      val dir = new java.io.File(project_template_directory)
      dir.exists() && dir.isDirectory
    }

    if (in_chipyard) {
      s"$chipyard_directory/$headerFileName"
    } else if (in_project_template) {
      s"$project_template_directory/$headerFileName"
    } else {
      s"$default_directory/$headerFileName"
    }
  }
}
