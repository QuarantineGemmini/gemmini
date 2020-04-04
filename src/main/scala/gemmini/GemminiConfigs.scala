package gemmini

import chisel3._
import chisel3.util._

sealed abstract trait GemminiMemCapacity
case class CapacityInKilobytes(kilobytes: Int) extends GemminiMemCapacity
case class CapacityInMatrices(matrices: Int) extends GemminiMemCapacity

case class GemminiArrayConfig[T <: Data : Arithmetic](
  tileRows: Int,
  tileColumns: Int,
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
  pe_latency: Int,
  headerFileName: String = "gemmini_params.h"
) {
  //==========================================================================
  // gemmini1 hardware-specific global constants
  //==========================================================================
  val sp_width = meshColumns * tileColumns * inputType.getWidth
  val sp_bank_entries = sp_capacity match {
    case CapacityInKilobytes(kb) => kb * 1024 * 8 / (sp_banks * sp_width)
    case CapacityInMatrices(ms) => ms * meshRows * tileRows / sp_banks
  }
  val acc_width = meshColumns * tileColumns * accType.getWidth
  val acc_bank_entries = acc_capacity match {
    case CapacityInKilobytes(kb) => kb * 1024 * 8 / (acc_banks * acc_width)
    case CapacityInMatrices(ms) => ms * meshRows * tileRows / acc_banks
  }

  val local_addr_t = new LocalAddr(sp_banks, sp_bank_entries, 
                                   acc_banks, acc_bank_entries)

  val max_in_flight_reqs = 16 // TODO calculate this somehow

  val mvin_len_bits = log2Up(((dma_maxbytes / (inputType.getWidth / 8)) max 
                              (meshColumns * tileColumns)) + 1)
  val mvin_rows_bits = log2Up(meshRows * tileRows + 1)
  val mvout_len_bits = log2Up(meshColumns * tileColumns + 1)
  val mvout_rows_bits = log2Up(meshRows * tileRows + 1)

  // TODO: move this. was originally in Controller.scala
  val tagWidth = 32

  //==========================================================================
  // gemmini2 hardware-specific compile-time global constants
  //==========================================================================
  require(tileRows === tileCols, "tileRows != tileCols!")

  val ITYPE_BITS       = inputType.getWidth
  val LOG2_ITYPE_BITS  = log2Up(ITYPE_BITS)
  val ITYPE_BYTES      = (inputType.getWidth+7) / 8
  val LOG2_ITYPE_BYTES = log2Up(ITYPE_BYTES)

  val OTYPE_BITS       = accType.getWidth
  val LOG2_OTYPE_BITS  = log2Up(OTYPE_BITS)
  val OTYPE_BYTES      = (accType.getWidth+7) / 8
  val LOG2_OTYPE_BYTES = log2Up(OTYPE_BYTES)

  val DIM             = tileRows
  val LOG2_DIM        = log2up(tileRows)
  val LOG2_DIM_COUNT  = log2up(tileRows) + 1

  val SP_BANKS        = sp_banks
  val SP_BANK_ROWS    = sp_bank_entries
  val SP_ROWS         = SP_BANKS * SP_BANK_ROWS
  val LOG2_SP_ROWS    = log2up(SP_ROWS)

  val ACC_BANKS       = acc_banks
  val ACC_BANK_ROWS   = acc_bank_entries
  val ACC_ROWS        = ACC_BANKS * ACC_BANK_ROWS
  val LOG2_ACC_ROWS   = log2up(ACC_ROWS)

  val MNK_BYTES                   = 0xffffffff // max M,N,K size in bytes
  val LOG2_MNK_BYTES              = log2up(MNK_BYTES)
  val MNK_BYTES_PER_TILE_ROW      = MNK_BYTES * DIM
  val LOG2_MNK_BYTES_PER_TILE_ROW = log2up(MNK_BYTES_PER_ROW)
  val TILE_IDX                    = MNK_BYTES / (DIM / 8)
  val LOG2_TILE_IDX               = log2up(TILE_IDX)

  //--------------------------------------------------------------------------
  val I_TILE_BYTE_WIDTH = DIM * inputType.getWidth
  val O_TILE_BYTE_WIDTH = DIM * accType.getWidth

  val GBL_B_SP_ROW_ADDR_1 = (BANKS * BANK_ROWS) - 2*DIM
  val GBL_B_SP_ROW_ADDR_2 = (BANKS * BANK_ROWS) - 1*DIM

  val TILE_ROWS_PER_GROUP_TMP = (BANKS * BANK_ROWS / DIM) - 2
  val TILE_COLS_PER_GROUP_TMP = (ACC_ROWS / DIM) / TILE_ROWS_PER_GROUP

  if(TILE_COLS_PER_GROUP_TMP == 0) {
    // NOTE: this happens if accumulator size < scratchpad size. Don't do 
    //       this! your accumulator should be much larger than scratchpad!
    val TILE_ROWS_PER_GROUP = 4
    val TILE_COLS_PER_GROUP = (ACC_ROWS / DIM) / TILE_ROWS_PER_GROUP
  } else {
    val TILE_ROWS_PER_GROUP = TILE_ROWS_PER_GROUP_TMP
    val TILE_COLS_PER_GROUP = TILE_COLS_PER_GROUP_TMP
  }
  val TILE_ROWS_PER_GROUP_M1 = TILE_ROWS_PER_GROUP - 1
  val TILE_COLS_PER_GROUP_M1 = TILE_COLS_PER_GROUP - 1
  require(TILE_ROWS_PER_GROUP > 0, 
    f"TILE_ROWS_PER_GROUP = $TILE_ROWS_PER_GROUP")
  require(TILE_COLS_PER_GROUP > 0, 
    f"TILE_COLS_PER_GROUP = $TILE_COLS_PER_GROUP")


  val BYTE_ROWS_PER_TILE    = DIM
  val I_BYTE_COLS_PER_GROUP = TILE_COLS_PER_GROUP * I_TILE_BYTE_WIDTH
  val O_BYTE_COLS_PER_GROUP = TILE_COLS_PER_GROUP * O_TILE_BYTE_WIDTH
  val I_TILE_BYTE_WIDTH     = I_TILE_BYTE_WIDTH
  val O_TILE_BYTE_WIDTH     = O_TILE_BYTE_WIDTH

  //==========================================================================
  // other stuff
  //==========================================================================
  require(isPow2(sp_bank_entries), "each SRAM bank must have a power-of-2 rows, to simplify address calculations") // TODO remove this requirement
  require(sp_bank_entries % (meshRows * tileRows) == 0, "the number of rows in a bank must be a multiple of the dimensions of the systolic array")
  require(meshColumns * tileColumns == meshRows * tileRows, "the systolic array must be square") // TODO remove this requirement
  require(meshColumns * tileColumns >= 2, "the systolic array must have a dimension of at least 2") // TODO remove this requirement
  require(isPow2(meshColumns * tileColumns), "the systolic array's dimensions must be powers of 2") // TODO remove this requirement
  require(acc_bank_entries % (meshRows * tileRows) == 0, "the number of rows in an accumulator bank must be a multiple of the dimensions of the systolic array")

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
    header ++= s"#define ACC_ROWS ${acc_banks * acc_bank_entries}\n" // TODO add ACC_BANKS as well

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
