#include "include/types.h"
#include "include/ram.h"
#include "include/flash.h"
#include "include/xram.h"
#include "include/eeprom.h"
#include "include/binary_ops.h"
#include "include/undefined.h"

//////////////////////////////// FUNCTIONS:
#define WAIT_MACHINE_CYCLES_BY_2(x) /* waits for x*2 machine cycles, x = 0 is 0x100 */

//////////////////////////////// CONSTANTS
#define KNOCK_SENSOR_TEST_CONFIG_WORD ((byte)(0x00))
#define KNOCK_SENSOR_TEST_CONFIG_WORD_PREFIX ((byte)0xE0)
#define KNOCK_SENSOR_INTEGRATOR_TIME_CONSTANT_CONFIG_WORD_PREFIX ((byte)0xC0)
#define KNOCK_SENSOR_BALANCE_CONTROL_CONFIG_WORD_PREFIX ((byte)0x80)
#define KNOCK_SENSOR_REFERENCE_FILTER_FREQUENCY_CONFIG_WORD_PREFIX ((byte)0x40)
#define KNOCK_SENSOR_FILTER_FREQUENCY_CONFIG_WORD_MASK ((byte)0x3F)

#define KNOCK_SENSOR_REFERENCE_FILTER_FREQUENCY_CONFIG_WORD ((byte)0x01)

#define EEPROM_ADDRESS ((byte)0xA0)
#define EEPROM_SELECT_PAGE_BLOCK(addr, block) (((addr) & 0xFD) | (((bl) & 0x01) << 1))
// r = 1 - read, r = 0 - write
#define I2C_READ_ADDRESS(addr, r) (((addr) & 0xFE) | ((r) & 0x01)) 

#define COOLANT_TEMP_PIN ((pin)0x07)
#define INTAKE_AIR_TEMP_PIN ((pin)0x0E)
#define CO_POT_PIN ((pin)0x08)
#define IGNITION_VOLTAGE_PIN ((pin)0x09)
#define THROTTLE_POSITION_PIN ((pin)0x0C)
#define EGO1_PIN ((pin)0x01)
#define EGO2_PIN ((pin)0x02)

#define IGNITION_VOLTAGE_FACTOR BYTE(0x75)

//////////////////////////////// General purpose flags:
/*
PSW.5 (F0) - was previously working?
  0 = ignition was turned on for the first time.
PSW.1 (F1) - general purpose flag, boolean parameter employed in trampoline functions
*/

//////////////////////////////// Pins

enum Pins {
  P0_0 = 1,
  P0_1,
  // and so on and on and on ...
  P9_7
};


//////////////////////////////// Auxiliary functions

inline bool status_watchdog_triggerred() {
  return CHECK_BIT_AT(RAM[0x20], 6);
}

inline bool status_xram_checksum_invalid() {
  return CHECK_BIT_AT(RAM[0x20], 7);
}


inline bool kitting_has_ego_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_IDX(FLASH, KITTING)[0], 1);
}

inline bool kitting_has_additional_ego_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_IDX(FLASH, KITTING)[1], 4)
}

inline bool kitting_has_absorber() {
  return CHECK_BIT_AT(GET_RNG_START_IDX(FLASH, KITTING)[4], 4);
}

inline bool kitting_has_intake_air_temperature_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_IDX(FLASH, KITTING)[0], 3);
}

inline bool kitting_has_co_potentiometer_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_IDX(FLASH, KITTING)[0], 7);
}

inline bool kitting_has_irom() {
  return CHECK_BIT_AT(GET_RNG_START_IDX(FLASH, KITTING)[2], 0);
}

inline bool kitting_has_camshaft_position_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_IDX(FLASH, KITTING)[0], 4);
}

inline bool kitting_camshaft_position_sensor_cross_section_aligned_with_TDC() {
  return CHECK_BIT_AT(GET_RNG_START_IDX(FLASH, KITTING)[0], 5);
}

inline bool kitting_has_knock_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_IDX(FLASH, KITTING)[0], 2);
}

inline bool kitting_should_adapt_throttle_position_sensor() {
  return CHECK_BIT_AT(GET_RNG_START_IDX(FLASH, KITTING)[2], 6);
}

void reset_init(void) {
  disable_interrupts();
  clear_stack();
  reset_interrupt_requests();
  disable_access_to_xram();
  set_clk_out_prescaler(12); // clkout freq = f_osc/12
  select_dptr(0);
}

void init_pins(void) {
/*
  *** SUMMARY ***
  Bidirectional port structure employed.

  PORTS' DIRECTIONS:
  P0 - all in
  P1 - 2, 3 - in; other - out
  P2 - all in
  P3 - 0, 3, 5 - in; other - out
  P4 - 7 - in; others - out
  P5 - 7 - in; others - out
  P6 - all out
  P7 - 
  P8 - 
  P9 - 0, 4, 5 - in; other - out

  PORTS' CONTENTS:
  P1 - 0xFF
  P3 - 0xFF
  P4 - 0xBF
  P5 - 0x9F
  P6 - 0xFF
  P7 - 
  P8 - 
  P9 - 0xB3
*/
}

void init_HIP0045(void) {
  P6 &= 0xBF; // enable HIP0045 by setting P6_6 (!CE @HIP0045) low

  RAM[0x7E] |= 0x4;

  RAM[0x7F] = RAM[0x7E];

  SendOverSPI(RAM[0x7E], P5_5, P5_7, P5_6);

  P6 |= 0x40; // disabke HIP0045 by setting P6_6 high
/*
  HIP0045 configured with byte 0x44 sent in following order: 
  D7I..D1I = 0010 0010

  D7I: out1 = 0 - RTFM
  D6I: out3 = 0
  D5I: out5 = 1 - main relay switch on
  D4I: out7 = 0

  D3I: out0 = 0 & P5.4 = on - tachometer off (?)
  D2I: out2 = 0
  D1I: out4 = 1 - MAF burnout (off ?)
  D0I: out5 = 0
*/
}

void init_HIP9010(void) {
  SET_BIT_IN(ADCON0, 6); // ADCON0.6 = ADCON0.CLK = 1 - clock output enable through P1.6 (CLKOUT = OSCIN @ HIP9010)
  SYSCON &= 0x7F; // clear CLKP bit - CLKOUT freq = f_osc/6
  
  WAIT_MACHINE_CYCLES_BY_2(0x33);
  WAIT_MACHINE_CYCLES_BY_2(0);
  WAIT_MACHINE_CYCLES_BY_2(0);
  WAIT_MACHINE_CYCLES_BY_2(0);
  WAIT_MACHINE_CYCLES_BY_2(0);
  WAIT_MACHINE_CYCLES_BY_2(0);

#define turn_on_hip9010 P9 &= 0x7F /* enable SPI @ HIP9010 by setting P9.7 low = !CS @ HIP9010 */
#define turn_off_hip9010 P9 |= 0x80 /* disable SPI @ HIP9010 by setting P9.7 high = !CS @ HIP9010 */
/*  
  send 0x2C over SPI to HIP9010/9011 in the following order:
  B7..B0 = 0010 1100

  address = 00 - knock filter frequency
  data = 101100 = 44 (dec) = 8.02 kHz
*/
  turn_on_hip9010; 
  SendOverSPI(FLASH[0x87A7] & KNOCK_SENSOR_FILTER_FREQUENCY_CONFIG_WORD_MASK, P1_5, NULL, P9_6);
  turn_off_hip9010;
  
  WAIT_MACHINE_CYCLES_BY_2(0x0E);
  
/*
  send 0x41 over SPI to HIP9010/9011 in the following order:
  0100 0001

  address = 01 - reference filter frequency
  data = 000001 = 1 = 1.26 kHz
*/
  turn_on_hip9010;
  SendOverSPI(KNOCK_SENSOR_REFERENCE_FILTER_FREQUENCY_CONFIG_WORD | KNOCK_SENSOR_REFERENCE_FILTER_FREQUENCY_CONFIG_WORD_PREFIX, P1_5, NULL, P9_6);
  turn_off_hip9010;
  
  WAIT_MACHINE_CYCLES_BY_2(0x0E);

/*
  send 0x93 over SPI to HIP9010/9011 in the following order:
  1001 0011

  address = 10 - balance control (gain ratio)
  data = 010011 = 13(dec) = 0.649
*/
  turn_on_hip9010;
  SendOverSPI(FLASH[0x24EF + (FLASH[0x87A9] & 0x3F)] | KNOCK_SENSOR_BALANCE_CONTROL_CONFIG_WORD_PREFIX, P1_5, NULL, P9_6);
  turn_off_hip9010;

  WAIT_MACHINE_CYCLES_BY_2(0x0E);

/*
  send 0xC0 over SPI to HIP9010/9011 in the following order:
  1100 0000

  address = 110 - integrator time constant
  data = 0 = 40 usec
*/
  turn_on_hip9010;
  SendOverSPI(FLASH[0xAE01 & 0x1F] | KNOCK_SENSOR_INTEGRATOR_TIME_CONSTANT_CONFIG_WORD_PREFIX, P1_5, NULL, P9_6);
  turn_off_hip9010;
  
  WAIT_MACHINE_CYCLES_BY_2(0x0E);

/*
  send 0xE0 over SPI to HIP9010/9011 in the following order:
  1110 0000

  address = 110 - test/channel select/channel attenuate control
  data = 0:
  ta, tb, tc = 0 - output knock rectifier,
  channel 0 selected
  attenuation applied to knock filter
*/
  turn_on_hip9010;
  SendOverSPI(KNOCK_SENSOR_TEST_CONFIG_WORD | KNOCK_SENSOR_TEST_CONFIG_WORD_PREFIX, P1_5, NULL, P9_6);
  turn_off_hip9010;
  
  WAIT_MACHINE_CYCLES_BY_2(0x0E);

#undef turn_off_hip9010
#undef turn_on_hip9010
}

void init_interrupt_priorities(void) {
/*
set interrupt priorities:

interrupt group 5 - 11(bin) - 3 (highest)
interrupt group 4 - 01      - 1
interrupt group 3 - 10      - 2
interrupt group 2 - 10
interrupt group 1 - 10
interrupt group 0 - 00      - 0 (lowest)

IP0.OWDS and IP0.WDTS are cleared.
*/
}

// output: R1:R0 (R0 - low, R1 - high)
word check_sum_xram_fx00_to_f657(void) {
  word Result = 0x0001;
  word Ptr = RNG_START_IDX(XRAM, SUM_OF_EGO_CALIBRATION) ; // current case: both EGO and absorber are present
  
  if (!kitting_has_ego_sensor()) // There is no EGO
    Ptr = 0xF600;
  else if (!kitting_has_absorber()) // There is no absorber
    Ptr = RNG_START_IDX(XRAM, EGO_CALIBRATION);

  do {
    Result += XRAM[Ptr++];
  } while (Ptr != MEM_IDX(XRAM, CHECKSUM));
}

// INPUT - R1:R0
//         R3:R2
//
// OUTPUT - R1:R0 = R1:R0 - R3:R2
//
// R1 - high, R0 - low
// R3 - high, R2 - low
word subtract_word(word A, word B) {
  return A - B; // subtract with borrow, CY bit is set if borrow needed
}

void init_xram(void) {
  word FlashPtr = 0xABF1;
  word XramPtr = 0xF7D5;
  
  for (byte Cnt = 0xF8; Cnt != 0; --Cnt)
    XRAM[XramPtr++] = FLASH[FlashPtr++];

  XramPtr = 0xF675;
  for (word Cnt = 0x160; Cnt != 0; --Cnt)
    XRAM[XramPtr] = 0x00;
  
  XRAM[0xF97E] = 0x00;
  XRAM[0xF972] = 0x00;
  XRAM[0xF973] = 0x00;
  XRAM[0xF974] = 0x00;
}

void nullify_xram(word XramPtr, byte Bytes) {
  do {
    XRAM[XramPtr++] = 0x00;
    --Bytes;
  } while (Bytes != 0);
}

word sum_xram(word XramPtr, byte WordCount) {
  word Sum = 0x0001;

  do {
    Sum += *(word *)(&XRAM[XramPtr++]);
    --WordCount;
  } (WordCount != 0);
  
  return Sum;
}

void mark_eeprom_failure(void) {
  RAM[0x21] |= BIT_AT(1, 0);
}

void read_eeprom_to_xram(void) {
  word XramPtr = 0xFF00;
  byte Offset = 0;
  
  // send slave address + page block selector (0)
  if (!SendOverI2C(0, P3_3, P1_7)) {
    mark_eeprom_failure();
    nullify_xram(0xFF30, 0x10);
  }
  
  // send word address (byte offset in page block)
  if (!SendOverI2C(Offset, P3_3, P1_7)) {
    mark_eeprom_failure();
    nullify_xram(0xFF30, 0x10);
  }
  
  byte Size = 0x42;
  for (int Idx = 0; Idx < Size; ++Idx)
    XRAM[XramPtr++] = ReadOnI2C(P3_3, P1_7);
  
  if (sum_xram(XramPtr - Size, Size >> 1)) {
    mark_eeprom_failure();
  }
  
  Offset += Size;

  //////////////////////////////////////////////////////

  // send slave address + page block selector (0)
  if (!SendOverI2C(0, P3_3, P1_7)) {
    mark_eeprom_failure();
    nullify_xram(0xFF30, 0x10);
  }

  // send word address (byte offset in page block)
  if (!SendOverI2C(Offset, P3_3, P1_7)) {
    mark_eeprom_failure();
  }

  Size = 0x32;
  for (int Idx = 0; Idx < Size; ++Idx)
    XRAM[XramPtr++] = ReadOnI2C(P3_3, P1_7);

  if (sum_xram(XramPtr - Size, Size >> 1)) {
    mark_eeprom_failure();
  }
  
  Offset += Size;
  
  //////////////////////////////////////////////////////

  // send slave address + page block selector (0)
  if (!SendOverI2C(0, P3_3, P1_7)) {
    mark_eeprom_failure();
    nullify_xram(0xFF30, 0x10);
  }

  // send word address (byte offset in page block)
  if (!SendOverI2C(Offset, P3_3, P1_7)) {
    mark_eeprom_failure();
  }

  Size = 0x0C;
  for (int Idx = 0; Idx < Size; ++Idx)
    XRAM[XramPtr++] = ReadOnI2C(P3_3, P1_7);

  if (sum_xram(XramPtr - Size, Size >> 1)) {
    mark_eeprom_failure();
  }
}

void no_eeprom(void) {
  XRAM[0xFF74] = FLASH[0x8069];
  XRAM[0xFF75] = FLASH[0x806A];
  XRAM[0xF770] = FLASH[0x8093];
}

// close idle air intake valve
void close_bypass_air_valve(void) {
  CLEAR_BIT_IN(P1, 0);
  SET_BIT_IN(P1, 1);
}

// open idle air intake valve
void open_bypass_air_valve(void) {
  CLEAR_BIT_IN(P1, 1);
  SET_BIT_IN(P1, 0);
}

// FlashPtr - location of table in FLASH
// Offset - ???
// Factor - ???
// Negate - should table data be XOR'ed with 0x80 (negated?)
// Returns - result:
//           T[N] in high byte and zero in low byte if factor eq nil,
//           factor*(T[N+1] - T[N]) if factor isn't nil
//           N = Offset
// _635D:
// Example inputs:
// A. FlashPtr = 0x831F (coolant temperature table), Negate = false
// A.1. Offset = 0x08, Factor = 0xF4
// A.2. Offset = 0x0B, Factor = 0xF0
// A.3. Offset = 0x0A, Factor = 0xF4
word GetValueFromTableImpl(word FlashPtr, byte Offset, byte Factor, bool Negate) {
  if (!Factor)
    return COMPOSE_WORD(FLASH[FlashPtr + Offset], 0);

  // A.1. TableData = FLASH[0x831F + 0x08 + 1] = 0x44
  // A.2. TableData = FLASH[0x831F + 0x0B + 1] = 0xA2
  // A.3. TableData = FLASH[0x831F + 0x0A + 1] = 0x83
  byte TableData = FLASH[FlashPtr + Offset + 1];

  if (Negate)
    TableData ^= 0x80;

  // A.1. Prod1 = 0x44 * 0xF4
  // A.2. Prod1 = 0xA2 * 0xF0
  // A.3. Prod1 = 0x83 * 0xF4
  word Prod1 = TableData * Factor; // A - low byte of result, B - high byte of result

  // A.1. TableData = FLASH[0x831F + 0x08] = 0x25
  // A.2. TableData = FLASH[0x831F + 0x0B] = 0x83
  // A.3. TableData = FLASH[0x831F + 0x0A] = 0x63
  TableData = FLASH[FlashPtr + Offset];

  if (Negate)
    TableData ^= 0x80;

  // A.1. Prod1 = 0x25 * 0xF4
  // A.2. Prod1 = 0x83 * 0xF0
  // A.3. Prod1 = 0x63 * 0xF4
  word Prod2 = TableData * (-Factor);

  // A.1. Result = (0x25 + 0x44) * 0xF4 = 0x6414
  // A.2. Result = (0x83 + 0xA2) * 0xF0 = 0x12B0
  // A.3. Result = (0x63 + 0x83) * 0xF4 = 0xDB38
  word Result = Prod1 + Prod2;

  if (Negate) {
    Result ^= 0x8000;
    CY = CHECK_BIT_AT(Result, 0x0F);
  }

  return Result;
}

// FlashPtr - location of table in FLASH
// Input - packed offset and factor
//         factor - least significant three bits, will be SHL 5, max value = 0x1F
//         offset - most significant five bits, will be SHR 3, eq xxx0 0000)
// Negate - should table data be XOR'ed with 0x80 (negated?)
// Returns - result:
//           T[N] in high byte and zero in low byte if factor eq nil,
//           factor*(T[N+1] - T[N]) if factor isn't nil
//  6352:
word GetValueFromTable(word FlashPtr, byte Input, bool Negate) {
  byte Offset = (Input & 0xF8) >> 3; // R3 = tripple_rotate_right(R2, 0x1F); // maximum value = 1F
  byte Factor = (Input & 0x07) << 5; // A = tripple_rotate_right(R2, 0xE0); // Factor: xxx0 0000

  return GetValueFromTableImpl(FlashPtr, Offset, Factor, Negate);
}

void init_ram_63(byte TableDataHigh) {
  RAM[0x63] = TableDataHigh;
  byte XramData = XRAM[0xF602];

  if (IS_NEGATIVE(TableDataHigh)) {  // TableDataHigh & 0x80
    if (TableDataHigh >= XramData)
      RAM[0x63] = XramData - 1;

    return;
  }

  if (TableDataHigh < XramData)
    RAM[0x63] = XramData;
}

/*
  Lookup value in the table in FLASH.
  Input:
   - Input - value to look up.
   - FlashPtr - pointer to the first table value.

  Table size = 0x11 bytes.

  Output byte is packed:
   - high nibble = Idx of last value less or equal than Input
   - low nibble = offset of Input in interval Table[Idx]..Table[Idx + 1] with 1/16 stepping
                  Used for interpolation / profiling.
  That is: MSB..LSB: xxxx yyyy,
           xxxx = Idx of last value less or equal than Input;
           yyyy = offset of Input in interval FLASH[FlashPtr + Idx]..FLASH[FlashPtr + Idx + 1]
                  Used for interpolation / profiling.

  Saturation is performed if Input is out of bounds of the Table.
*/
TableEntryT Lookup17ByteTableWithSaturation(byte Input, word FlashPtr) {
  TableEntryT Result;
  byte Idx = 0; // max valid value = 0x0f
  const byte TableLineLength = 0x11;

  for (Idx = 0; (Idx < TableLineLength) && (Input >= FLASH[FlashPtr + Idx]); ++Idx) {
    ;
  }

  if (TableLineLength - 1 = Idx) {
    Result.TE.TableIdx = TableLineLength - 2; // can't be the last item as it is barrier value (sort of overflow?)
    Result.TE.InterpolationFraction = 0x0F;
    assert(0xFF == Result.ByteVal);
    return Result; // 0xFF;  // saturate to the last value in the table
  }


  // CurrentFlashVal is the first value larger than Input.
  // Input is expected to be between previous value and current value.

  if (!Idx) {
    Result.TE.TableIdx = 0x00;
    Result.TE.InterpolationFraction = 0x00;
    assert(0x00 == Result.ByteVal);
    return Result; // 0x00; // saturate to the first value in the table
  }

  byte LargerVal = FLASH[FlashPtr + Idx];

  --Idx; 
  // Max Idx value at the time = 0x0e.

  byte LesserVal = FLASH[FlashPtr + Idx];

  // Input is between LesserVal and LargerVal.

  Result.TE.TableIdx = Idx;
  Result.TE.InterpolationFraction = 0x00;

  if (LesserVal == LargerVal) {
    assert(SWAP_NIBBLES(Idx) == Result.ByteVal);
    return Result; // SWAP_NIBBLES(Idx); // high nibble is the index itself
  }

  byte IntervalWidth = LargerVal - LesserVal;

  byte Quot;

  CLEAR_BIT_IN(RAM[0x27], 2); // we don't need to divide twice

  _62AF: // looks like point of changes for modification FW setup
  do {
    Quot = LOW((0x10 * (Input - LesserVal)) / IntervalWidth);

    if (CHECK_BIT_AT(RAM[0x27], 2))
      CLEAR_BIT_IN(RAM[0x27], 2);
    else
      break;
  } while (1);

  SET_BIT_IN(RAM[0x27], 2);

  Result.TE.InterpolationFraction = Quot;
  assert(SWAP_NIBBLES(Idx) + Quot == Result.TE.InterpolationFraction);

  return Result;
}

word ProfileTableValue(TableEntryT Difference, TableEntryT Template, word FlashPtr, byte TableLineLength, bool Negate) {
  word BasePtr = FlashPtr + Template.TE.TableIdx * TableLineLength + Difference.TE.TableIdx;

  if (!Difference.TE.InterpolationFraction && !Template.TE.InterpolationFraction)
    return COMPOSE_WORD(FLASH[BasePtr], 0x00);

  byte DiffProfFr = Difference.TE.InterpolationFraction;
  byte TplProfFr = Template.TE.InterpolationFraction;
  byte Negation = Negate ? 0x80 : 0x00;
  word Result = ((        DiffProfFr  * TplProfFr)          * (Negation + FLASH[BasePtr + TableLineLength + 1]))
              + (((0x10 - DiffProfFr) * TplProfFr)          * (Negation + FLASH[BasePtr + TableLineLength]))
              + ((        DiffProfFr  * (0x10 - TplProfFr)) * (Negation + FLASH[BasePtr + 1]))
              + (((0x10 - DiffProfFr) * (0x10 - TplProfFr)) * (Negation + FLASH[BasePtr]));

  return COMPOSE_WORD(HIGH(RESULT) - Negation, LOW(Result));
}

void UpdateRam63Impl() {
  const byte TableBase = 0x8AFB;
  const byte TableLineLength = 0x11;

  byte TableLineIdx = RAM[0x4C];
  word TableLinePtr = TableBase + TableLineIdx * TableLineLength;
  byte LookupValue = RAM[0x5D];

  TableEntryT Entry = Lookup17ByteTableWithSaturation(LookupValue, TableLinePtr);
  
  XRAM[0xF779] = Entry.ByteVal;

  word Val = Entry.ByteVal;
  word Val2 = (sword)((sbyte)(XRAM[0xF602]));
  word Sum = Val + Val2;
  byte Result;

/*
  R1_R0 = 00:xx
  R3_R2 = 00:yy
  SUM = 00:zz or 01:zz - carry, CY = 1
  
  R1_R0 = 00:xx
  R3_R2 = FF:yy
  SUM = FF:zz or 00:zz, CY = 1
*/

  if (Sum < 0)
    Result = 0;
  else if (Sum > 0xFF)
    Result = 0xFF
  else
    Result = LOW(Sum);

  RAM[0x63] = Result;
}

void InitRam5d() {
  const byte TableBase = 0x8AFB;
  const byte TableLineLength = 0x11;

  word RamData = (sbyte)(RAM[0x63]);
  word XramData = (sword)((sbyte)(XRAM[0xF602]));

  sword Diff = RamData - XramData;
  
  byte DiffByte;

/*
  R1_R0 = 00:xx
  R3_R2 = 00:yy
  DIFF = 00:zz, CY = 0 or FF:zz - carry, CY = 1
        yy <= xx or yy > xx
  R1_R0 = 00:xx
  R3_R2 = FF:yy
  DIFF = 01:zz, CY = 1 or FF:zz, CY = 1
        yy <= xx or yy > xx

  CHECK_BIT_AT(DIFF, 15) == 1: yy > xx
  CHECK_BIT_AT(DIFF, 15) == 0 && !HIGH(DIFF): yy <= xx
  CHECK_BIT_AT(DIFF, 15) == 0 && HIGH(DIFF): yy <= xx
*/

  if (Diff < 0)
    DiffByte = 0x00;
  else if (Diff > 0xFF)
    DiffByte = 0xFF;
  else
    DiffByte = LOW(Diff);

/*
  At this point Acc contains saturated difference:
   RAM[0x63] - XRAM[0xF602] which saturates to:
   - 0x00 if result is negative
   - 0xFF if overflow happened (i.e. RAM[0x63] >= 0x80 and XRAM[0xF602] is negative)

  If saturation conditions are not met the difference value is preserved.
*/

  XRAM[0xF779] = Result;
  word Profiled;
  {
    TableEntryT Diff, Tpl;
    Diff.ByteVal = DiffByte;
    Tpl.ByteVal = RAM[0x4A];
    Profiled = ProfileTableValue(Diff, Tpl, TableBase, TableLineLength, false);
  }

  RAM[0x5D] = HIGH(Profiled);
}

void UpdateRam63() {
  if (CHECK_BIT_AT(RAM[0x72], 5))
    UpdateRam63Impl();
  else
    InitRam5d();
}


// example inputs:
// 1. 0x831F, 0x8F40 for Coolant Temperature = 8 degrees
// 2. 0x831F, 0xBF00 for Coolant Temperature = 102 degrees
// 3. 0xAF40, 0xBF00 for Coolant Temperature = 71 degrees
word GetAdcValueFromTable(word FlashPtr, word ADCValue) {
  // max value in ADCValue = F0:FF
  // max offset = HIGH(ADCValue) >> 4 = F0 >> 4 = F
  // max Factor = (HIGH(ADCValue) << 4) | (LOW(ADCValue) >> 4) = (F0 << 4) | (C0 >> 4) = 0C
  // max Factor = (HIGH(ADCValue) << 4) | (LOW(ADCValue) >> 4) = (EF << 4) | (C0 >> 4) = FC
  // With non nil factor, offset is incremented.
  // Hence, table size is 0x11

  // 1. Offset = 0x8F >> 4 = 0x08, Factor = (0x8F << 4) | (0x40 >> 4) = 0xF4
  // 2. Offset = 0xBF >> 4 = 0x0B, Factor = (0xBF << 4) | (0x00 >> 4) = 0xF0
  // 3. Offset = 0xAF >> 4 = 0x0A, Factor = (0xAF << 4) | (0x40 >> 4) = 0xF4
  byte Offset = HIGH(ADCValue) >> 4;
  byte Factor = (HIGH(ADCValue) << 4) | (LOW(ADCValue) >> 4);
  return GetValueFromTableImpl(FlashPtr, Offset, Factor, false);
}

// example inputs:
// 1. 0x831F, 0x8F40 for Coolant Temperature = 8 degrees
// 2. 0x831F, 0xBF00 for Coolant Temperature = 102 degrees
// 3. 0xAF40, 0xBF00 for Coolant Temperature = 71 degrees
word GetAdcValueFromTableAndAdjustForCalculus(word FlashPtr, word ADCValue) {
  // 1. Result = (0x25 + 0x44) * 0xF4 = 0x6414
  // 2. Result = (0x83 + 0xA2) * 0xF0 = 0x12B0
  // 3. Result = (0x63 + 0x83) * 0xF4 = 0xDB38

  word Result = GetAdcValueFromTable(FlashPtr, ADCValue);

  // 1. Returned value: 0x5014
  // 2. Returned value: 0x0000
  // 3. Returned value: 0xC738
  if (HIGH(Result) < 0x14)
    return COMPOSE_WORD(0, 0);

  return COMPOSE_WORD(HIGH(Result) - 0x14, LOW(Result));
}

byte AdjustTemperature(word TemperatureTableValue) {
  static const word MAX_SUM = 0xFFFF;
  static const word TO_ADD = WORD(0x50);
  static const word DIVIDER = WORD(0xA0);

  byte Result = 0xFF;
  
  // 1. 0x5014 < MAX_SUM - TO_ADD, true
  // 2. 0x0000 < MAX_SUM - TO_ADD, true
  // 3. 0xC738 < MAX_SUM - TO_ADD, true
  if (TemperatureTableValue <= MAX_SUM - TO_ADD) {
    // 1. TemperatureTableValue = 0x5014 + 0x50 = 0x5064
    // 2. TemperatureTableValue = 0x0000 + 0x50 = 0x0050
    // 3. TemperatureTableValue = 0xC738 + 0x50 = 0xC788
    TemperatureTableValue += TO_ADD;

    word Quot;

    do {
      CLEAR_BIT_IN(RAM[0x27], 2);

      Quot = TemperatureTableValue / DIVIDER;
    } while (CHECK_BIT_AT(RAM[0x27], 2));

    // 1. Quot = 0x5064 / 0xA0 = 0x80
    // 2. Quot = 0x0050 / 0xA0 = 0
    // 3. Quot = 0xC788 / 0xA0 = 0x013F
    if (!HIGH(Quot))
      Result = LOW(Quot);
  }

  // 1. Returned value: 0x80
  // 2. Returned value: 0x00
  // 3. Returned value: 0xFF
  return Result;
}

word MultiplySigned(byte _M1, byte _M2) {
  sbyte M1 = _M1;
  sbyte M2 = _M2;
  if (!CHECK_BIT_AT(M1, 7))
    return WORD(M1) * M2;

  M1 = -M1; // M1 = (~M1) + 1;
  word Result = WORD(M1) * M2;

  M1 = LOW(Result);
  M2 = HIGH(Result);

  M1 = -M1; // M1 = (~M1) + 1;

  if (M1)
    M2 = ~M2;
  else
    M2 = -M2; // M2 = (~M2) + 1;

  return COMPOSE_WORD(M1, M2);
}

void Inputs_Part1() {
  CLEAR_BIT_IN(RAM[0x26], 6);

  // COOLANT TEMPERATURE
  {
    // example 1 - Temperature is 8 degrees Celsius,
    // voltage = 2.801 V ~ 10mV * (273 + 8) = 10mV * 281 = 2810 mV = 2.810 V
    // CoolantTemp should be (10-bit value): 2^10 * 2.801 / 5 = 0x23D = 0010 0011 1101 => 1000 1111 0100 0000 => 0x8F40
    // example 2 - Temperature is 102 degrees Celsius
    // voltage = 3.735 V ~ 10mV * (273 + 102) = 10mV * 375 = 3750 mV = 3.75 V
    // CoolantTemp should be (10-bit value): 2^10 * 3.735 / 5 = 0x2FC = 0010 1111 1100 => 1011 1111 0000 0000 => 0xBF00
    // example 3 - Temperature is 71 degrees Celcius
    // volatage = 3.424 V ~ 10mV * (273 + 71) = 10mV * 344 = 3440 mV = 3.44 V
    // CoolantTemp should be (10-bit value): 2^10 * 3.424 / 5 = 0x2BD = 0010 1011 1101 => 1010 1111 0100 0000 => 0xAF40
    word CoolantTemp = ADC_10bit(COOLANT_TEMP_PIN);
    SET_MEM_BYTE(XRAM, ADC_COOLANT_TEMP, HIGH(CoolantTemp));
    bool CoolantTempNotInLimits = false;

    if (!FLASH[0x805D]) {
      // 0x8F < 0x64 - false
      // 0xBF < 0x64 - false
      // 0xAF < 0x64 - false
      if (HIGH(CoolantTemp) < FLASH[0x8057]) {
        // coolant_temp_less_than_low_limit
        SET_BIT_IN(RAM[0x23], 2);
        CLEAR_BIT_IN(RAM[0x23], 3);

        CoolantTempNotInLimits = true;
      } else
        // 0x8F > 0xF0 - false
        // 0xBF > 0xF0 - false
        // 0xAF > 0xF0 - false
        if (HIGH(CoolantTemp) > FLASH[0x8058]) {
        // coolant_temp_larger_than_high_limit
        CLEAR_BIT_IN(RAM[0x23], 2);
        SET_BIT_IN(RAM[0x23], 3);

        CoolantTempNotInLimits = true;
      }
    }

    word AdjustedCoolantTemp;

    // CoolantTempNotInLimits = false in both examples
    if (!CoolantTempNotInLimits || FLASH[0x805D]) {
      CLEAR_BIT_IN(RAM[0x23], 2);
      CLEAR_BIT_IN(RAM[0x23], 3);

      // 1. AdjustedCoolantTemp: 0x5014
      // 2. AdjustedCoolantTemp: 0x0000
      // 2. AdjustedCoolantTemp: 0xC738
      AdjustedCoolantTemp = GetAdcValueFromTableAndAdjustForCalculus(MEM_IDX(FLASH, COOLANT_TEMPERATURE_TABLE_1), CoolantTemp);
    } else if (CoolantTempNotInLimits) /* CoolantTempNotInLimits && !FLASH[0x805D] */ {
      AdjustedCoolantTemp = COMPOSE_WORD(FLASH[0x8A4B], 0);
    }

    // 1. 0x50
    // 2. 0x00
    // 2. 0xC7
    RAM[0x3A] = HIGH(AdjustedCoolantTemp);
    // 1. 0x80
    // 2. 0x00
    // 3. 0xFF
    RAM[0x3D] = AdjustTemperature(AdjustedCoolantTemp);
  }

  // INTAKE AIR TEMPERATURE
  {
    word IntakeAirTemp = ADC_10bit(INTAKE_AIR_TEMP_PIN);
    SET_MEM_BYTE(XRAM, ADC_INTAKE_AIR_TEMP, HIGH(IntakeAirTemp));

    bool IntakeAirTempOutOfLimits = false;
    bool HasIntakeAirTempSensor = kitting_has_intake_air_temperature_sensor();
    word AdjustedIntakeAirTemp;

    if (HasIntakeAirTempSensor) {
      // Has intake air temperature sensor
      if (!FLASH[0x8060]) {
        if (HIGH(IntakeAirTemp) < GET_MEM_BYTE(FLASH, MIMIMUM_INTAKE_AIR_TEMPERATURE)) {
          // intake air temp below minimum
          SET_BIT_IN(RAM[0x24], 4);
          CLEAR_BIT_IN(RAM[0x24], 5);

          IntakeAirTempOutOfLimits = true;
        } else if (GET_MEM_BYTE(FLASH, MAXIMUM_INTAKE_AIR_TEMPERATURE) < HIGH(IntakeAirTemp)) {
          // intake air temp above minimum
          CLEAR_BIT_IN(RAM[0x24], 4);
          SET_BIT_IN(RAM[0x24], 5);

          IntakeAirTempOutOfLimits = true;
        }
      }

      if (FLASH[0x8060] || !IntakeAirTempOutOfLimits) {
        CLEAR_BIT_IN(RAM[0x24], 4);
        CLEAR_BIT_IN(RAM[0x24], 5);

        // TODO Table length ?
        AdjustedIntakeAirTemp = GetAdcValueFromTableAndAdjustForCalculus(
            MEM_IDX(FLASH, INTAKE_AIR_TEMPERATURE_TABLE), IntakeAirTemp);
      }
    }

    if (!HasIntakeAirTempSensor || IntakeAirTempOutOfLimits)
      AdjustedIntakeAirTemp = COMPOSE_WORD(FLASH[0x8061], 0);

    RAM[0x3B] = HIGH(AdjustedIntakeAirTemp);
    RAM[0x3E] = AdjustTemperature(AdjustedIntakeAirTemp);
  }

  // MODE SELECTION
  {
    if (CHECK_BIT_AT(P9, 5))  // test LO @ MC33199 (ISO9141)
      XRAM[0xF7BE] = 3;
    else
      XRAM[0xF7BE] = 0;
  }

  // CO POTENTIOMETER
  {
    bool COPotNotInLimits = false;
    bool CantInitCOPot = false;
    byte COPot;

    word AdjustedCOPot;

    if (kitting_has_co_potentiometer_sensor()) {
      // There is a CO Potentiometer sensor
      _544A:
      COPot = ADC_8bit(CO_POT_PIN);
      SET_MEM_BYTE(XRAM, ADC_CO_POT, COPot);

      if (COPot < FLASH[0x8067]) {
        SET_BIT_IN(RAM[0x23], 4);
        CLEAR_BIT_IN(RAM[0x23], 5);

        COPotNotInLimits = true;
      } else if (COPot > FLASH[0x8068]) {
        CLEAR_BIT_IN(RAM[0x23], 4);
        SET_BIT_IN(RAM[0x23], 5);

        COPotNotInLimits = true;
      }

      _5478:
      CLEAR_BIT_IN(RAM[0x23], 4);
      CLEAR_BIT_IN(RAM[0x23], 5);

      if (COPot & 0x80)
        COPot = 0xFF;
      else
        COPot *= 2;

      COPot -= 0x50;

      AdjustedCOPot = MultiplySigned(COPot,  0xC0);
    } else if (kitting_has_irom()) {
      // No CO Potentiometer but has an IROM
      CLEAR_BIT_IN(RAM[0x23], 4);
      CLEAR_BIT_IN(RAM[0x23], 5);

      CantInitCOPot = true;
    } else {
      // Neither CO Potentiometer nor IROM is available
      CLEAR_BIT_IN(RAM[0x23], 4);
      CLEAR_BIT_IN(RAM[0x23], 5);

      COPotNotInLimits = true;
    }

    if (COPotNotInLimits)
      AdjustedCOPot = COMPOSE_WORD(FLASH[0x8069], FLASH[0x8069]);

    if (!CHECK_BIT_AT(RAM[0x73], 3)) {
      if (!CantInitCOPot)
        SET_MEM_BYTE(XRAM, ADJUSTED_CO_POT, HIGH(AdjustedCOPot));
      else
        SET_MEM_BYTE(XRAM, ADJUSTED_CO_POT, XRAM[0xFF74]);
    }
  }

  // IGNITION SWITCH VOLTAGE
  {
    byte IgnVoltage = ADC_8bit(IGNITION_VOLTAGE_PIN);
    SET_MEM_BYTE(XRAM, ADC_IGNITION_SWITCH_VOLTAGE, IgnVoltage);

    word MultipliedIgnVoltage = WORD(IgnVoltage) * IGNITION_VOLTAGE_FACTOR;
    byte AdjustedIgnVoltage;

    if (CHECK_BIT_AT(MultipliedIgnVoltage, 0x0F)) // not possible as minimum IgnVoltage for it is 0x0119 which doesn't fit a byte
      AdjustedIgnVoltage = 0xFF;
    else
      AdjustedIgnVoltage = HIGH(MultipliedIgnVoltage * 2);

    if (AdjustedIgnVoltage < FLASH[0x8062])
      SET_BIT_IN(RAM[0x22], 4);
    else
      CLEAR_BIT_IN(RAM[0x22], 4);

    if (FLASH[0x8063] < AdjustedIgnVoltage)
      SET_BIT_IN(RAM[0x22], 5);
    else
      CLEAR_BIT_IN(RAM[0x22], 5);

    RAM[0x3C] = AdjustedIgnVoltage;

    if (AdjustedIgnVoltage < 0x36)
      AdjustedIgnVoltage = 0;
    else
      AdjustedIgnVoltage -= 0x36;

    AdjustedIgnVoltage = HIGH(WORD(AdjustedIgnVoltage) * 0x40);

    if (AdjustedIgnVoltage > 0x1F)
      AdjustedIgnVoltage = 0x1F;

    RAM[0x3F] = AdjustedIgnVoltage;
  }
}

//_2813:
void InitXramF8CD() {
  SET_MEM_BYTE(XRAM, ADJUSTED_COOLANT_TEMP, RAM[0x3A]);

  if ((RAM[0x3A] < FLASH[0x87B7]) || !kitting_has_knock_sensor()) {
    word XramPtr = 0xF8CD;
    for (int Idx = 0; Idx < 0x80; ++Idx)
      XRAM[XramPtr++] = 0;
  } else {
    word XramPtr = 0xF8CD;

    for (int Idx = 0; Idx < 8; ++Idx) {
      word FlashPtr = 0xADF1;

      for (int Idx2 = 0; Idx2 < 0x10; ++Idx2)
        XRAM[XramPtr++] = FLASH[FlashPtr++];
    }
  }
}

// _285C:
void InitXramF500AndF7A4() {
  if (!kitting_has_ego_sensor() || !kitting_has_absorber())
    return;

_286C:
  CLEAR_BIT_IN(RAM[0x2B], 0);

  if (RAM[0x3A] < FLASH[0x8788]) {
// _28B5:
adjusted_coolant_temp_less_than_flash_8788:
    word XramPtr = RNG_START_IDX(XRAM, EGO_CALIBRATION);
    word ResultXramPtr = XramPtr - 0x100;

    for (int Idx = 0; Idx < 0x100; ++Idx, ++XramPtr, ++ResultXramPtr)
      XRAM[ResultXramPtr] = XRAM[XramPtr];

    XRAM[0xF7A4] = FLASH[0x8789];
  } else {
    if (status_watchdog_triggerred() || status_xram_checksum_invalid()) {
      word XramPtr = RNG_START_IDX(XRAM, EGO_CALIBRATION);
      word ResultXramPtr = XramPtr - 0x100;
      word FlashPtr = 0x9A1C;

      for (int Idx = 0; Idx < 0x100; ++Idx, ++XramPtr, ++FlashPtr, ++ResultXramPtr)
        XRAM[ResultXramPtr] = FLASH[FlashPtr] + XRAM[XramPtr];
    }

    XRAM[0xF7A4] = FLASH[0x878A];
  }
}

// _28CD:
void FinishInitPeripherals() {
  // Summary:
  // - External interrupt 0 (P3.2, not connected to anywhere):
  //  - Trigger by level (as opposed to negative edge)
  //  - Cleared external interrup 0 flag
  //  - Enable external interrupt 0
  // - CCU, CC2 (P1.2, INT5, camshaft position sensor):
  //  - capture on write to register CCL2
  // - CCU, CC3 (P1.3, INT6, crankshaft position sensor):
  //  - capture on rising edge at P1.3
  //  - clear external interrupt 6 edge flag
  //  - enable external interrupt 6
  // - Timer1:
  //  - mode: 16 bit timer/counter
  //  - timer/counter is enabled only while INT 1 (P3.3) pin is high (sure!) and TR1 bit is set
  //    P3.3 = SDA @ NM24C04 / AT24C04 (EEPROM, I2C), pulled to +5V through 4.7k resistor
  //  - started from 0x0000
  // - Timer0:
  //  - mode: 16 bit timer/counter
  //  - started from 0xFACB
  // - Watchdog:
  //  - input freq = f_osc/384
  //
  // Also, checks for state of LO @ MC33199 (ISO9141) and jumps to either funny_thing_with_ISO9141 or to FAED_trampoline

  // TCON.IT0 External interrupt 0 level/edge trigger control flag,
  // If IT0 = 0, level triggered external interrupt 0 is selected.
  // If IT0 = 1, negative edge triggered external interrupt 0 is selected.
  // (external interrupt 0 is at P3.2, which isn't connected to anywhere)
  SET_BIT_IN(TCON, 0);
  // TCON.IE0 = 0
  // External interrupt 0 request flag
  // Set by hardware. Cleared by hardware when processor vectors to interrupt routine
  // (if IT0 = 1) or by hardware (if IT0 = 0).
  CLEAR_BIT_IN(TCON, 1);

  // IEN0.EX0 = 1, enable external interrupt 0
  SET_BIT_IN(IEN0, 0);

  // COCAH2 = 1
  // COCAL1 = 1
  // CC2 (P1.2) capture on write to register CCL2.
  // P1.2 - camshaft position sensor
  CCEN |= 0x30;

  // COCAH3 = 0
  // COCAL3 = 1
  // CC3, capture on rising edge at P1.3 (INT6)
  // P1.3 - crankshaft position sensor
  CCEN |= 0x40;

  // IRCON0.IEX6 = 0
  // External interrupt 6 edge flag
  // Set by hardware when external interrupt edge was detected or when a compare
  // event occurred at pin 1.3/INT6/CC3. Cleared by hardware when processor vectors
  // to interrupt routine.
  CLEAR_BIT_IN(IRCON0, 5);

  // IEN1.EX6 = 1
  // External interrupt 6 / capture/compare interrupt 3 enable
  // If EX6 = 0, external interrupt 6 is disabled.
  SET_BIT_IN(IEN1, 5);

  // Timer1:
  // TMOD.GATE = 1
  // TMOD.M0 = 1
  // GATE:
  // Gating control
  // When set, timer/counter 'x' is enabled only while 'INT x' pin is high and 'TRx' control bit is set.
  // When cleared timer 'x' is enabled whenever 'TRx' control bit is set.
  // M0 = 1, M1 = 0 => 16 bit timer/counter
  TMOD |= 0x50;

  // TCON.(!TR1) = 0
  // TR1: Timer 1 run control bit.
  // Set/cleared by software to turn timer/counter 1 on/off.
  CLEAR_BIT_IN(TCON, 6);  // stop timer1

  TL1 = TH1 = 0;
  SET_BIT_IN(TCON, 6);  // start timer1

  // Timer0:
  // TMOD.M0 = 1
  // M0 = 0, M1 = 1 => 16 bit timer/counter
  TMOD |= 0x01;

  CLEAR_BIT_IN(TCON, 4);  // stop timer0
  TL0 = 0xCB;
  TH0 = 0xFA; // init timer0 register with 0xFACB
  CLEAR_BIT_IN(TCON, 5);  // clear timer0 overflow bit
  SET_BIT_IN(TCON, 4);    // start timer0

  // IEN0.ET0 = 1
  // Timer 0 overflow interrupt enable.
  // If ET0 = 0, the timer 0 interrupt is disabled.
  SET_BIT_IN(IEN0, 1);

  RAM[0x35] = 0x14;
  CLEAR_BIT_IN(RAM[0x28], 0);
  XRAM[0xF96D] = 0xF7;
  XRAM[0xF96E] = 0x0A;
  XRAM[0xF96F] = 0;
  XRAM[0xG970] = 0;

  // watchdog:
  // Reload value = 0x78
  // WDTREL.WPSEL = 1, PRSC.WDTP = 1 => watchdog input freq = f_osc/384
  WDTREL = 0xF8;

  // IEN0.(!WDT) = 0
  // Watchdog timer refresh flag
  // Set to initiate a refresh of the watchdog timer. Must be set directly before SWDT is set to prevent an unintentional refresh of the watchdog timer.
  SET_BIT_IN(IEN0, 6);

  // IEN1.(!SWDT) = 0
  // Watchdog timer start flag
  // Set to activate the watchdog timer. When directly set after setting WDT, a watchdog timer refresh is performed.
  SET_BIT_IN(IEN1, 6);
}

/*
INTERRUPTS:
 - reset
 */

void reset_interrupt() noreturn {
  reset_init();
  
  if (PSW.F0) {
    init_pins();
  } else {
    init_pins();
  }

/********************************
  CMEN = 0
  CCEN = 0
  CC4EN = 0
  TCON = 0
  TMOD = 0
  T2CON = 0
  CTCON = 0x40, compare timer freq = f_osc/2
  CT1CON = 0x40, freq = f_osc / 2
  IEN0 = 0 - disable interrupts: all, timer2, serial channel 0, timer1 overflow, external 1, timer0 overflow, external 0
  IEN1 = 0 - disable interrupts: timer2 external reload, external interrupts 2-6, adc
  IEN2 = 0 - disable interrupts: register compare match interrupt, compare timer interrupt, CM0-7 compare match interrupt, serial iface1
  IEN3 = 0 - disable interrupts: compare timer1 overflow, compare timer1 - general capture/compare interrupt.
  PRSC = 0xD5 - T2P1 = 0; T2P0 = 1 - set timer2 freq = f_osc/12 or f_osc/24
                T1P1 = 0; T1P0 = 1 - divider ration = 1:2
                T0P1 = 0; T0P0 = 1 - divider ration = 1:2


  CMSEL
  CAFR
  CRCH CRCL
  COMSETL COMSETL
  SETMSK CLRMSK
  CTRELH CTRELL
  CT1RELH CT1RELL
  TH2 TL2
********************************/

  RAM[0x7E] = 0x40;
  RAM[0x7F] = 0x40;

  P6 &= 0xFE; // hold 5V drop regulator by setting P6_0 (HOLD @ TLE4267) low

  // is it for HIP0045?
  for (R0 = 0x7C; R0 != 0x7E; ++R0)
    RAM[R0] = 0;

  init_HIP0045();
  init_HIP9010();
  
  enable_access_to_xram(); // xram = 0xF400..0xFFFF
  // TCON.IT0 = 1, negative edge triggered external interrupt 0 (external interrupt 0 is at P3.2, which isn't connected to anywhere)
  // TCON.IT1 = 1, negative edge triggered external interrupt 1 (external interrupt 1 is at P3.3 - SDA @ NM24C04 / AT24C04 (EEPROM, I2C))
  TCON |= SET_BIT(0) | SET_BIT(2);
  //set_negative_edge_trigger_ext_int(0); // P3_2
  //set_negative_edge_trigger_ext_int(1); // P3.3
  
  PRSC = 0xE5;  // PRSC = 0xE5 = 1110 0101
                // WDTP = 1 - watchdog freq f_osc/24 or f_osc/384
                // S0P = 1 - serial interface0 baud rate prescaler by 2 is active
                // T2P1 = 1; T2P0 = 0 - timer2 input clock = f_osc/192 (see T2PS and T2PS1)
                // T1P1 = 0; T1P0 = 1 - prescaler divider ratio for timer1 = 1:2
                // T0P1 = 0; T0P0 = 1 - prescaler divider ratio for timer0 = 1:2

  T2CON = 0xE5; // T2CON = 0xE5 = 1110 0101
                // T2PS = 1 - select timer2 frequency
                // I3FR = 1 - external interrupt 3 on positive edge on INT3 (P1.0)
                // I2FR = 1 - external interrupt 2 on positive edge on INT2 (P1.4)
                // T2R1 = 0; T2R0 = 0 - timer2 reload disabled
                // T2CM = 1 - timer2 compare mode 1 selected
                // T2I1 = 0; T2I0 = 1 - timer2 function - timer with selected frequency

  CTCON = 0xC0; // CTCON = 0xC0 = 1100 0000
                // T2PS1 = 1 - select timer2 freq
                // ICR = 0; ICS = 0; CTF = 0 - these bits are set by hw on interrupt
                // CTP = 1; CLK2 = 0; CLK1 = 0; CLK0 = 0 - compare timer input clock at f_osc/2

  //set_watchdog_frequency_divider(12 or 192); // TODO see WPSEL
  //enable_serial0_baud_rate_prescaler();
  
  //set_timer2_input_clock_divider(192); // timer2 input clock freq = f_osc/192
  //disable_timer2_reload();
  //timer2_compare_mode(1);
  //set_timer2_function_timer_with_selected_frequency();

  //set_prescaler_divider_ratio(timer1, 1, 2); // ratio for timer1 = 1:2
  //set_prescaler_divider_ratio(timer0, 1, 2); // ratio for timer0 = 1:2

  //set_positive_edge_trigger_ext_int(3); // P1_0
  //set_positive_edge_trigger_ext_int(2); // P1_4
  
  //set_compare_timer_input_clock_divider(2); // compare timer input clock at f_osc/2
  
  CTRELL = CTRELH = 0x00; // launch compare timer ; compare timer reload register (low and high halves)
  TL2 = TH2 = 0x00; // launch timer2? ; timer2 register
  
  CMSEL = 0x20; // CMSEL = 0x20 = 0010 0000
                // Assign CML5/CMH5 (EGR valve at Port4) registers to the compare timer and compare mode 0 selected.
                // Other CMLx/CMHx (x = 0..4, 6, 7) registers are assigned to compare timer 2 and compare mode 1 selected.
                // CMEN = 0 for the time being
  //assign_registers_to_compare_timer_with_mode0(5); // CMH5/CML5 registers are assigned to compare timer and compare mode 0 selected
  //assign_registers_to_compare_timer2_with_mode1(0, 1, 2, 3, 4, 6, 7); // CMHx/CMLx (x=0..4, 6, 7) registers are assigned to compare timer2 and compare mode 1 selected
  
  PSW.F1 = 0;
  if (IP0.OWDS | IP0.WDTS) {
    PSW.F1 = 1;
  }
  
  init_interrupt_priorities();
  
  // clear ram up to 0x7B (incl) to 0
  for (R0 = 0x01; R0 != 0x7C; ++R0)
    RAM[R0] = 0x00;
  
  RAM[0x27] |= SET_BIT_V(PSW.F0, 3);
  RAM[0x20] |= SET_BIT_V(PSW.F1, 6);

  word R1_R0 = check_sum_xram_fx00_to_f657(); // R0 - low, R1 - high
  word R3_R2 = GET_MEM_WORD(XRAM, CHECKSUM);      // R2 - low, R3 - high
  R1_R0 = subtract_word(R1_R0, R3_R2);
  
  RAM[0x20] |= SET_BIT_V(R0 || R1, 7);
  
  init_xram();

  RAM[0x2D] |= SET_BIT_V(kitting_has_camshaft_position_sensor(), 7); // Has camshaft position sensor
  RAM[0x2E] |= SET_BIT_V(kitting_camshaft_position_sensor_cross_section_aligned_with_TDC(), 0); // Camshaft position sensor cross-section is aligned with TDC
  RAM[0x2E] |= SET_BIT_V(kitting_has_knock_sensor(), 1); // Is there knock sensor
  
  if (CHECK_BIT_AT(kitting_has_irom(), 0)) // Is there IROM?
    read_eeprom_to_xram();
  else
    no_eeprom();
  
  SET_BIT_IN(PCON, 7); // PCON.SMOD = 1, When set, the baud rate of serial interface 0 in modes 1, 2, 3 is doubled
  SET_BIT_IN(ADCON0, 7); // ADCON0.BD = 1, When set, the baud rate of serial interface 0 is derived from a dedicated baud rate generator.
  
  RAM[0x5F] = 0x20;
  RAM[0x60] = 0x03;
  RAM[0x61] = 0x21;
  RAM[0x62] = 0x00;
  
  open_bypass_air_valve();
  
  CCL1 = RAM[0x5F]; CCH1 = RAM[0x60];
  CRCL = CCL1 + 0x28; CRCH = CCH1 + 0x00;
  
  CCEN |= SET_BIT(1) | SET_BIT(3); // Timer2: Compare enabled for CC register 0 (CRC = CRCH:CRCL) and CC register 1 (CC1 = CCH1:CCL1)
                                   // FYI:
                                   //  - Timer2 is in compare mode 1;
                                   //  - freq = f_osc/192;
                                   //  - reload disabled;
                                   //  - input - timer with selected frequency; 
                                   //  - external interrupt 3 on positive edge on INT3 (P1.0);
                                   //  - external interrupt 2 on positive edge on INT2 (P1.4);

  close_bypass_air_valve();
  
  SET_BIT_IN(IEN1, 2);     // External interrupt 3 / capture/compare interrupt 0 enable
  CLEAR_BIT_IN(IEN1, 3);    // External interrupt 4 / capture/compare interrupt 0 enable
  
  // Closing of bypass intake air valve is scheduled at TH2:TL2 = CRC (mode 1 of timer2 operation)
  
  if (status_watchdog_triggerred() || status_xram_checksum_invalid()) {
    XRAM[0xF602] = FLASH[0x8761]; // 0
    XRAM[0xF603] = 0x01;
    
    if (kitting_has_ego_sensor()) {
      word XramPtr = RNG_START_IDX(XRAM, EGO_CALIBRATION);
      word FlashPtr = 0x991C;
      
      for (int Cnt = 0; Cnt < 0x100; ++Cnt)
        XRAM[XramPtr++] = FLASH[FlashPtr++];
    }
    
    XRAM[0xF600] = 0x80;
    
    {
      word XramPtr = 0xF605;
      
      for (byte Cnt = 0; Cnt < 0x53; ++Cnt)
        XRAM[XramPtr++] = 0;
    }
  }

  word TableData = GetValueFromTable(0xA2FD, 0, false);
  init_ram_63(HIGH(TableData));
  
  UpdateRam63();

_2748:
  if (XRAM[0xF603])
    SET_BIT_IN(RAM[0x2A], 3);

  XRAM[0xF7BF] = 3;
  XRAM[0xF68D] = 0;
  XRAM[0xF6B9] = 0xFF
  RAM[0x59] = FLASH[0x8753];
  XRAM[0xF770] = 0;
  XRAM[0xF67F] = 0;
  XRAM[0xF680] = 0;
  XRAM[0xF6D2] = 0xFF;
  XRAM[0xF6D3] = 0xFF;
  RAM[0x57] = RAM[0x58] = FLASH[0x8755];
  XRAM[0xF6FB] = 0xFF;
  XRAM[0xF6FC] = 0xFF;

  SET_MEM_WORD(XRAM, THROTTLE_POSITION_1, COMPOSE_WORD(FLASH[0x807C], 0));
  SET_MEM_WORD(XRAM, THROTTLE_POSITION_2, COMPOSE_WORD(FLASH[0x807C], 0));

  RAM[0x66] = 0;
  RAM[0x67] = FLASH[0x808C];
  XRAM[0xF707] = XRAM[0xF708] = XRAM[0xF709] = XRAM[0xF70A] = XRAM[0xF70B] = XRAM[0xF70C] = XRAM[0xF70D] = XRAM[0xF70E] = 0x80;
  RAM[0x0D] = 0;
  RAM[0x0E] = 0;
  RAM[0x6F] = 0x0C;
  RAM[0x70] = 0;

  if (!CHECK_BIT_AT(RAM[0x73], 2))
    DPTR[0xF7A3] = 0;

  if (!CHECK_BIT_AT(RAM[0x73], 1)) {
    CLEAR_BIT_IN(RAM[0x2E], 5);
    XRAM[0xF7A5] = 0;
  }

  CMEN |= 0x20; // enable CM5 - EGR valve at P4.5
  SET_BIT_IN(RAM[0x2A], 0);
  SET_BIT_IN(RAM[0x2A], 1);
  SET_BIT_IN(RAM[0x29], 3);
  SET_BIT_IN(RAM[0x2B], 7);
  CLEAR_BIT_IN(RAM[0x2B], 2);

_27FE: // call to init_flags_and_values @ _3A54
  CLEAR_BIT_IN(RAM[0x2A], 6);
  CLEAR_BIT_IN(RAM[0x2A], 7);
  CLEAR_BIT_IN(RAM[0x2A], 2);
  RAM[0x64] = 0;

  if (!CHECK_BIT_AT(RAM[0x73], 2))
    XRAM[0xF7A3] = 0;

  if (!CHECK_BIT_AT(RAM[0x73], 1)) {
    CLEAR_BIT_IN(RAM[0x2E], 5);
    XRAM[0xF7A5] = 0;
  }

  XRAM[0xF6F9] = 0;
  XRAM[0xF6FA] = FLASH[0x8755];

_2801:
  for (byte Idx = 0; Idx < 0x20; ++Idx)
    XRAM[0xF94D + Idx] = XRAM[0xF600];

  Inputs_Part1();

  InitXramF8CD();

  InitXramF500AndF7A4();

_28CD:
  FinishInitPeripherals();
  jump check_K_L_Line;
  // ... TODO ...
}

///////////////////////////////////////////////////////////////////////
// TEMPORARY
///////////////////////////////////////////////////////////////////////

inline byte tripple_rotate_right(byte Data, byte Mask) {
  Data = ((Data & 0x0F) << 4) | ((Data & 0xF0) >> 4); // swap nibbles
  Data = (Data << 1) | (Data >> 7);                   // rotate left
  return Data & Mask;
}

check_K_L_Line:
{
  if (status_watchdog_triggerred() || !status_xram_checksum_invalid())
    jump funny_thing_with_ISO9141;

  if (!(P9 & 0x20)) // LO of MC33199 (ISO9141) IS NOT active
    jump funny_thing_with_ISO9141;

  CLEAR_BIT_IN(P3, 1);  // TxD @ MC33199 (ISO9141)

  WAIT_MACHINE_CYCLES_BY_2(0x0E);
  
  if (P9 & 0x20) // LO of MC33199 (ISO9141) IS active
    jump funny_thing_with_ISO9141;

  SET_BIT_IN(P3, 1);  // TxD @ MC33199 (ISO9141)

  WAIT_MACHINE_CYCLES_BY_2(0x0E);
  
  if (!(P9 & 0x20)) // LO of MC33199 (ISO9141) IS NOT active
    jump funny_thing_with_ISO9141;
  
  jump FAED_trampoline; // fail control loop
}

// _2950:
// K/L-line communication?
funny_thing_with_ISO9141:
MAIN_LOOP_TRAMPOLINE:
{
  SET_BIT_IN(P3, 1); // set high on TxD @ MC33199 (ISO9141) (TxD, serial channel0)

  CLEAR_BIT_IN(RAM[0x2F], 0);
  CLEAR_BIT_IN(RAM[0x2F], 1);

  init_xram_for_serial0();

  S0RELH = 0xFF;
  S0RELL = 0xCC; // select serial0 frequency
  S0CON = 0x58; // SM1 = 0
                // SM0 = 1, 8-bit uart, variable baud rate
                // SM20 = 0, disable serial0 multiprocessor communication
                // REN0 = 1, enable receiver
  SET_BIT_IN(IEN0, 4); // enable serial0 interrupt
  SET_BIT_IN(IEN0, 7); // enable interrupts overall

  MAIN_LOOP();
}

inline bool CHECK_AND_CLEAR_BIT(byte RamPtr, bit Bit) {
  bool Ret = false;

  if (CHECK_BIT_AT(RAM[RamPtr], Bit)) {
    /* bit is set */
    /* clear bit atomically */
    for (;;) {
      byte Expected = RAM[RamPtr];
      byte Desired = Expected;
      CLEAR_BIT_IN(Expected, Bit);

      if (CAS(&RAM[RamPtr], Expected, Desired)) {
        Ret = true;
        break;
      }
    }
  } else {
    /* bit is clear */
    break;
  }

  return Ret;
}

#define COPY_BIT(Dst, DstBit, Src, SrcBit)  \
do {                                        \
  if (CHECK_BIT_AT(Src, SrcBit))            \
    SET_BIT_IN(Dst, DstBit);                \
  else                                      \
    CLEAR_BIT_IN(Dst, DstBit);              \
} while (0)

inline void TURN_OFF_IGNITION_COIL_1_4() {
  CLEAR_BIT_IN(P5, 1);
}

inline void TURN_OFF_IGNITION_COIL_2_3() {
  CLEAR_BIT_IN(P5, 0);
}

inline void TURN_OFF_INJECTOR_1() {
  CLEAR_BIT_IN(P4, 0);
}

inline void TURN_OFF_INJECTOR_2() {
  CLEAR_BIT_IN(P4, 1);
}

inline void TURN_OFF_INJECTOR_3() {
  CLEAR_BIT_IN(P4, 2);
}

inline void TURN_OFF_INJECTOR_4() {
  CLEAR_BIT_IN(P4, 3);
}

// _2967:
void MAIN_LOOP() {
  for (;;) {
    if (CHECK_AND_CLEAR_BIT(0x28, 0))
      break;

    // _296A:
    word W = COMPOSE_WORD(XRAM[0xF96E], XRAM[0xF96D]);
    bool Overflow = !W;
    --W;
    XRAM[0xF96E] = HIGH(W);
    XRAM[0xF96D] = LOW(W);

    if (!Overflow)
      continue;

    XRAM[0xF96D] = 0;

    while (!CHECK_BIT_AT(RAM[0x28], 0));
    CLEAR_BIT_IN(RAM[0x28], 0);
    break;
  }

  // _2983:
  // Timer0 overflowed
  
  // IEN0.(!WDT) = 0
  SET_BIT_IN(IEN0, 6);
  SET_BIT_IN(IEN1, 6); // reset and activate watchdog timer

  COPY_BIT(RAM[0x2D], 1, RAM[0x2A], 2);
  COPY_BIT(RAM[0x2D], 2, RAM[0x2A], 7);

  RAM[0x2D] |= SET_BIT_V(kitting_has_camshaft_position_sensor(), 7);
  RAM[0x2E] |= SET_BIT_V(kitting_camshaft_position_sensor_cross_section_aligned_with_TDC(), 0);
  RAM[0x2E] |= SET_BIT_V(kitting_has_knock_sensor(), 1);

  // _29AC:
  if (!CHECK_BIT_AT(RAM[0x73], 5)) {
    // _29AF:
    XRAM[0xF7BC] = 0;
  }

  // _29B4:
  if (CHECK_BIT_AT(RAM[0x28], 5)) {
    // _29B7:
    XRAM[0xF6B9] = 0xFF;
  } else {
    // _29BF:
    if (CHECK_AND_CLEAR_BIT(0x25, 5)) {
      init_xram_f6bb_f6bc_and_ram_48_49_4b_4c_49_4b_4c(false);
      jump _2B19;
    }

    if (XRAM[0xF6B9] != 0xFF)
      ++XRAM[0xF6B9];
  }

  // _29CD:
  if (CHECK_BIT_AT(RAM[0x2A], 0))
    init_xram_f6bb_f6bc_and_ram_48_49_4b_4c_49_4b_4c_49_4b_4c(true);

  if (XRAM[0xF6B9] < 0x05)
    init_xram_f6bb_f6bc_and_ram_48_49_4b_4c_49_4b_4c_49_4b_4c(true);

  // _29D9:
  SET_BIT_IN(RAM[0x2A], 0);
  CLEAR_BIT_IN(RAM[0x25], 1);
  RAM[0x30] = RAM[0x44] = RAM[0x45] = RAM[0x48] = RAM[0x49] = RAM[0x4A] = RAM[0x4B] = RAM[0x4C] = 0
  XRAM[0xF6BA] = XRAM[0xF6BB] = XRAM[0xF6BC] = 0;

  // _29FB:
  // nullify set/clear masks for timer2 @ port5
  SETMSK = 0;
  CLRMSK = 0;

  TURN_OFF_IGNITION_COIL_1_4();
  TURN_OFF_IGNITION_COIL_2_3();

  // Clear IEN2.ECR
  // COMCLR register compare match interrupt enable
  // If ECR = 0, the COMCLR compare match interrupt is disabled.
  IEN2 &= 0xDF;

  CMEN &= 0xFE;
  TURN_OFF_INJECTOR_1();
  CMEN &= 0xFB;
  TURN_OFF_INJECTOR_3();
  CMEN &= 0xF7;
  TURN_OFF_INJECTOR_4();
  CMEN &= 0xFD;
  TURN_OFF_INJECTOR_2();

  // _2A1C:
  jump _2B19;
}

// Returns FLASH[FlashPtr + TableIdx] + HIGH(DiffFactor * (FLASH[FlashPtr + TableIdx + 1] - FLASH[FlashPtr + TableIdx]))
// _62CE:
byte InterpolateTableValue(word FlashPtr, byte TableIdx, byte DiffFactor) {
  const byte Base = FLASH[FlashPtr + TableIdx];
  const byte Diff = FLASH[FlashPtr + TableIdx + 1] - Base;
  const word Offset = WORD(DiffFactor) * Diff;
  return Base + HIGH(Offset);
}

// TODO memory map for memory locations used here
// _2B19 should be called after this sub-proc
void init_xram_f6bb_f6bc_and_ram_48_49_4b_4c_49_4b_4c_49_4b_4c(bool SkipIntro) {
  if (!SkipIntro) {
    if (CHECK_BIT_AT(RAM[0x2A], 0) && (XRAM[0xF6B9] < 2))
      CLEAR_BIT_IN(RAM[0x20], 2);

    // _2A2D:
    XRAM[0xF6B9] = 0;
  }

_2A32:

  CLEAR_BIT_IN(IEN0, 7); // disable all interrupts
  byte Ram44 = RAM[0x44];
  byte Ram45 = RAM[0x45];
  SET_BIT_IN(IEN0, 7); // allow interrupts

  quad Dividend;
  word Divisor;
  quad Quot;
  bool DivisionSkipped = false;
  word QuotW;

  if (Ram44 || Ram45) {
    // _2A7C:
    CLEAR_BIT_IN(RAM[0x27], 2);
    do {
      Dividend = 0x01E84800;
      Divisor = COMPOSE_WORD(Ram45, Ram44);
      Quot = Dividend / Divisor;
    } while (CHECK_AND_CLEAR_BIT(RAM[0x27], 2));
    SET_BIT_IN(RAM[0x27], 2);
  } else {
    CLEAR_BIT_IN(IEN0, 7); // disable all interrupts
    byte Ram30 = RAM[0x30];
    byte Ram1C = RAM[0x1C];
    byte Ram1D = RAM[0x1D];
    SET_BIT_IN(IEN0, 7); // allow interrupts

    if (Ram30 != 4) {
      QuotW = 0;
      DivisionSkipped = true;
    } else {
      // _2A4B:
      CLEAR_BIT_IN(RAM[0x27], 2);
      do {
        Dividend = 0x00082300;
        Divisor = COMPOSE_WORD(Ram1D, Ram1C);
        Quot = Dividend / Divisor;
      } while (CHECK_AND_CLEAR_BIT(RAM[0x27], 2));
      SET_BIT_IN(RAM[0x27], 2);
    }
  }

  if (!DivisionSkipped) {
    A = QUAD_BYTE(Quot, 3) | QUAD_BYTE(Quot, 2);
    if (!A) {
      // _high_word_of_quot_is_zero:
      QuotW = LOW_W(Quot);
    } else {
      // _high_word_of_quot_is_not_zero:
      QuotW = 0xFFFF // saturate
    }
  }

  // _calc_ram_48:
  XRAM[0xF6BB] = LOW(QuotW);
  XRAM[0xF6BC] = HIGH(QuotW);

  {
    quad X = QUAD(QuotW) + QUAD(0x0020);
    byte Ram48Val = 0xFF;

    if (X >= QUAD(0xFFFF)) {
      X = LOW_W(X) << 2;

      if (X >= QUAD(0xFFFF))
        Ram48Val = QUAD_BYTE(X, 0);
    }

    RAM[0x48] = Ram48Val;
  }

  // _ram_48_filled:
  {
    word XramData = COMPOSE_WORD(XRAM[0xF6BC], XRAM[0xF6BB]);
    const byte DiffByte = 0x80;
    const word Diff = WORD(DiffByte);

    byte Ram49 = COMPOSE_WORD(0xFF, 0xFF - DiffByte) < XramData ? 0xFF : HIGH(XramData + Diff);
    RAM[0x49] = Ram49;
  }
  
  XRAM[0xF6BA] = RAM[0x49] < 0x1F ? 0x1F : RAM[0x49];

  RAM[0x4A] = InterpolateTableValue(0x83B0, XRAM[0xF6BC], XRAM[0xF6BB]); // TODO Table size?
  RAM[0x4B] = ((RAM[0x4A] + 4) >> 3) & 0x1F;
  RAM[0x4C] = ((RAM[0x4A] + 8) >> 4); // high nibble

  // continue in _2B19
}

// _5FFB:
word scale10bitADCValue(word V, byte Factor) {
  // calculus: (WORD(HIGH(V)) * Factor) + LOW(WORD(LOW(V)) * Factor)
  return HIGH_W(QUAD(V) * Factor);
}

// _64A4:
void addWordInXRAMWord(word V, word XramPtr) {
  V += COMPOSE_WORD(XRAM[XramPtr + 1], XRAM[XramPtr]);

  XRAM[XramPtr] = LOW(V);
  XRAM[XramPtr + 1] = HIGH(V);
}

// _6498:
void addByteInXRAMWord(byte _V, word XramPtr) {
  word V = COMPOSE_WORD(XRAM[XramPtr + 1], XRAM[XramPtr]);
  V += _V;

  XRAM[XramPtr] = LOW(V);
  XRAM[XramPtr + 1] = HIGH(V);
}

// TODO NAME
_2B19:
{
  if (!CHECK_BIT_AT(RAM[0x28], 6) &&
      (XRAM[0xF679] >= FLASH[0x809B] && RAM[0x49] >= FLASH[0x809A])) {
    // _2B3A
    SET_BIT_IN(RAM[0x28], 6);
  }

  // query_inputs:
  // _2B3C:
  // Coolant temperature sensor
  {
    word CoolantTemp = ADC_10bit(COOLANT_TEMP_PIN);
    SET_MEM_BYTE(XRAM, ADC_COOLANT_TEMP, HIGH(CoolantTemp));

    CoolantTemp = scale10bitADCValue(CoolantTemp, 8);

    addWordInXRAMWord(CoolantTemp, MEM_IDX(XRAM, COOLANT_TEMP_SUM));
  }

  // _2B53:
  // Intake air temperature sensor
  {
    word IntakeAirTemp = ADC_10bit(INTAKE_AIR_TEMP_PIN);
    SET_MEM_BYTE(XRAM, ADC_INTAKE_AIR_TEMP, HIGH(IntakeAirTemp));

    IntakeAirTemp = scale10bitADCValue(IntakeAirTemp, 8);
    addWordInXRAMWord(IntakeAirTemp, MEM_IDX(XRAM, INTAKE_AIR_SUM));
  }

  // _2B6A:
  // CO Potentiometer sensor
  {
    byte COPot = ADC_8bit(CO_POT_PIN);
    SET_MEM_BYTE(XRAM, ADC_CO_POT, COPot);

    addByteInXRAMWord(COPot, MEM_IDX(XRAM, CO_POT_SUM));
  }

  // _2B7A:
  // Ignition switch
  {
    byte Voltage = ADC_8bit(IGNITION_VOLTAGE_PIN);
    SET_MEM_BYTE(XRAM, ADC_IGNITION_SWITCH_VOLTAGE, Voltage);
    addByteInXRAMWord(Voltage, MEM_IDX(XRAM, IGNITION_SW_VOLTAGE_SUM));

    const byte ThresholdVoltage = FLASH[0x8096];
    Voltage = ADC_8bit(IGNITION_VOLTAGE_PIN);

    if (Voltage < 0)
      Voltage = 0;

    // _2B9D:
    word ScaledVoltage = WORD(Voltage) * IGNITION_VOLTAGE_FACTOR;

    if (CHECK_BIT_AT(HIGH(ScaledVoltage), 7))
      Voltage = 0xFF; // staurate
    else
      Voltage = HIGH(ScaledVoltage) << 1;

    if (Voltage < ThresholdVoltage)
      CLEAR_BIT_IN(RAM[0x28], 4);
    else
      SET_BIT_IN(RAM[0x28], 4);
  }
  
  // _2BAF:
  // Throttle Position Sensor
  {
    word ThrottlePosition = ADC_10bit(THROTTLE_POSITION_PIN);
    SET_MEM_BYTE(XRAM, ADC_THROTTLE_POSITION, HIGH(ThrottlePosition));

    ThrottlePosition = scale10bitADCValue(ThrottlePosition, 8);
    addWordInXRAMWord(ThrottlePosition, MEM_IDX(XRAM, THROTTLE_POSITION_SUM));

    // _2BC6:
    if (!XRAM[0xF69D]) {
      // _2BCC:
      SET_MEM_BYTE(XRAM, THROTTLE_POSITION_BYTE_1, HIGH(ThrottlePosition));
      SET_MEM_BYTE(XRAM, THROTTLE_POSITION_BYTE_2, HIGH(ThrottlePosition));
    } else {
      // xram_f69d_not_zero:
      // _2BD7:
      if (GET_MEM_BYTE(XRAM, THROTTLE_POSITION_BYTE_1) < HIGH(ThrottlePosition)) {
        // _2BE6:
        if (GET_MEM_BYTE(XRAM, THROTTLE_POSITION_BYTE_2) < HIGH(ThrottlePosition))
          SET_MEM_BYTE(XRAM, THROTTLE_POSITION_BYTE_2, HIGH(ThrottlePosition));
      } else {
        // _2BDF:
        SET_MEM_BYTE(XRAM, THROTTLE_POSITION_BYTE_1, HIGH(ThrottlePosition));
      }
    }
  }

  // _2BF3:
  addByteInXRAMWord(RAM[0x49], MEM_IDX(XRAM, RAM_49_SUM));

  if (++XRAM[0xF69D] == 0x20) {
    Xram_F69D_eq_20();
  }

  goto _2ED3;
}

inline bool CheckCoolantTemperature(byte XramF6BF, byte Flash805D, byte CoolantTemperature, byte Min, byte Max) {
  bool TemperatureOK = true;

  if (XramF6BF >= Flash805D) {
    if (CoolantTemperature < Min) {
      TemperatureOK = false;
      SET_BIT_IN(RAM[0x23], 2);
      CLEAR_BIT_IN(RAM[0x23], 3);
    } else if (CoolantTemperature > Max) {
      TemperatureOK = false;
      CLEAR_BIT_IN(RAM[0x23], 2);
      SET_BIT_IN(RAM[0x23], 3);
    }
  }

  return TemperatureOK;
}

inline word InitRam3ATemperature(byte *TableRangeStart, word CoolantTemperature, byte Threshold) {
  CLEAR_BIT_IN(RAM[0x23], 2);
  CLEAR_BIT_IN(RAM[0x23], 3);
  word AdjustedCoolantTemp = GetAdcValueFromTableAndAdjustForCalculus(
      TableRangeStart - FLASH, ScaledADCCoolantTemp);
  RAM[0x3A] = HIGH(AdjustedCoolantTemp);
  if (RAM[0x30] >= Threshold)
    SET_BIT_IN(RAM[0x26], 6);
  else
    CLEAR_BIT_IN(RAM[0x26], 6);

  return AdjustedCoolantTemp;
}

// _5FE2:
// M1 is signed
// M2 is unsigned
word MultiplySigned(byte M1, byte M2) {
  if (!CHECK_BIT_AT(M1, 7))
    return M1 * M2;

  M1 = -M1;
  word P = M1 * M2;
  
  M1 = -LOW(P);
  M2 = HIGH(P);

  if (!M1) {
    M2 = -M2;
  } else {
    M2 = ~M2;
  }

  return COMPOSE_WORD(M2, M1);
}

inline bool ShouldAdaptThrottlePositionSensorBasedOnXramValue(byte Value) {
  return (0x0B != Value) && (0x15 != Value) &&
         (0x16 != Value) && (0x17 != Value) &&
         (0x1F != Value) && (0x20 != Value) &&
         (0x29 != Value);
}

// _695C
void ClearXramF69E_0C_Bytes() {
  for (XramPtr = MEM_IDX(XRAM, COOLANT_TEMP_SUM); XramPtr < MEM_IDX(XRAM, COOLANT_TEMP_SUM) + 0x0C; ++XramPtr)
    XRAM[XramPtr] = 0x00;
  XRAM[0xF69D] = 0;
}

// _2C09
void Xram_F69D_eq_20() {
  // Prerequisites:
  // DPTR = 0xF69D
  // XRAM[0xF69D] == 0x20

  XRAM[0xF69D] = 0; // Reset counter

  // Coolant Temperature
  {
    word AdjustedCoolantTemp;

    if (!CHECK_BIT_AT(RAM[0x72], 7)) {
      _2C15:
      ram_72_bit_7_clear:
      word ScaledADCCoolantTemp = GET_MEM_WORD(XRAM, COOLANT_TEMP_SUM); // R1:R0
      bool TemperatureOK = true;
      bool Ram26Bit6Set = CHECK_BIT_AT(RAM[0x26], 6);

      if (Ram26Bit6Set) {
        _2C5F:
        ram_26_bit_6_set:
        if (TemperatureOK = CheckCoolantTemperature(
            XRAM[0xF6BF], FLASH[0x805D], HIGH(ScaledADCCoolantTemp), FLASH[0x8059], FLASH[0x805A])) {
          _2C86:
          xram_F6BF_less_than_flash_805D_or_coolant_temperature_OK_2:
          AdjustedCoolantTemp = InitRam3ATemperature(
              RNG_START_IDX(XRAM, COOLANT_TEMPERATURE_TABLE_2), ScaledADCCoolantTemp, FLASH[0x805B]);
        }
      } else {
        _2C20:
        if (TemperatureOK = CheckCoolantTemperature(
            XRAM[0xF6BF], FLASH[0x805D], HIGH(ScaledADCCoolantTemp), FLASH[0x8057], FLASH[0x8058])) {
          _2C4A:
          xram_F6BF_less_than_flash_805D_or_coolant_temperature_OK:
          AdjustedCoolantTemp = InitRam3ATemperature(
              RNG_START_IDX(XRAM, COOLANT_TEMPERATURE_TABLE_1), ScaledADCCoolantTemp, FLASH[0x805C]);
        }
      }

      if (!TemperatureOK) {
        _2CB1:
        _coolant_error_condition_common:
        RAM[0x3A] = FLASH[0x8A4B + ((XRAM[0xF6BF] >> 4) & 0x0F)];
        AdjustedCoolantTemp = COMPOSE_WORD(RAM[0x3A], 0);
      }
    } else {
      _2C10:
      AdjustedCoolantTemp = COMPOSE_WORD(RAM[0x3A], 0);
    }

    _2CC4:
    temperature_init_done:
    RAM[0x3D] = AdjustTemperature(AdjustedCoolantTemp);
  }

  // Intake Air Temperature
  {
    bool IntakeAirTempOK = true;
    bool HasIntakeAirTemperatureSensor = kitting_has_intake_air_temperature_sensor();
    word AdjustedIntakeAirTemp;

    if (HasIntakeAirTemperatureSensor && !CHECK_BIT_AT(RAM[0x73], 0)) {
      word IntakeAirTemp = GET_MEM_WORD(XRAM, INTAKE_AIR_SUM);

      if (XRAM[0xF6BF] >= FLASH[0x8060]) {
        _2CF2:
        if (HIGH(IntakeAirTemp) < GET_MEM_BYTE(FLASH, MIMIMUM_INTAKE_AIR_TEMPERATURE)) {
          IntakeAirTempOK = false;
          _2D17:
          scaled_intake_air_temperature_less_than_minimum_intake_air_temperature:
          SET_BIT_IN(RAM[0x24], 4);
          CLEAR_BIT_IN(RAM[0x24], 5);
        } else if (GET_MEM_BYTE(FLASH, MAXIMUM_INTAKE_AIR_TEMPERATURE) < HIGH(IntakeAirTemp)) {
          IntakeAirTempOK = false;
          _2D1D:
          scaled_intake_air_temperature_larger_or_equal_than_maximum_intake_air_temperature:
          CLEAR_BIT_IN(RAM[0x24], 4);
          SET_BIT_IN(RAM[0x24], 5);
        }
      }

      if (IntakeAirTempOK) {
        _2D09:
        xram_F6BF_less_than_flash_8060:
        CLEAR_BIT_IN(RAM[0x24], 4);
        CLEAR_BIT_IN(RAM[0x24], 5);
        // R1:R0
        AdjustedIntakeAirTemp = GetAdcValueFromTableAndAdjustForCalculus(
            RNG_START_IDX(FLASH, INTAKE_AIR_TEMPERATURE_TABLE) - FLASH, IntakeAirTemp);
        RAM[0x3B] = HIGH(AdjustedIntakeAirTemp);
      }
    }

    if (!IntakeAirTempOK || !HasIntakeAirTemperatureSensor) {
      _2D21:
      no_intake_air_temperature_sensor:
      RAM[0x3B] = FLASH[0x8061];
      AdjustedIntakeAirTemp = COMPOSE_WORD(RAM[0x3B], 0);
    }

    _2D2A:
    ram_73_bit_0_set:
    RAM[0x3E] = AdjustTemperature(AdjustedIntakeAirTemp);
  }

  // CO Potentiometer
  {
    _2D31:
    if (kitting_has_co_potentiometer_sensor()) {
      goto _2D47;
    } else if (kitting_has_irom()) {
    } else {
      CLEAR_BIT_IN(RAM[0x23], 4);
      CLEAR_BIT_IN(RAM[0x23], 5);
      goto fallback_CO_pot;
    }

    word Product;
    bool HasCOPotSensor = kitting_has_co_potentiometer_sensor();
    bool HasIROM = kitting_has_irom();

    if (HasCOPotSensor) {
      _2D47:
      bool COPotNotInLimits = false;
      word COPot = GET_MEM_WORD(XRAM, CO_POT_SUM);

      if (COPot < 0x1FFF)
        COPot = 0xFFFF;
      else
        COPot <<= 3;

      if (HIGH(COPot) < FLASH[0x8067]) {
        _2D5F:
        SET_BIT_IN(RAM[0x23], 4);
        CLEAR_BIT_IN(RAM[0x23], 5);
        COPotNotInLimits = true;
      }

      _2D65:
      co_pot_not_less_than_minimum_co_pot:

      if (HIGH(COPot) > FLASH[0x8068]) {
        _2D6F:
        CLEAR_BIT_IN(RAM[0x23], 4);
        SET_BIT_IN(RAM[0x23], 5);
        COPotNotInLimits = true;
      }

      _2D75:
      co_pot_less_or_equal_than_maximum_co_pot:

      if (!COPotNotInLimits) {
        _2D75:
        CLEAR_BIT_IN(RAM[0x23], 4);
        CLEAR_BIT_IN(RAM[0x23], 5);

        byte High = HIGH(COPot);
        if (CHECK_BIT_AT(High, 7))
          High = 0xFF;
        else
          High *= 2;

        High -= 0x50;

        Product = MultiplySigned(High, 0xC0);
      } else {
        Product = COMPOSE_WORD(FLASH[0x8069], FLASH[0x8069]);
      }
    } else if (HasIROM) {
      _2D9E:
      pick_CO_pot_from_eeprom:
      CLEAR_BIT_IN(RAM[0x23], 4);
      CLEAR_BIT_IN(RAM[0x23], 5);

      if (!CHECK_BIT_AT(RAM[0x73], 3))
        SET_MEM_BYTE(XRAM, ADJUSTED_CO_POT, XRAM[0xFF74]);
    } else {
      CLEAR_BIT_IN(RAM[0x23], 4);
      CLEAR_BIT_IN(RAM[0x23], 5);
      Product = COMPOSE_WORD(FLASH[0x8069], FLASH[0x8069]);
    }

    _2D91:
    calculate_xram_f681:
    if (HasCOPotSensor || !HasIROM) {
      if (!CHECK_BIT_AT(RAM[0x73], 3))
        SET_MEM_BYTE(XRAM, ADJUSTED_CO_POT, HIGH(Product));
    }

    _2DAF:
    xram_f681_initialized:
    if (!CHECK_BIT_AT(RAM[0x73], 4)) {
      if (HasIROM)
        XRAM[0xF682] = XRAM[0xFF75];
      else
        XRAM[0xF682] = FLASH[0x806A];
    }
  }

  _2DD4:
  xram_f682_initialized:
  // Throttle Position Sensor
  {
    bool ShouldAdaptThrottlePositionSensor = false;
    word Throttle = GET_MEM_WORD(XRAM, THROTTLE_POSITION_2);

    if ((FLASH[0x8080] < GET_MEM_BYTE(XRAM, THROTTLE_POSITION_BYTE_2) - GET_MEM_BYTE(XRAM, THROTTLE_POSITION_BYTE_1)) ||
        (Throttle < GET_MEM_WORD(XRAM, THROTTLE_POSITION_SUM)) &&
        kitting_should_adapt_throttle_position_sensor() &&
        ShouldAdaptThrottlePositionSensorBasedOnXramValue(XRAM[0xF7BC]) &&
        (HIGH(Throttle) > FLASH[0x807C])) {
      _2E4E:
      word Throttle = GET_MEM_WORD(XRAM, THROTTLE_POSITION_2) + COMPOSE_WORD(0, 0x02);
      SET_MEM_WORD(XRAM, THROTTLE_POSITION_2, Throttle);
    } else {
      _2E02:
      Throttle = (GET_MEM_WORD(XRAM, THROTTLE_POSITION_SUM) & 0xC0) + 0x40;
      SET_MEM_WORD(XRAM, THROTTLE_POSITION_1, Throttle);
      SET_MEM_WORD(XRAM, THROTTLE_POSITION_2, Throttle);
    }

    _2E75:
    no_need_to_adapt_throttle_position_sensor:
    Throttle = GET_MEM_WORD(XRAM, THROTTLE_POSITION_SUM);

    if (HIGH(Throttle) < FLASH[0x807A]) {
      _2E99:
      xram_f6a7_less_low_limit:
      SET_BIT_IN(RAM[0x24], 0);
      CLEAR_BIT_IN(RAM[0x24], 1);
    } else if (HIGH(Throttle) > FLASH[0x807B]) {
      _2E9F:
      xram_f6a7_larger_upper_limit:
      CLEAR_BIT_IN(RAM[0x24], 0);
      SET_BIT_IN(RAM[0x24], 1);
    } else {
      _2E93:
      CLEAR_BIT_IN(RAM[0x24], 0);
      CLEAR_BIT_IN(RAM[0x24], 1);
    }

    _2EA3:
    done_checking_xram_f6a7:
    word Ram49Sum = GET_MEM_WORD(XRAM, RAM_49_SUM);
    word Ram49SumPrev = GET_MEM_WORD(XRAM, RAM_49_SUM_PREV);
    SET_MEM_WORD(XRAM, RAM_49_SUM_PREV, Ram49Sum);

    if (Ram49Sum < Ram49SumPrev)
      Ram49Sum = 0;
    else {
      Ram49Sum -= Ram49SumPrev;

      if (HIGH(Ram49Sum))
        Ram49Sum = COMPOSE_WORD(HIGH(Ram49Sum), 0xFF);
    }

    XRAM[0xF6AC] = LOW(Ram49Sum);
    ClearXramF69E_0C_Bytes();
    SET_BIT_IN(RAM[0x28], 3);
  }
}

// _696B
void ClearXram_F69A_F69B() {
  XRAM[0xF69A] = 0;
  // only zero low byte
  SET_MEM_BYTE(XRAM, IGNITION_SW_VOLTAGE_SUM);
}

inline void ProcessEGO(pin EGOPin, byte *XramDiffSum,
                       byte *RamPtrFlashValFlag, byte RamPtrFlashValFlagBit,
                       bool KittingHasEgoSensor,
                       byte *RamPtrEgoLessLowLimit, byte RamPtrEgoLessLowLimitBit,
                       byte *RamPtrEgoLargerUpperLimit, byte RamPtrEgoLargerUpperLimitBit) {
  byte EGO = ADC_8bit(EGOPin);

  word EgoW = EGO * 0xFF;

  if (HIGH(EgoW) >= 0x40)
    EGO = 0xFF;
  else
    EGO = HIGH(EgoW) << 2;

  byte Diff;

  if (*XramDiffSum < EGO)
    Diff = - HIGH(FLASH[0x8089] * (EGO - *XramDiffSum));
  else
    Diff = HIGH(FLASH[0x8089] * (*XramDiffSum - EGO));

  byte FlashValue;

  if (CHECK_BIT_AT(*RamPtrFlashValFlag, RamPtrFlashValFlagBit))
    FlashValue = FLASH[0x808B];
  else
    FlashValue = FLASH[0x808A];

  if (EGO < FlashValue)
    CLEAR_BIT_IN(*RamPtrFlashValFlag, RamPtrFlashValFlagBit);
  else
    SET_BIT_IN(*RamPtrFlashValFlag, RamPtrFlashValFlagBit);

  if (KittingHasEgoSensor) {
    if (RAM[0x3A] >= FLASH[0x8781]) {
      if (EGO < FLASH[0x8087])
        SET_BIT_IN(*RamPtrEgoLessLowLimit, RamPtrEgoLessLowLimitBit);
      else
        CLEAR_BIT_IN(*RamPtrEgoLessLowLimit, RamPtrEgoLessLowLimitBit);

      if (FLASH[0x8088] < EGO)
        SET_BIT_IN(*RamPtrEgoLargerUpperLimit, RamPtrEgoLargerUpperLimitBit);
      else
        CLEAR_BIT_IN(*RamPtrEgoLargerUpperLimit, RamPtrEgoLargerUpperLimitBit);
    }
  }
}

// _6060:
word AbsWordByMSB(word V) {
  if (CHECK_BIT_AT(V, 15))
    V = COMPOSE_WORD(0 - HIGH(V) - (!!(LOW(V) != 0)), 0 - LOW(V));

  return V;
}

_2ED3:
{
  if (++XRAM[0xF69A] == 4) {
    _2EDC:
    XRAM[0xF69A] = 0; // Reset counter
    word IgnVoltage = GET_MEM_WORD(XROM, IGNITION_SW_VOLTAGE_SUM);
    IgnVoltage >>= 2;
    IgnVoltage = LOW(IgnVoltage) * 0x75;
    byte IgnVoltageByte;

    if (CHECK_BIT_AT(IgnVoltage, 15))
      IgnVoltageByte = 0xFF;
    else
      IgnVoltageByte = HIGH(IgnVoltage) << 1;

    _2EFC:
    if (IgnVoltageByte < FLASH[0x8062])
      SET_BIT_IN(RAM[0x22], 4);
    else
      CLEAR_BIT_IN(RAM[0x22], 4);

    if (IgnVoltageByte > FLASH[0x8063])
      SET_BIT_IN(RAM[0x22], 5);
    else
      CLEAR_BIT_IN(RAM[0x22], 5);

    RAM[0x3C] = IgnVoltageByte;

    if (IgnVoltageByte < 0x36)
      IgnVoltageByte = 0;
    else
      IgnVoltageByte -= 0x36;

    _2F1F:
    IgnVoltage = IgnVoltageByte * 0x40;
    IgnVoltageByte = HIGH(IgnVoltage);
    if (IgnVoltageByte > 0x1F)
      IgnVoltageByte = 0x1F;

    _2F2C:
    RAM[0x3F] = IgnVoltageByte;
    ClearXram_F69A_F69B();
  } // if (++XRAM[0xF69A] == 4)

  _2F31:
  check_L_line:
  {
    // MODE SELECTION
    {
      if (CHECK_BIT_AT(P9, 5))  // test LO @ MC33199 (ISO9141)
        XRAM[0xF7BE] = 3;
      else
        XRAM[0xF7BE] = 0;
    }
  }

  _2F41:
  // EGO #1 Sensor (8bit ADC)
  {
    ProcessEGO(EGO1_PIN, &XRAM[0xF68B], &RAM[0x2B], 4, kitting_has_ego_sensor(),
               &RAM[0x22], 6, &RAM[0x22], 7);
  }

  _2FD0:
  ego2_sensor_start:
  // EGO #2 Sensor (8bit ADC)
  {
    ProcessEGO(EGO2_PIN, &XRAM[0xF68C], &RAM[0x2B], 5,
               kitting_has_ego_sensor() && kitting_has_additional_ego_sensor(),
               &RAM[0x23], 0, &RAM[0x23], 1);
  }

  _3067:
  no_additional_ego_sensor:
  {
    if (kitting_has_additional_ego_sensor()) {
      _306F:
      if (CHECK_BIT_AT(RAM[0x2B], 4)) {
        _3079:
        if (CHECK_BIT_AT(RAM[0x2B], 5))
          CLEAR_BIT_IN(RAM[0x2E], 2);
      } else {
        _3072:
        if (!CHECK_BIT_AT(RAM[0x2B], 5))
          SET_BIT_IN(RAM[0x2E], 2);
      }
    } else {
      _3080:
      no_additional_ego_sensor_2:
      COPY_BIT(RAM[0x2B], 4, RAM[0x2E], 2);
    }
  }

  _3084:
  start_egr_blow: // Oh rly?
  if (CHECK_BIT_AT(RAM[0x24], 0) || CHECK_BIT_AT(RAM[0x24], 1)) {
    _30EF:
    throttle_position_out_of_limits:
    byte Ram40Val;

    if (CHECK_BIT_AT(RAM[0x2A], 1)) {
      _30FA:
      Ram40Val = 0;
    } else {
      _30F2:
      Ram40Val = FLASH[0x931C + RAM[0x4B]];
    }

    _30FB:
    RAM[0x40] = Ram40Val;

    SET_MEM_WORD(XRAM, THROTTLE_POSITION_LESS_THRESHOLD, COMPOSE_WORD(Ram40Val, 0));
    SET_MEM_WORD(XRAM, THROTTLE_POSITION_THRESHOLD, COMPONSE_WORD(Ram40Val, 0));

    SET_MEM_WORD(XRAM, THROTTLE_POSITION_1, COMPOSE_WORD(FLASH[0x807C], 0));
    SET_MEM_WORD(XRAM, THROTTLE_POSITION_2, COMPOSE_WORD(FLASH[0x807C], 0));
  } else {
    _308A:
    word Throttle = ADC_10bit(THROTTLE_POSITION_PIN);

    if (Throttle < GET_MEM_WORD(XRAM, THROTTLE_POSITION_1))
      goto throttle_position_less_than_threshold; // => _30B8

    if (Throttle >= GET_MEM_WORD(XRAM, THROTTLE_POSITION_1)) {
      _309D:
      Throttle = scale10bitADCValue(Throttle, FLASH[0x807D]);

      if (HIGH(Throttle) == FLASH[0x807E]) {
        _30B4:
        scaled_throttle_position_eq_flash_807e:
        Throttle = COMPOSE_WORD(0, 0);
      } else if (HIGH(Throttle) > FLASH[0x807E]) {
        _30B2:
        Throttle = COMPOSE_WORD(FLASH[0x807E], 0);
      } /* else if (HIGH(Throttle) < FLASH[0x807E]) {
        goto scaled_throttle_position_less_flash_807e; // => _30BC
      } */
    } else {
      _30B8:
      throttle_position_less_than_threshold:
      Throttle = COMPOSE_WORD(0, 0);
    }

    _30BC:
    scaled_throttle_position_less_flash_807e:
    word Throttle2 = GET_MEM_WORD(XRAM, THROTTLE_POSITION_THRESHOLD) - Throttle;
    Throttle2 = AbsWordByMSB(Throttle2);

    if (HIGH(Throttle2) < FLASH[0x8081]) {
      _30DB:
      SET_MEM_WORD(XRAM, THROTTLE_POSITION_LESS_THRESHOLD, Throttle);
      RAM[0x40] = HIGH(Throttle);
    }

    _30E5:
    SET_MEM_WORD(XRAM, THROTTLE_POSITION_THRESHOLD, Throttle);
  }

  _3124:
  // TODO
}

// _C000:
void init_xram_for_serial0() {
  if (CHECK_BIT_AT(RAM[0x2F], 0)) {
    _C006:
    if (CHECK_BIT_AT(RAM[0x2F], 1)) {
      _C07A:
      XRAM[0xF983] = 0xFF;
      XRAM[0xF984] = 0xFF;

      XRAM[0xF989] = 0x02;
      XRAM[0xF98A] = 0;
      XRAM[0xF98B] = 0x14;
      XRAM[0xF98C] = 0;

      XRAM[0xF98D] = 0x14;
      XRAM[0xF98E] = 0;
      XRAM[0xF98F] = 0x88;
      XRAM[0xF990] = 0x13;

      XRAM[0xF991] = 0;
      XRAM[0xF992] = 0;
      XRAM[0xF993] = 0xC8;
      XRAM[0xF994] = 0;

      XRAM[0xF995] = 0;
      XRAM[0xF996] = 0;
      XRAM[0xF997] = 0x14;
      XRAM[0xF998] = 0;

      XRAM[0xF985] = 0x0A;
      XRAM[0xF986] = 0;
      XRAM[0xF987] = 0x02;
      XRAM[0xF988] = 0;
    } else {
      _C009:
      XRAM[0xF981] = 0x17;
      XRAM[0xF982] = 0;
      XRAM[0xF983] = 0x1B;
      XRAM[0xF984] = 0;

      XRAM[0xF989] = 0;
      XRAM[0xF98A] = 0;
      XRAM[0xF98B] = 0x13;
      XRAM[0xF98C] = 0;

      XRAM[0xF991] = 0x13;
      XRAM[0xF992] = 0;
      XRAM[0xF993] = 0x89;
      XRAM[0xF994] = 0x13;

      XRAM[0xF995] = 0x4;
      XRAM[0xF996] = 0;
      XRAM[0xF997] = 0x15;
      XRAM[0xF998] = 0;

      XRAM[0xF98D] = 0x1A;
      XRAM[0xF98E] = 0;
      XRAM[0xF98F] = 0x31;
      XRAM[0xF990] = 0;

      XRAM[0xF987] = 0;
      XRAM[0xF988] = 0;
    }
  } else {
    _C0EB:
    if (CHECK_BIT_AT(RAM[0x2F], 1)) {
      _C15E:
      XRAM[0xF983] = 0xFF;
      XRAM[0xF984] = 0xFF;

      XRAM[0xF989] = 0x02;
      XRAM[0xF98A] = 0;
      XRAM[0xF98B] = 0x14;
      XRAM[0xF98C] = 0;

      XRAM[0xF98D] = 0xC8;
      XRAM[0xF98E] = 0;
      XRAM[0xF98F] = 0x88;
      XRAM[0xF990] = 0x13;

      XRAM[0xF991] = 0x02;
      XRAM[0xF992] = 0;
      XRAM[0xF993] = 0xC8;
      XRAM[0xF994] = 0;

      XRAM[0xF995] = 0;
      XRAM[0xF996] = 0;
      XRAM[0xF997] = 0x14;
      XRAM[0xF998] = 0;

      XRAM[0xF985] = 0x0A;
      XRAM[0xF986] = 0;
      XRAM[0xF987] = 0x02;
      XRAM[0xF988] = 0;
    } else {
      _C0EE:
      XRAM[0xF981] = 0x17;
      XRAM[0xF982] = 0;
      XRAM[0xF983] = 0x1B;
      XRAM[0xF984] = 0;

      XRAM[0xF989] = 0;
      XRAM[0xF98A] = 0;
      XRAM[0xF98B] = 0x14;
      XRAM[0xF98C] = 0;

      XRAM[0xF991] = 0;
      XRAM[0xF992] = 0;
      XRAM[0xF993] = 0x88;
      XRAM[0xF994] = 0x13;

      XRAM[0xF995] = 0;
      XRAM[0xF996] = 0;
      XRAM[0xF997] = 0x14;
      XRAM[0xF998] = 0;

      XRAM[0xF98D] = 0;
      XRAM[0xF98E] = 0;
      XRAM[0xF98F] = 0xF4;
      XRAM[0xF990] = 0x01;

      XRAM[0xF987] = 0;
      XRAM[0xF988] = 0;
    }
  }

_C1CC:
  CLEAR_BIT_IN(RAM[0x2F], 2);
  CLEAR_BIT_IN(RAM[0x2F], 4);
  CLEAR_BIT_IN(RAM[0x2F], 5);
  CLEAR_BIT_IN(RAM[0x2F], 6);

  XRAM[0xFBB3] = 0;
  XRAM[0xF9A3] = 0;
  XRAM[0xF9A1] = 0;

  if (CHECK_BIT_AT(RAM[0x2F], 1)) {
    _C1F5:
    CLEAR_BIT_IN(IEN0, 4); // disable serial0 interrupt
    XRAM[0xF97F] = 0;
    XRAM[0xF980] = 0;
    RAM[0x77] = 0x04;
  } else {
    _C1E4:
    XRAM[0xF97F] = 0;
    XRAM[0xF980] = 0;
    SET_BIT_IN(IEN0, 4); // enable serial0 interrupt
    RAM[0x77] = 0xFF;
  }
}
