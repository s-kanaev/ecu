#include "include/types.hpp"
#include "include/ram.hpp"
#include "include/flash.hpp"
#include "include/xram.hpp"
#include "include/eeprom.hpp"
#include "include/binary_ops.hpp"
#include "include/undefined.hpp"
#include "include/pins.hpp"
#include "include/registers.hpp"
#include <mask_set.hpp>

#include "e591/memory-locations.hpp"
#include "e591/impl.hpp"
#include "e591/inlines.hpp"


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

//////////////////////////////// General purpose flags:
/*
PSW.5 (F0) - was previously working?
  0 = ignition was turned on for the first time.
PSW.1 (F1) - general purpose flag, boolean parameter employed in trampoline functions
*/


//////////////////////////////// Auxiliary functions


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
  } while (WordCount != 0);

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

  if (TableLineLength - 1 == Idx) {
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

  Quot = LOW((0x10 * (Input - LesserVal)) / IntervalWidth);

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

  return COMPOSE_WORD(HIGH(Result) - Negation, LOW(Result));
}

void UpdateRam63Impl() {
  const word TableBase = 0x8AFB;
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
    Result = 0xFF;
  else
    Result = LOW(Sum);

  RAM[0x63] = Result;
}

void InitRam5d() {
  const word TableBase = 0x8AFB;
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

  XRAM[0xF779] = DiffByte;
  word Profiled = ProfileTableValue(
      DiffByte, RAM[0x4A], TableBase, TableLineLength, false);

  RAM[0x5D] = HIGH(Profiled);
}

void UpdateRam63() {
  if (CHECK_BIT_AT(RAM[0x72], 5))
    UpdateRam63Impl();
  else
    InitRam5d();
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
  XRAM[0xF970] = 0;

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

// check_K_L_Line:
void mainLoopSelector() {
  if (status_watchdog_triggerred() ||
      !checkLO_MC33199() ||
      setTxD_and_checkLO_MC33199(false) ||
      !setTxD_and_checkLO_MC33199(true)) {
    mainLoopTrampoline();
  } else {
    FAED_trampoline:
    // TODO
    ;
  }
}


/*
INTERRUPTS:
 - reset
 */

void reset_interrupt() /*noreturn*/ {
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
  XRAM[0xF6B9] = 0xFF;
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
    XRAM[0xF7A3] = 0;

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

  mainLoopSelector();
  //goto check_K_L_Line;
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

inline bool CHECK_AND_CLEAR_BIT(byte *Ptr, bit Bit) {
  bool Ret = false;

  if (CHECK_BIT_AT(*Ptr, Bit)) {
    /* bit is set */
    /* clear bit atomically */
    for (;;) {
      byte Expected = *Ptr;
      byte Desired = Expected;
      CLEAR_BIT_IN(Expected, Bit);

      if (CAS(Ptr, Expected, Desired)) {
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

// _2950:
// K/L-line communication?
// funny_thing_with_ISO9141:
// MAIN_LOOP_TRAMPOLINE:
void mainLoopTrampoline() {
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

// TODO memory map for memory locations used here
// _2B19 should be called after this sub-proc
void init_xram_f6bb_f6bc_and_ram_48_49_4a_4b_4c(bool SkipIntro) {
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
    Dividend = 0x01E84800;
    Divisor = COMPOSE_WORD(Ram45, Ram44);
    Quot = Dividend / Divisor;
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
      Dividend = 0x00082300;
      Divisor = COMPOSE_WORD(Ram1D, Ram1C);
      Quot = Dividend / Divisor;
    }
  }

  if (!DivisionSkipped) {
    if (!QUAD_BYTE(Quot, 3) && !QUAD_BYTE(Quot, 2)) {
      // _high_word_of_quot_is_zero:
      QuotW = LOW_W(Quot);
    } else {
      // _high_word_of_quot_is_not_zero:
      QuotW = 0xFFFF; // saturate
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

  // TODO table length
  RAM[0x4A] = InterpolateTableValue(0x83B0, XRAM[0xF6BC], XRAM[0xF6BB]);
  RAM[0x4B] = ((RAM[0x4A] + 4) >> 3) & 0x1F;
  RAM[0x4C] = ((RAM[0x4A] + 8) >> 4); // high nibble
}

// _2C09
void Xram_F69D_eq_20() {
  // Prerequisites:
  // DPTR = 0xF69D
  // XRAM[0xF69D] == 0x20

  XRAM[0xF69D] = 0; // Reset counter

  // Coolant Temperature
  {
    Xram_F69D_eq_20_CoolantTemperature();
  }

  // Intake Air Temperature
  {
    Xram_F69D_eq_20_IntakeAirTemperature();
  }

  // CO Potentiometer
  {
    Xram_F69D_eq_20_CO_Pot();
  }

  _2DD4:
  xram_f682_initialized:
  // Throttle Position Sensor
  {
    Xram_F69D_eq_20_ThrottlePosition();
  }
}

// _696B
void ClearXram_F69A_F69B() {
  XRAM[0xF69A] = 0;
  // only zero low byte
  XRAM[WORD_MEM_IDX(XRAM, IGNITION_SW_VOLTAGE_SUM)] = 0x00;
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


// _60BA:
word multiply16WithSaturation(word V) {
  if (V & 0xF000)
    V = 0xFFFF;
  else
    V *= 16;
}

// _62FC:
byte tableLookup0(word FlashPtr, OffsetAndFactorT OffAndFactor) {
  return GetValueFromTableImpl(
      FlashPtr, OffAndFactor.Offset, OffAndFactor.Factor, false);
}

struct TwoValuesForThrottle {
  word V1;  // R3:R2
  word V2;  // R1:R0
};

// _5555:
TwoValuesForThrottle getTwoValuesForThrottle(word ScaledThrottlePosition) {
  byte Interpolated = InterpolateTableValue(
      0x856C, HIGH(ScaledThrottlePosition), LOW(ScaledThrottlePosition));
  word Profiled;
  // TODO table length, see RAM[0x3D]
  const byte Factor1 = tableLookup0(0x900B, RAM[0x3D]);
  // TODO table length, see RAM[0x3E]
  const byte Factor2 = tableLookup0(0xB80D, RAM[0x3E]);
  word Scaled;

  TwoValuesForThrottle Result;

  // TableEntryT Difference, TableEntryT Template, word FlashPtr, byte TableLineLength, bool Negate
  {
    // TODO table length (see RAM[0x4A] values)
    Profiled = ProfileTableValue(RAM[0x4A], Interpolated, 0x856C, 0x10, false);
    Scaled = scale10bitADCValue(Profiled, Factor1);
    Scaled = scale10bitADCValue(Scaled, Factor2);
    Scaled >>= 1;
    Result.V1 = Scaled;
  }

  {
    // TODO table length (see RAM[0x4A] values)
    Profiled = ProfileTableValue(RAM[0x4A], Interpolated, 0x8D0B, 0x10, false);
    Scaled = scale10bitADCValue(Profiled, Factor1);
    Scaled = scale10bitADCValue(Scaled, Factor2);
    Scaled >>= 1;
    Result.V2 = Scaled;
  }

  return Result;
}

// _55BE:
word get_flash_8e0b_plus_swapped_ram41_plus_ram4c() {
  word FlashPtr = 0x8E0B + SWAP_NIBBLES(RAM[0x41]) + RAM[0x4C];

  return COMPOSE_WORD(FLASH[FlashPtr + 1], FLASH[FlashPtr]);
}

// _55DE:
word filterThrottlePosition(word ThrottlePositionLessThreshold) {
  word Result;

  // A
  TwoValuesForThrottle V = getTwoValuesForThrottle(ThrottlePositionLessThreshold);

  byte Factor = XRAM[0xF602] + XRAM[0xF779];

  if (IS_NEGATIVE(XRAM[0xF602])) {
    // xram_f602_larger_7f
    // C
    if (Factor <= 0xFF) {
      // xram_f602_larger_7f_sum_xram_f602_and_f779_less_or_equal_ff
      // F
      Result = scale10bitADCValue(V.V2, NEGATE(Factor)) - V.V1 - Factor;
      if (CHECK_BIT_AT(T, 15)) {
        // 5615
        // H
        Result = 0;
      }
    } else {
      // sum_xram_f602_and_f779_less_or_equal_ff
      // copy(G)
      Result = scale10bitADCValue(V.V2, Factor) + V.V1;
    }
  } else {
    // 55F0
    // B
    if ((WORD(XRAM[0xF602]) + XRAM[0xF779]) > 0xFF) {
      // 55F4
      // D
      V.V1 += V.V2;
    }
    // copy(G)
    Result = scale10bitADCValue(V.V2, Factor) + V.V1;
  }
  // 5619
  // J
  Result += get_flash_8e0b_plus_swapped_ram41_plus_ram4c();
  return Result;
}

// _2967:
void MAIN_LOOP() {
  for (;;) {
    if (CHECK_AND_CLEAR_BIT(&RAM[0x28], 0))
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

  bool InitProcDone = false;
  // _29B4:
  if (CHECK_BIT_AT(RAM[0x28], 5)) {
    // _29B7:
    XRAM[0xF6B9] = 0xFF;
  } else {
    // _29BF:
    if (CHECK_AND_CLEAR_BIT(&RAM[0x25], 5)) {
      init_xram_f6bb_f6bc_and_ram_48_49_4a_4b_4c(false);
      InitProcDone = true;
    }

    if (XRAM[0xF6B9] != 0xFF)
      ++XRAM[0xF6B9];
  }

  // _29CD:
  if (CHECK_BIT_AT(RAM[0x2A], 0)) {
    init_xram_f6bb_f6bc_and_ram_48_49_4a_4b_4c(true);
    InitProcDone = true;
  }

  if (XRAM[0xF6B9] < 0x05) {
    init_xram_f6bb_f6bc_and_ram_48_49_4a_4b_4c(true);
    InitProcDone = true;
  }

  if (!InitProcDone) {
    // _29D9:
    SET_BIT_IN(RAM[0x2A], 0);
    CLEAR_BIT_IN(RAM[0x25], 1);
    RAM[0x30] = RAM[0x44] = RAM[0x45] = RAM[0x48] = RAM[0x49] = RAM[0x4A] = RAM[0x4B] = RAM[0x4C] = 0;
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
  }

  // _2A1C: is goto _2B19
  _2B19:
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

    addWordInXRAMWord(CoolantTemp, WORD_MEM_IDX(XRAM, COOLANT_TEMP_SUM));
  }

  // _2B53:
  // Intake air temperature sensor
  {
    word IntakeAirTemp = ADC_10bit(INTAKE_AIR_TEMP_PIN);
    SET_MEM_BYTE(XRAM, ADC_INTAKE_AIR_TEMP, HIGH(IntakeAirTemp));

    IntakeAirTemp = scale10bitADCValue(IntakeAirTemp, 8);
    addWordInXRAMWord(IntakeAirTemp, WORD_MEM_IDX(XRAM, INTAKE_AIR_SUM));
  }

  // _2B6A:
  // CO Potentiometer sensor
  {
    byte COPot = ADC_8bit(CO_POT_PIN);
    SET_MEM_BYTE(XRAM, ADC_CO_POT, COPot);

    addByteInXRAMWord(COPot, WORD_MEM_IDX(XRAM, CO_POT_SUM));
  }

  // _2B7A:
  // Ignition switch
  {
    byte Voltage = ADC_8bit(IGNITION_VOLTAGE_PIN);
    SET_MEM_BYTE(XRAM, ADC_IGNITION_SWITCH_VOLTAGE, Voltage);
    addByteInXRAMWord(Voltage, WORD_MEM_IDX(XRAM, IGNITION_SW_VOLTAGE_SUM));

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
    addWordInXRAMWord(ThrottlePosition, WORD_MEM_IDX(XRAM, THROTTLE_POSITION_SUM));

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
  addByteInXRAMWord(RAM[0x49], WORD_MEM_IDX(XRAM, RAM_49_SUM));

  if (++XRAM[0xF69D] == 0x20) {
    Xram_F69D_eq_20();
  }

  // goto _2ED3;
  _2ED3:

  if (++XRAM[0xF69A] == 4) {
    _2EDC:
    XRAM[0xF69A] = 0; // Reset counter
    word IgnVoltage = GET_MEM_WORD(XRAM, IGNITION_SW_VOLTAGE_SUM);
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
  // MODE SELECTION
  SelectMode();

  _2F41:
  // EGO #1 Sensor (8bit ADC)
  ProcessEGO(EGO1_PIN, &XRAM[0xF68B], &RAM[0x2B], 4, kitting_has_ego_sensor(),
             &RAM[0x22], 6, &RAM[0x22], 7);

  _2FD0:
  ego2_sensor_start:
  // EGO #2 Sensor (8bit ADC)
  ProcessEGO(EGO2_PIN, &XRAM[0xF68C], &RAM[0x2B], 5,
             kitting_has_ego_sensor() && kitting_has_additional_ego_sensor(),
             &RAM[0x23], 0, &RAM[0x23], 1);

  _3067:
  no_additional_ego_sensor:
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
    SET_MEM_WORD(XRAM, THROTTLE_POSITION_THRESHOLD, COMPOSE_WORD(Ram40Val, 0));

    SET_MEM_WORD(XRAM, THROTTLE_POSITION_1, COMPOSE_WORD(FLASH[0x807C], 0));
    SET_MEM_WORD(XRAM, THROTTLE_POSITION_2, COMPOSE_WORD(FLASH[0x807C], 0));
  } else {
    _308A:
    word Throttle = ADC_10bit(THROTTLE_POSITION_PIN);

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
  {
    word ThrottlePositionLessThreshold = GET_MEM_WORD(XRAM, THROTTLE_POSITION_LESS_THRESHOLD);
    RAM[0x43] = InterpolateTableValue(
        0x865C,
        HIGH(ThrottlePositionLessThreshold),
        LOW(ThrottlePositionLessThreshold));
    RAM[0x42] = ((RAM[0x43] + 4) & 0xF8) >> 3;
    RAM[0x41] = ((RAM[0x41] + 8) & 0xF0) >> 4;
  }

  // _3147:
  if (CHECK_BIT_AT(RAM[0x27], 4)) {
    // _31A5:
    // TODO
  } else {
    // _314A:
    word ThrottlePositionLessThreshold =
        GET_MEM_WORD(XRAM, THROTTLE_POSITION_LESS_THRESHOLD);
    SET_MEM_WORD(XRAM, THROTTLE_POSITION_LESS_THRESHOLD_2,
                 ThrottlePositionLessThreshold);
    word Filtered = filterThrottlePosition(ThrottlePositionLessThreshold);

    SET_MEM_WORD(XRAM, FILTERED_THROTTLE_POSITION_LESS_THRESHOLD_1, Filtered);
    SET_MEM_WORD(XRAM, FILTERED_THROTTLE_POSITION_LESS_THRESHOLD_2, Filtered);

    CLEAR_BIT_IN(IEN0, 0); // disable external interrupt 0
    SET_MEM_WORD(XRAM, FILTERED_THROTTLE_POSITION_LESS_THRESHOLD_3, Filtered);
    SET_BIT_IN(IEN0, 0); // enable external interrupt 0

    word Scaled =
        scale10bitADCValue(COMPOSE_WORD(XRAM[0xF6CD],XRAM[0xF6CC]), RAM[0x49]);
    Scaled = multiply16WithSaturation(Scaled);
    XRAM[0xF6E4] = LOW(Scaled);
    XRAM[0xF6E5] = HIGH(Scaled);

    CLEAR_BIT_IN(IEN0, 0);
    XRAM[0xF6DC] = XRAM[0xF6DD] = 0;
    SET_BIT_IN(IEN0, 0);
  }

  // _3514:
  // TODO
  ;
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

inline void StopTimer0() {
  CLEAR_BIT_IN(TCON, 4);
}

inline void StartTimer0() {
  SET_BIT_IN(TCON, 4);
}

inline void DisableSerial0Int() {
  Reg::Mask<Reg::IEN0> M;
  M.add<Reg::IEN0::ES0>();
}

inline void EnableSerial0Int() {
  Reg::Set<Reg::IEN0> M;
  M.add<Reg::IEN0::ES0>();
}

inline void EnableSerial0Receiver() {
  Reg::Set<Reg::S0CON> M;
  M.add<Reg::S0CON::REN0>();
}

inline void DisableSerial0Receiver() {
  Reg::Mask<Reg::S0CON> M;
  M.add<Reg::S0CON::REN0>();
}

inline bool checkSerial0TxD() {
  return CHECK_BIT_AT(P3, 1);
}

inline bool checkSerial0RxD() {
  return CHECK_BIT_AT(P3, 0);
}

#define BYTE_PAIR_GEQ(rh, lh) \
  ((XRAM[(rh)] >= XRAM[(lh)]) && (XRAM[(rh) - 1] >= XRAM[(lh) - 1]))

inline void _0F29() {
  // _0F29:
  // ram_2f_bit_4_or_bit_5_set:
  XRAM[0xF97F] = XRAM[0xF980] = 0;
  RAM[0x77] = 4;
}

inline void _0F1F() {
  // _0F1F:
  XRAM[0xF9A0] = XRAM[0xF9A1] = 0;
  _0F29();
}

inline void _0F1C() {
  // _0F1C:
  DisableSerial0Int(); // Disable Serial0 Interrupt
  _0F1F();
}

inline void _1076() {
  RAM[0x77] = 0xFC;
}

inline void _1192() {
  // _1192:
  XRAM[0xF97F] = XRAM[0xF980] = 0;
  // Clear Serail0 Receiver Interrupt request flag
  {
    Reg::Mask<Reg::S0CON> M;
    M.add<Reg::S0CON::RI0>();
  }

  EnableSerial0Receiver();
  RAM[0x77] = 0xFF;

  CLEAR_BIT_IN(RAM[0x2F], 1);
  init_xram_for_serial0();
  S0RELH_S0RELL = 0:0;
}

inline void _11E7() {
  // _11E7:
  // ram_2f_bit_1_not_set_4:
  EnableSerial0Int();
  RAM[0x77] = 0x06;
}

inline void _11B3() {
  // _11B3:
  // ram_2f_bit_2_set:
  XRAM[0xF97F] = XRAM[0xF980] = 0;
  if (!XRAM[0xF991] && !XRAM[0xF992]) {
    // _11D2:
    // xram_f991_and_xram_f992_eq_0:
    if (CHECK_BIT_AT(RAM[0x2F], 1) &&
        (XRAM[0xF983] || XRAM[0xF984])) {
      // _11E1:
      EnableSerial0Int();
      RAM[0x77] = 0xFF;
    }
    _11E7();
  } else {
    // _11C9:
    DisableSerial0Int();
    RAM[0x77] = 0x05;
  }
}

inline void _0EAF() {
  // _0EAF:
  XRAM[0xF97F] = XRAM[0xF980] = 0;
  EnableSerial0Int(); // Enable Serial0 interrupt
  RAM[0x77] = 0xFF;
}

inline void _124A() {
  // _124A:
  // xram_f987_and_xram_f988_eq_0:
  XRAM[0xF97F] = XRAM[0xF980] = 0;
  {
    Reg::Mask<Reg::S0CON> M;
    M.add<Reg::S0CON::TB80>();
  }
  // Set Serial0 Transmitter interrupt flag?
  {
    Reg::Set<Reg::S0CON> S;
    S.add<Reg::S0CON::TI0>();
  }
  EnableSerial0Int();
  RAM[0x77] = 2;
}

inline void _11F3() {
  // _11F3:
  if (CHECK_BIT_AT(RAM[0x2F], 1)) {
    // _11F9:
    // ram_2f_bit_1_not_set_5:
    XRAM[0xF99F] = XRAM[0xF9A3];
    XRAM[0xF99E] = XRAM[0xF9A2];

    DPTR[7] = 0xFAA8;
    if (!XRAM[0xF98D] && !XRAM[0xF98E]) {
      _124A();
    } else {
      // _123A:
      XRAM[0xF97F] = XRAM[0xF980] = 0;
      RAM[0x77] = 3;
    }
  } else {
    // _11F6:
    _0F1F();
  }
}

inline void _11F0() {
  // _11F0:
  DisableSerial0Receiver();

  _11F3();
}

static byte SET_CLEAR_MASKS_PER_COIL[static_cast<byte>(CoilE::CoilCount)] = {
  // 0 = Coil 2/3
  0xFE,
  // 1 = Coil 1/4
  0xFD,
};

// _12C9:
inline void either_ignition_coil_just_discharged() {
  // _12C9:
  if (--RAM[0x4F]) {
    goto _12F3;
    // TODO
  } else /* RAM[0x4F] started being equal 0 */ {
    switch (RAM[0x4D]) {
      case 0:
      case 1:
      default:
        // ram_4d_not_eq_1_3:
        goto _12FF;
        // TODO
    }
  }
  UNREACHABLE;
}

#define GOTO_x_THEN_12FF(x) \
do {                        \
  x();                      \
  goto _12FF;               \
} while (0)

// _12DA:
inline void either_ignition_coil_started_charging_sub() {
  // _12DA:
  byte Ram4EUpdateVal = (FLASH[0x8A5B + RAM[0x3F]] & 0xF0) >> 4;
  if (!Ram4EUpdateVal)
    ++Ram4EUpdateVal;
  RAM[0x4E] = Ram4EUpdateVal;
}

// _12FF:
inline void _12FF() {
  const byte Ram6AInc = ++RAM[0x6A];
  if (Ram6AInc == 0x3E)
    RAM[0x6A] = 0;

  if (Ram6AInc >= RAM[0x6B])
    SET_BIT_IN(P4, 4); // start scavenging absorber
  else
    CLEAR_BIT_IN(P4, 4); // stop scavenging absorber

  if (++RAM[0x68] >= RAM[0x69])
    SET_BIT_IN(P9, 1); // turn on fuel metering device?

  // _131B:
  // ram_68_less_ram_69:
  if (0x7D == RAM[0x68]) {
    RAM[0x68] = 0;
    CLEAR_BIT_IN(P9, 1);
  }

  // _1324:
  // ram_68_neq_7d:

  if (Reg::Bit<Reg::TCON, Reg::TCON::TF0>{}.get())
    SET_BIT_IN(RAM[0x20], 4);

  // _1333:
  // FINISH Timer0 Overflow Interrupt
}

inline void _12FD(const bool With_12F3 = false) {
  if (With_12F3) {
    // _12F3:
    RAM[0x4E] = (FLASH[0x8751] & 0xF0) >> 4;
  }

  // _12FD:
  // flash_8752_is_nil:
  CLEAR_BIT_IN(RAM[0x29], 1);

  _12FF();
}

inline void _12F3() {
  _12FD(true);
}

// _12EA:
inline void clrmsk_setmsk_updated_sub() {
  byte Ram4F = FLASH[0x8752];
  RAM[0x4F] = Ram4F;

  const bool With_12F3 = !!Ram4F;

  _12FD(With_12F3);
}

template <CoilE Coil, bool IgniteOrUpdateMasks>
inline void processIgnCoilForTimer0Overflow() {
  if constexpr (IGNITION_COIL_CHARGING(Coil)) {
    if constexpr (IgniteOrUpdateMasks) {
      IGNITE_COIL(Coil);
      either_ignition_coil_just_discharged(); // => _12C9
    } else {
      //goto either_ignition_coil_is_charging; // => _1290
      _12FF();
    }
  } else {
    if constexp (IgniteOrUpdateMasks) {
      START_CHARGING_IGNITION_COIL(Coil);
      //goto either_ignition_coil_started_charging; // => _12C6
      //goto either_ignition_coil_started_charging_sub; // => _12DA
      //GOTO_x_THEN_12FF(either_ignition_coil_started_charging_sub);
      either_ignition_coil_started_charging_sub();
      _12FF();
    } else {
      // _1285:
      CLRMSK &= SET_CLEAR_MASKS_PER_COIL[static_cast<byte>(Coil)];
      SETMSK &= SET_CLEAR_MASKS_PER_COIL[static_cast<byte>(Coil)];
      //goto clrmsk_setmsk_updated; // => _1293
      //goto clrmsk_setmsk_updated_sub; // => _12EA
      clrmsk_setmsk_updated_sub();
    }
  }

  UNREACHABLE;
}

/// \tparam IgniteOrUpdateMasks false to only update set/clr masks
///                             true to perform ignition/charging
template <bool IgniteOrUpdateMasks>
inline void processRam4DForTimer0Overflow() {
  byte Ram4D = RAM[0x4D];

  switch (Ram4D) {
    case 0: {
      processIgnCoilForTimer0Overflow<CoilE::C_1_4, IgniteOrUpdateMasks>();
      break;
    }
    case 1: {
      processIgnCoilForTimer0Overflow<CoilE::C_2_3, IgniteOrUpdateMasks>();
      break;
    }
    default: {
      //goto flash_8752_is_nil; // => _12FD
      _12FD();
      break;
    }
  }
}
// _0DB6:
void Timer0OverflowInterrupt() {
  StopTimer0();

  S0RELH_S0RELL is S0RELH:S0RELL

  // TH_TL0 is TH0:TL0
  {
    static const word ToAdd = 0xFAD0;
    if (0xFFFF - TH_TL0 < ToAdd)
      // 0DD0:
      TH_TL0 = 0xFACB;
    else
      TH_TL0 += ToAdd;
  }

  // 0DD6:
  // dont_reload_timer0:
  StartTimer0();

  if (--RAM[0x35] == 0) {
    RAM[0x35] = 0x14;
    SET_BIT_IN(RAM[0x28], 0);
  }

  // 0DE0:
  if (!(++XRAM[0xF97F]) && !(++XRAM[0xF980])) {
    // __0DEE:
    XRAM[0xF97F] = 0xFF;
    XRAM[0xF980] = 0xFF;
  }

  // TODO Decompile and re-organize

  byte Ram77 = RAM[0x77];
  // _0DF8:
  // no_overflow_on_inc_xram_f97f_or_f980:
  switch (Ram77) {
    case 0xFF: {
        // _0DFD:
        if (CHECK_BIT_AT(RAM[0x2F], 1)) {
          // _0E0D:
          // ram_2f_bit_1_set:
          if (BYTE_PAIR_GEQ(0xF980, 0xF994)) {
            // _0E31:
            // xram_f97f_geq_xram_f993:
            _0F1C();
          }
        } else {
          XRAM[0xF97F] = XRAM[0xF980] = 0;
        }
        break;
      }
    case 0xFE: {
        // _0E3C:
        // ram_77_eq_fe:
        if (CHECK_BIT_AT(S0CON, 0)) {
          // _0EAC:
          // receive_request_on_serial0:
          CLEAR_BIT_IN(S0CON, 0); // Clear RI0 flag (serial recieve request)

          _0EAF();
        } else /* _0E41 */ if (CHECK_BIT_AT(P3, 0)) {
          // _0E6A:
          // RxD_eq_1:
          if (BYTE_PAIR_GEQ(0xF980, 0xF982)) {
            // _0E8E:
            // xram_f97f_geq_xram_f981:
            XRAM[0xF97F] = XRAM[0xF980] = 0;

            if (CHECK_BIT_AT(RAM[0x2F], 1)) {
              // _0E9E:
              // ram_2f_1_is_set:
              SET_BIT_IN(RAM[0x2F], 0);
              init_xram_for_serial0();
              S0RELH_S0RELL = 0xFFD0;
            }

            //goto ram_2f_bit_1_not_set_4;
            _11E7();
          } else {
            _0EAF();
          }
        } else {
          // _0E44:
          if (BYTE_PAIR_GEQ(0xF980, 0xF984)) {
            // _0E68:
            // xram_f97f_geq_xram_f983:
            _0EAF();
          }
        }
        break;
      }
    case 0xFD: {
        // _102B:
        if (!checkSerial0TxD()) {
          // _1040:
          // serial0_txd_not_set:
          if (checkSerial0RxD()) {
            // _107C:
            // serial0_rxd_set:
            if (CHECK_BIT_AT(RAM[0x2F], 1)) {
              // _1082:
              // ram_2f_bit_2_set_2:
              SET_BIT_IN(P3, 1);
              EnableSerial0Receiver();
              _0F1F();
            } else {
              // _108A:
              // ram_2f_bit_1_not_set_3:
              XRAM[0xF97F] = XRAM[0xF980] = 0;
            }
          } else /* _1043: */ if (BYTE_PAIR_GEQ(0xF980, 0xF986)) {
            // _1067:
            // xram_f97f_geq_xram_f985:
            SET_BIT_IN(P3, 1); // Set TxD @ MC33199
            EnableSerial0Receiver();
            XRAM[0x7F] = XRAM[0xF980] = 0;

            _1076();
          }
        } else {
          // _102E:
          DisableSerial0Receiver();
          CLEAR_BIT_IN(P3, 1);
          XRAM[0xF97F] = XRAM[0xF980] = 0;
        }
        break;
      }
    case 0xFC: {
        // _109A:
        if (BYTE_PAIR_GEQ(0xF980, 0xF988)) {
          // _10BE:
          // xram_f97f_geq_xram_f987:
          XRAM[0xF99F]:XRAM[0xF99E] = XRAM[0xF9A3]:XRAM[0xF9A2];
          DPTR[7] = 0xFAA8;
          _124A();
        }
        break;
      }
    case 0x08: {
        // _0F96:
        if (BYTE_PAIR_GEQ(0xF980, 0xF998)) {
          // _0FBA:
          // xram_f97f_geq_xram_f997:
          DisableSerial0Int();
          if (!CHECK_BIT_AT(RAM[0x2F], 2)) {
            // _0FC3:
            // ram_2f_bit_2_not_set:
            _1192();
          } else {
            // _0FC0: goto ram_2f_bit_2_set; // => _11B3
            _11B3();
          }
        }
        break;
      }
    case 0x07: {
        // _0F60:
        if (BYTE_PAIR_GEQ(0xF980, 0xF996)) {
          // _0F84:
          // xram_f97f_geq_xram_f995:
          {
            Reg::Mask<Reg::S0CON> M;
            M.add<Reg::S0CON::RI0>();
          }
          {
            Reg::Set<Reg::S0CON> S;
            S.add<Reg::S0CON::REN0>();
          }
          EnableSerial0Int();
          RAM[0x77] = 8;
        }
        break;
      }
    case 0x06: {
        // _0EF2:
        if (BYTE_PAIR_GEQ(0xF980, 0xF994)) {
          // _0F19:
          // xram_f97f_geq_xram_f993_2:
          if (!CHECK_BIT_AT(RAM[0x2F], 1)) {
            // _0F19:
            // ram_2f_bit_1_not_set:
            if (CHECK_BIT_AT(RAM[0x2F], 0)) {
              // _0F4F:
              // ram_2f_bit_0_set:
              DisableSerial0Int(); // Disable Serial0 Interrupt
              if (CHECK_BIT_AT(RAM[0x2F], 4) ||
                  CHECK_BIT_AT(RAM[0x2F], 5)) {
                _0F29();
              } else {
                // _0F58:
                CLEAR_BIT_IN(RAM[0x2F], 2);

                _1192();
              }
            } else {
              // _0F3C:
              XRAM[0xF97F] = XRAM[0xF980] = 0;
              EnableSerial0Int(); // Enable Serial0 Interrupt
              RAM[0x77] = 0xFF;
            }
          } else {
            _0F1C();
          }
        }
        break;
      }
    case 0x05: {
        // _0EC5:
        if (BYTE_PAIR_GEQ(0xF980, 0xF991)) {
          // _0EE9:
          // xram_f97f_geq_xram_f991:
          CLEAR_BIT_IN(S0CON, 0); // Clear Serial0 Receieve request
          SET_BIT_IN(S0CON, 4); // Enable serial0 reception
          //goto ram_2f_bit_1_not_set_4; // => _11E7
          _11E7();
        }
        break;
      }
    case 0x04: {
        // _0FCE:
        // ram_77_eq_4:
        if (!CHECK_BIT_AT(Reg::S0CON, Reg::S0CON::RI0)) {
          // _0FE3:
          // no_serial0_receive_request:
          if (BYTE_PAIR_GEQ(0xF980, 0xF990)) {
            // _1007:
            // xram_f97f_geq_xram_f98f:
            XRAM[0xF97F] = XRAM[0xF980] = 0;
            if (!CHECK_BIT_AT(RAM[0x2F], 1)) {
              // _101F:
              // ram_2f_bit_1_not_set_2:
              if (!CHECK_BIT_AT(RAM[0x2F], 0)) {
                // _1022:
                _11B3();
              }
            } else {
              // _1014:
              CLEAR_BIT_IN(RAM[0x2F], 0);
              if (CHECK_BIT_AT(RAM[0x2F], 0)) {
                // _101C:
                // ram_2f_bit_0_set_2:
              } else {
                // _1019:
              }
            }
          }
        } else {
          // _0FD3:
          {
            Mask<Reg::S0CON> M;
            M.add<Reg::S0CON::RI0>();
          }
          XRAM[0xF97F] = XRAM[0xF980] = 0;
        }
        break;
      }
    case 0x03: {
        // _10DF:
        if (Reg::Bit<Reg::S0CON, Reg::S0CON::RI0>().get()) {
          // _10E7:
          // no_serial0_receive_request_2:
          if (BYTE_PAIR_GEQ(0xF980, 0xF98E)) {
            // _110B:
            // xram_f97f_geq_xram_f98d:
            XRAM[0xF97F] = XRAM[0xF980] = 0;
            if (XRAM[0xF987] || XRAM[0xF988]) {
              // _1124:
              // xram_f987_or_xram_f988_not_null:
              if (CHECK_BIT_AT(RAM[0x2F], 1)) {
                // _112A:
                // ram_2f_bit_1_set_2:
                RAM[0x77] = 0xFD;
              } else {
                // _1127:
                _1076();
              }
            } else {
              // _124A:
              // xram_f987_and_xram_f988_eq_0:
              _124A();
            }
          }
        } else {
          // _10E4:
          _11F0();
        }
        break;
      }
    case 0x02: {
        // _1133:
        if (BYTE_PAIR_GEQ(0xF980, 0xF98C)) {
          // _1157:
          // _xram_f97f_geq_xram_f98b:
          {
            Mask<Reg::IEN0> M;
            M.add<Reg::IEN0::ES0>();
          }
          _11F3();
        }
        break;
      }
    case 0x01: {
        // _1160:
        if (Reg::Bit<Reg::S0CON, Reg::S0CON::RI0>().get()) {
          // _1168:
          // no_serial0_receive_request_3:
          if (BYTE_PAIR_GEQ(0xF980, 0xF98A)) {
            // _118C:
            // xram_f97f_geq_xram_f989:
            _124A();
          }
        } else {
          // _1165:
          _11F0();
        }
        break;
      }
    default:
      break;
  }

  // TODO Decompile and re-organize
  // _1263:
  if (CHECK_BIT_AT(RAM[0x29], 1)) {
    // _1269:
    // ram_29_bit_1_set:
    if (CHECK_BIT_AT(RAM[0x2B], 7)) {
      // _126F:
      // ram_2b_bit_7_set:

      processRam4DForTimer0Overflow<false>();
    } else {
      // _12FD:
      // flash_8752_is_nil:
      _12FD();
    }
  } else {
    // _1299:
    if (RAM[0x4F]) {
      // 12A0:
      // ram_4f_neq_nil:
      if (--RAM[0x4E]) {
        // _1296:
        // ram_4e_neq_0:
        goto _12FF;
      } else {
        // _12A3:
        processRam4DForTimer0Overflow<true>();
      }
    } else {
      // _129D:
      goto _12FF;
      _12FF();
    }
  }
}
