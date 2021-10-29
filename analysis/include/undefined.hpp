#pragma once

/* Send DATA byte over SPI interface described by
   MOSI (output), MISO (input) and CLK pins.
   Sends data over MOSI pin.
   Returns byte fetched on MISO pin.
   0 for MISO - no input. */
byte SendOverSPI(byte DATA, pin MOSI, pin MISO, pin CLK);

/* Send DATA byte over I2C bus described by
   SDA (I/O), SCL (output).
   Returns if byte was ACK'ed */
bool SendOverI2C(byte DATA, pin SDA, pin SCL);
/* Read single byte over I2C bus described by
   SDA (I/O), SCL (output).
   Returns byte read */
byte ReadOnI2C(pin SDA, pin SCL);

/* A/D Convert voltage at the specified pin.
   Result is 10-bit integer value which is set at the highest 10 bits of a word.
 */
word ADC_10bit(pin Pin);
/* A/D Convert voltage at the specified pin.
   Result is 8-bit integer value.
 */
byte ADC_8bit(pin Pin);

void enable_access_to_xram();
void disable_access_to_xram();


