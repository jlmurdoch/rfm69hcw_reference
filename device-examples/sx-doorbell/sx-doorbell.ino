/*
 * Elro / Byron SX doorbell transmitter example
 * Tested on a Byron SX209i
 *
 * Format:
 * - 26 pulses = lead + data x 24 + coda
 * - Manchester Coding (24 data pulses = 12 data bits)
 * - Leading HIGH is 260us
 * - 0 bit = LOW (260us), HIGH (520us) 
 * - 1 bit = LOW (520us), HIGH (260us)
 * - Sync bit LOW is 2600us
 * - Repeated numerous times
 */

#include <SPI.h>

// SPI for setup, DIO2 for data
#define RFM69HCW_CS       6   // D - SPI Chip Select
#define RFM69HCW_RST      9   // C - Reset
#define RFM69HCW_G2_DIO2  11  // A - Data

#define MODULATION    OOK
#define FREQ          433.92
#define BITRATE       2400
#define BIT_SYNC      0x60 // Bit Sync off, as pulses are asymmetrical

// RFM69HCW clock speed - do not change
#define FXOSC 32000000.0

// RFM69HCW enumerations
enum { FSK, OOK };

// Settings for the doorbell
#define PULSE 260 // 2400bps
enum { SHORT, LONG, SYNC = 15};
enum { TUBULAR3 = 1, MORNING_DEW, BIG_BEN, TUBULAR2 = 5, SAXOPHONE, SOLO = 9, TUBULAR_SCALE = 13, CLARINET };
uint8_t bellid = 255; // 0 to 255
uint8_t toneid = SOLO; // Least annoying test tone 
uint8_t repeat = 40;

// Core code for SPI read / write commands
uint8_t spi_cmd(uint8_t addr, uint8_t value) {
  uint8_t status;

  digitalWrite(RFM69HCW_CS, LOW);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(addr);
  status = SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(RFM69HCW_CS, HIGH);
    
  return status;
}

// For write, set the wnr bit (Section 5.2.1) 
uint8_t spi_write(uint8_t addr, uint8_t value) {
  return spi_cmd(addr | 0x80, value);
}

// For read, write 0x00
uint8_t spi_read(uint8_t addr) {
  return spi_cmd(addr, 0x00);
}

// Initialise RFM69HCW
void rfminit() {
  // Section 5.2.1 - SPI Interface
  digitalWrite(RFM69HCW_CS, HIGH);

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  
  // Section 7.2.2 - Manual Reset
  digitalWrite(RFM69HCW_RST, HIGH);
  delayMicroseconds(100);
  digitalWrite(RFM69HCW_RST, LOW);
  // Spec says 5ms, but takes longer
  delay(6);

  // Read the version register, if it's not 0x24, something is wrong
  if(spi_read(0x10) == 0x24)
    Serial.println(F("RFM69HCW SPI Working"));
  else
    Serial.println(F("RFM69HCW SPI Failed"));
  
  // RegOpMode: Standby Mode & wait
  spi_write(0x01, 0x04); 
  while(!(spi_read(0x27)) & 0x80);
  
  // RegDataModul: Continuous w/ bit-sync and no other shaping 
  uint8_t modulation = MODULATION;
  spi_write(0x02, BIT_SYNC | modulation << 3); 

  // RegFrf*: Frequency across 3 bytes
  uint32_t freqrf = (FREQ * 1000000.0) / (FXOSC / 524288);
  spi_write(0x07, (freqrf >> 16) & 0xFF);
  spi_write(0x08, (freqrf >> 8) & 0xFF);
  spi_write(0x09, freqrf & 0xFF);

  // RegBitrate*: Bitrate needed for Bit Synchronizer
  uint16_t bitrate = FXOSC / BITRATE;
  spi_write(0x03, (bitrate >> 8) & 0xFF);
  spi_write(0x04, bitrate & 0xFF);

  // ClkOut set to Off to save some power
  spi_write(0x26, 0x07);

  /*
   * Transmission Settings
   */ 
  
  // Non-high power settings for testing
  spi_write(0x11, 0x50); // PA1 @ +6 dBm
  spi_write(0x13, 0x1A); // Default with OCP on
    
  // Enter Sleep Mode
  spi_write(0x01, 0x00);
  while(!(spi_read(0x27)) & 0x80);

  // Shutdown the SPI for now
  SPI.endTransaction();
}

void setup() {
  // Setup pins other than SPI, VDD and ground
  pinMode(RFM69HCW_CS, OUTPUT);
  pinMode(RFM69HCW_RST, OUTPUT);
  pinMode(RFM69HCW_G2_DIO2, OUTPUT);

  // SPI Setup
  SPI.begin();

  // Configure the receiver
  rfminit();
}

void emitPulse(byte state, byte size) {
  // Hold the level for short or long
  digitalWrite(RFM69HCW_G2_DIO2, state);
  delayMicroseconds(PULSE * (size + 1));
}

void loop() {
  // If you press the Boot(sel) button, it will transmit
  if (BOOTSEL) {
    // Payload = ID (8 bits) + Tone (4 bits)
    uint16_t bits = (bellid << 4) | (toneid & 0xF);

    // Enter Transmit Mode
    spi_write(0x01, 0x0C);
    while(!(spi_read(0x27)) & 0x80);

    // Send the raw data over DIO2
    for (int x = 0; x < repeat; x++) {
      // Leading HIGH pulse
      emitPulse(0x1, SHORT);

      // from bit 11 to bit 0 (12 bits)
      int y = 11;
      // Send LOW then HIGH for each bit, adjusting gap if zero or one
      do {
        // Low: 0=short, 1=long
        emitPulse(0x0, bits >> y & 0x1);
        // High: 0=long, 1=short
        emitPulse(0x1, ~bits >> y & 0x1); 
      } while (y--);

      // Ending LOW pulse
      emitPulse(0x0, SYNC);
    }

    // Enter sleep Mode
    spi_write(0x01, 0x00);
    while(!(spi_read(0x27) & 0x80));

    // Wait for button to be released
    while(BOOTSEL)
      delay(1);
  }
}
