/*
 * Generic Radio Example
 */

#include <SPI.h>

#define RFM69HCW_G0_IRQ   5   // E - PayloadReady/CrcOk
#define RFM69HCW_CS       6   // D - SPI Chip Select
#define RFM69HCW_RST      9   // C - Reset
#define RFM69HCW_G2_DIO2  11  // A - Data
#define RFM69HCW_G1_DIO1  13  // 13 - DCLK if Gaussian FSK needed

#define MODULATION    OOK
#define FREQ          433.92
#define BITRATE       1000
#define BANDWIDTH     DCCFREQ_4 << 5 | RXBWMANT_16 << 3 | RXBWEXP_0
#define AFC_BANDWIDTH DCCFREQ_4 << 5 | RXBWMANT_16 << 3 | RXBWEXP_0
#define BIT_SYNC      0x60 // Bit Sync off, as pulses are asymmetrical

// RFM69HCW clock speed - do not change
#define FXOSC 32000000.0

enum { FSK, OOK };

uint8_t spi_cmd(uint8_t addr, uint8_t value) {
  uint8_t status;

  digitalWrite(RFM69HCW_CS, LOW);
  SPI.transfer(addr);
  status = SPI.transfer(value);
  digitalWrite(RFM69HCW_CS, HIGH);

  return status;
}

// Set the wnr bit to write (Section 5.2.1) 
uint8_t spi_write(uint8_t addr, uint8_t value) {
  return spi_cmd(addr | 0x80, value);
}

// Pass nothing when reading
uint8_t spi_read(uint8_t addr) {
  return spi_cmd(addr, 0x00);
}

void rfminit() {
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
  uint32_t freqrf = (FREQ * 1000000) / (FXOSC / 524288);
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
   * TX- Settings
   */ 
  // High Power toggles
  spi_write(0x5A, 0x5D); // Section 3.3.7 PA1 on for +20dBm
  spi_write(0x5C, 0x7C); // Section 3.3.7 PA2 on for +20dBm

  // Power Settings for Tx
  spi_write(0x13, 0x0F); // Default = 0x1A. Disable OCP = 0x0F
  spi_write(0x11, 0x7F); // Default = 0x9F. 23dBm = 0x1F, PA1+2 on = 0x60
  // spi_write(0x12, 0x09); // Default = 0x09. Ramp time for FSK

  // Sleep
  spi_write(0x01, 0x00);
  while(!(spi_read(0x27)) & 0x80);
}

void setup() {
  // Setup pins other than SPI, VDD and ground
  pinMode(RFM69HCW_CS, OUTPUT);
  pinMode(RFM69HCW_RST, OUTPUT);
  pinMode(RFM69HCW_G2_DIO2, OUTPUT);
  // pinMode(RFM69HCW_G1_DIO1, OUTPUT); // For DCLK

  // For flashing the LED on emitting
  pinMode(LED_BUILTIN, OUTPUT);

  // Serial console: e.g. cu -s 115200 -l /dev/ttyACM0
  Serial.begin(115200);
  delay(5000);

  // Section 5.2.1 - SPI Interface
  digitalWrite(RFM69HCW_CS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  Serial.println(F("SPI connection open"));

  // Section 7.2.2 - Manual Reset
  digitalWrite(RFM69HCW_RST, HIGH);
  delayMicroseconds(100);
  digitalWrite(RFM69HCW_RST, LOW);
  // Spec says 5ms, but takes longer
  delay(6);

  // Configure the receiver
  rfminit();
}

void loop() {
  delay(10000); // 10 secs

  digitalWrite(LED_BUILTIN, HIGH);

  // Section 3.2.2 - Async output is on DIO2  
  digitalWrite(RFM69HCW_G2_DIO2, HIGH);
  // Transmit
  spi_write(0x01, 0x0C);
  while(!(spi_read(0x27)) & 0x80);

  // Section 3.2.2 - Async output is on DIO2  
  // High for 1ms (1 kbps)
  digitalWrite(RFM69HCW_G2_DIO2, HIGH);
  //delayMicroseconds(1); // Minimum pre-hold is 250ns
  //digitalWrite(RFM69HCW_G1_DIO1, HIGH);
  //delayMicroseconds(1); // Minimum post-hold is 250ns
  delayMicroseconds(1000);

  // Low for 1ms (1 kbps)
  digitalWrite(RFM69HCW_G2_DIO2, LOW);
  //delayMicroseconds(1); // Minimum pre-hold is 250ns
  //digitalWrite(RFM69HCW_G1_DIO1, HIGH);
  //delayMicroseconds(1); // Minimum post-hold is 250ns
  delayMicroseconds(1000);

  // Sleep
  spi_write(0x01, 0x00);
  while(!(spi_read(0x27)) & 0x80);

  digitalWrite(LED_BUILTIN, LOW);
}
