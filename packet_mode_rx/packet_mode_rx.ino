/*
 * RFM69HCW Packet Mode Receiving Example
 * Description: 
 * - Packet Mode for the Mitsubishi PAR-WT50R-E Thermostat
 * - Performs data reception over SPI & FIFO
 * - DIO[0-5] / G[0-5] are redundant, but could be used
 * - Preamble, Sync word, then fixed 31 bytes
 * - No CRC checking, dynamic length or filtering
 */
 
#include <SPI.h>

// Only CS & RST used
// Adafruit RFM69HCW Breakout / FeatherWing
#define RFM69HCW_G0_IRQ   5   // E - PayloadReady/CrcOk
#define RFM69HCW_CS       6   // D - SPI Chip Select
#define RFM69HCW_RST      9   // C - Reset
#define RFM69HCW_G2_DIO2  11  // A - Data

// Mitsubishi PAR-WT50R-E Thermostat
#define MODULATION    FSK
#define BASEFREQ      867.8032
#define FREQDEV       4800
#define BITRATE       9600
#define BANDWIDTH     DCCFREQ_4 << 5 | RXBWMANT_20 << 3 | RXBWEXP_5
#define AFC_BANDWIDTH DCCFREQ_4 << 5 | RXBWMANT_20 << 3 | RXBWEXP_5

// RFM69HCW clock speed - do not change
#define FXOSC 32000000

enum { DCCFREQ_16, DCCFREQ_8, DCCFREQ_4, DCCFREQ_2, DCCFREQ_1, DCCFREQ_0_5, DCCFREQ_0_25, DCCFREQ_0_125 };
enum { RXBWMANT_16, RXBWMANT_20, RXBWMANT_24 };
enum { RXBWEXP_0, RXBWEXP_1, RXBWEXP_2,  RXBWEXP_3,  RXBWEXP_4, RXBWEXP_5,  RXBWEXP_6,  RXBWEXP_7 };

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
    Serial.println("RFM69HCW Online");
  
  // RegOpMode: Standby Mode & wait
  spi_write(0x01, 0x04);
  while(!(spi_read(0x27)) & 0x80);
  
  /*
   * Packet Mode:
   * - Preamble (x bytes) = 0x55
   * - Syncword (2 bytes) = 0x2DD4
   * - Length (1 byte) = 0x1C
   * - Address (1 byte) = 0x8D or 0x0D
   * - Payload (<length> bytes)
   * - CRC-16 (2 bytes - appears to be broken)
   */
  
  // Packet, FSK. No Shaping 
  spi_write(0x02, 0x00);
  
  // Packet: Syncword detection on
  spi_write(0x2E, 0x80);
  
  // Packet: Syncword define = 0x2D
  spi_write(0x2F, 0x2D);
  
  // Packet: Fixed Size, no formatting
  spi_write(0x37, 0x00);
  
  // Packet: Length is 31 bytes: 2nd syncword + payload (28 bytes) + CRC16
  spi_write(0x38, 0x1F);
   
  // RegBitrate*: Bitrate needed for Bit Synchronizer
  uint16_t bitrate = FXOSC / BITRATE;
  spi_write(0x03, (bitrate >> 8) & 0xFF);
  spi_write(0x04, bitrate & 0xFF);
 
  /*
   * Essential FSK Settings:
   * - 0x05-6 RegFdev*:      Set Frequency Deviation      
   * - 0x0B   RegAfcCtrl:    Improved AFC routine 
   * - 0x1E   RegAfcFei:     AFC on on Rx start
   * - 0x29   RegRssiThresh: RSSI Trigger @ -72dB
   * - 0x6F   RegTestDagc:   Fade improvement
   */
  uint16_t freqdev = FREQDEV / (FXOSC / 524288);
  spi_write(0x05, (freqdev >> 8) & 0x3F);
  spi_write(0x06, freqdev & 0xFF);

  // RegFrf*: Frequency across 3 bytes  
  uint32_t freqrf = (BASEFREQ * 1000000) / (FXOSC / 524288);
  spi_write(0x07, (freqrf >> 16) & 0xFF);
  spi_write(0x08, (freqrf >> 8) & 0xFF);
  spi_write(0x09, freqrf & 0xFF);

  // RegRxBw: Tuned bandwidth (narrow)
  spi_write(0x19, BANDWIDTH);

  // RSSI Threshold
  spi_write(0x29, 75 * 2);
 
  // AFC bandwidth (wider)
  spi_write(0x1A, AFC_BANDWIDTH);
  // AFC On - At Start
  spi_write(0x1E, 0x04);
  // Use AFC Improved
  spi_write(0x0B, 0x20);
  // DAGC: ModIdx<2 = 0x20, ModIdx>2=0x20
  spi_write(0x6F, 0x20);
  
  // RegOpMode: Receiving mode, auto sequencer disabled
  spi_write(0x01, 0x10);
  while(!(spi_read(0x27)) & 0x80);

  Serial.println("RFM69HCW Ready");
}

void setup() {
  // Setup pins
  pinMode(RFM69HCW_CS, OUTPUT);
  pinMode(RFM69HCW_RST, OUTPUT);

  // Serial console: e.g. cu -s 115200 -l /dev/ttyACM0
  Serial.begin(115200);
  delay(5000);

  // Section 5.2.1 - SPI Interface
  digitalWrite(RFM69HCW_CS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

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
  // Payload received?
  if (spi_read(0x28) & 0x04) {
    // While the FIFO is not empty...
    while(spi_read(0x28) & 0x40) {
      // Print each value
      Serial.print(spi_read(0x00), HEX);
      Serial.print(" ");
    }
    Serial.println("");
  }
  delay(1);
}