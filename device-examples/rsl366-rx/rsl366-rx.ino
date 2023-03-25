/*
 * Generic Radio Receiver - RSL366
 *
 * Tranmission pattern is:
 * 1  HIGH (400us)
 * 2  LOW (1340us)
 * 3A HIGH (1340us) + LOW (400us) = 0
 * 3B HIGH (400us) + LOW (1340us) = 1
 * 4  HIGH (400us)
 * <...repeat 1-4 for all 12 bits ...>
 * X HIGH (400us)
 * Y LOW (1340us + 13,000us)
 * <...repeat all of above four times ...> 
 *
 * Data is 12 bits, broken into 3 nibbles:
 * - nibble 1 - Channel ID (1 bit)
 * - nibble 2 - Device ID (1 bit)
 * - nibble 3 - OFF = 0x1, ON = 0x0   
 */
 
#include <SPI.h>

// Uses continuous mode, DIO2 is used for DATA, not DIO0 (packets)
// Adafruit RFM69HCW Breakout / FeatherWing
#define RFM69HCW_CS       6   // D - SPI Chip Select
#define RFM69HCW_RST      9   // C - Reset
#define RFM69HCW_G2_DIO2  11  // A - Data

#define MODULATION    OOK
#define FREQ          433.82
#define BITRATE       2275
#define BANDWIDTH     DCCFREQ_4 << 5 | RXBWMANT_16 << 3 | RXBWEXP_0
#define AFC_BANDWIDTH DCCFREQ_4 << 5 | RXBWMANT_16 << 3 | RXBWEXP_0
#define BIT_SYNC      0x60 // Bit Sync off, as pulses are asymmetrical

// RFM69HCW clock speed - do not change
#define FXOSC 32000000.0

enum { FSK, OOK };
enum { DCCFREQ_16, DCCFREQ_8, DCCFREQ_4, DCCFREQ_2, DCCFREQ_1, DCCFREQ_0_5, DCCFREQ_0_25, DCCFREQ_0_125 };
enum { RXBWMANT_16, RXBWMANT_20, RXBWMANT_24 };
enum { RXBWEXP_0, RXBWEXP_1, RXBWEXP_2,  RXBWEXP_3,  RXBWEXP_4, RXBWEXP_5,  RXBWEXP_6,  RXBWEXP_7 };

uint8_t thisValue, lastValue = 0;
uint32_t thisTime, lastTime = 0;
int thisGap = 0;
uint8_t pulses = 0;
uint16_t thisBuf, lastBuf = 0;

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

  // RegRxBw: Tuned bandwidth (narrow)
  spi_write(0x19, BANDWIDTH);
  // RegAfcBw: AFC bandwidth (wider)
  spi_write(0x1A, AFC_BANDWIDTH);

  // RegBitrate*: Bitrate needed for Bit Synchronizer
  uint16_t bitrate = FXOSC / BITRATE;
  spi_write(0x03, (bitrate >> 8) & 0xFF);
  spi_write(0x04, bitrate & 0xFF);

  /*
   * Essential OOK Settings:
   * - 0x1B   RegOokPeak:  RSSI threshold set by peak RSSI value
   * - 0x1D   RegOokFix:   RSSI floor is -32dB of peak value
   */
  spi_write(0x1B, 0x40);
  spi_write(0x1D, 32);
  
  // RegOpMode: Receiving mode, auto sequencer disabled
  spi_write(0x01, 0x10);
  while(!(spi_read(0x27) & 0x80));
}

void setup() {
  // Setup pins other than SPI, VDD and ground
  pinMode(RFM69HCW_CS, OUTPUT);
  pinMode(RFM69HCW_RST, OUTPUT);
  pinMode(RFM69HCW_G2_DIO2, INPUT);

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


// Used to convert single bits to their position value
byte bitconv(byte x) {
  x &= 0xF;
  if (x > 1)
    return 1 + bitconv(x/2);
  else 
    return 0;
}

void loop() {
  thisValue = digitalRead(RFM69HCW_G2_DIO2);

  if (thisValue != lastValue) {
    // Do the necessary state logging
    thisTime = micros();
   
    // 400ms = 0x1, 1340ms = 0x04/0x5 
    thisGap = (thisTime - lastTime) >> 8;
    pulses++;

    switch (thisGap) {
      case 0x1:
      case 0x4:
      case 0x5:
        // Valid pulse gaps seen

        if (pulses % 4 == 0) {
          // 4th bit = one we want to write, so make room
          thisBuf <<= 1;
          // Add the bit: 1340us = 1, 400us = 0;
          thisBuf |= thisGap >> 2;
        } else if (pulses == 49) {
          if (thisBuf != lastBuf) {
            // If bitstream doesn't match last, save and wait for another
            lastBuf = thisBuf; 
          } else {
            // If we have two matching bitstreams, it must be good, so print
            Serial.print("Ch: ");
            Serial.print(4 - bitconv(thisBuf >> 8));
            Serial.print(", Id: ");
            Serial.print(4 - bitconv(thisBuf >> 4));
            Serial.print(", On: ");          
            Serial.print(thisBuf & 0x1);
            Serial.println();
            lastBuf = 0;
          }
          // Clean up, start again
          thisBuf = 0;
          pulses = 0;
        }
        break;
      default:
        // Invalid pulse gaps, start again
        thisBuf = 0;
        pulses = 0;
        break;
    }
    // Save the state
    lastValue = thisValue;
    lastTime = thisTime;
  }
}
