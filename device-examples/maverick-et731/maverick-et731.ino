/*
 * Maverick ET-732 BBQ Thermometer 
 * - 433.85MHz, OOK
 * - Manchester-encoding data (52bits)
 * - Bit sync doesn't / cannot work with the RFM69HCW:
 * - A) asymmetric pulse sizes (230us & 520us = >6.5% deviation)
 * - B) 10 bits of preamble for Manchester encoding (Need 12 bits)

 * Raw data:
 * - First bit: after 5.0ms (short pulse, then gap)
 * - Other bits: every 520us
 * - Skip any intermediate transitions (usually at 230ms)
 * - Pairs of bits should alternate 0>1 or 1>0 for Manchester code
 * - 104 bits in total - 52bits in Manchester encoding 
 *
 * Example:
 *  5.0ms   |520us|520us|520us|520us|520us|520us|520us|...
 *          |     |     |     |     |     |     |     |
 *          V_____V   __V     V__   V__   V_____V   __V   
 * _________|     |__|  |_____|  |__|  |__|     |__|  |__
  
 *  Raw: |  1     0  |  0     1  |  1     1  |  0     0  |
 *  MC:  |     1     |     0     |  Invalid  |  Invalid  |
 *
 * Manchester Encoded Data:
 * - Size: 52bits
 * - Format: (Header: 12bits, State: 4bits, Probes: 10 bits x 2, LFSR: 16 bits)
 *
 * Example: 
 *                      Hdr S Probe LFSR
 * Startup (no probes): FA8 7 00000 xxxx
 * Ongoing (no probes): FA8 2 00000 xxxx
 * Ongoing (#1 @ 21C):  FA8 2 8A400 xxxx
 * Ongoing (#2 @ 21C):  FA8 2 00229 xxxx
 *
 * LFSR = Linear Feedback Shift Register
 *        - Acts as a checksum on state+probe data
 *        - Result should be a consistent session id
 *
 * TODO:
 *  - Sleep for 10 secs between messages
 *  - Only start when init is seen?
 *  - Check the UID against another same-time message
 *  - Make the UID the "locked on" message until 5 mins of no reception
 *  - Blink the LED
 */

 
#include <SPI.h>

// As there are no packets, DIO2 is used for DATA, not DIO0
// Adafruit RFM69HCW Breakout / FeatherWing
#define RFM69HCW_G0_IRQ   5   // E - PayloadReady/CrcOk
#define RFM69HCW_CS       6   // D - SPI Chip Select
#define RFM69HCW_RST      9   // C - Reset
#define RFM69HCW_G2_DIO2  11  // A - Data

#define MODULATION    OOK
#define FREQ          433.85
#define BITRATE       4080
#define BANDWIDTH     DCCFREQ_4 << 5 | RXBWMANT_16 << 3 | RXBWEXP_0
#define AFC_BANDWIDTH DCCFREQ_4 << 5 | RXBWMANT_16 << 3 | RXBWEXP_0
#define BIT_SYNC      0x60 // Bit Sync off, as pulses are asymmetrical (230us vs 520us)

// RFM69HCW clock speed - do not change
#define FXOSC 32000000.0

enum { FSK, OOK };
enum { DCCFREQ_16, DCCFREQ_8, DCCFREQ_4, DCCFREQ_2, DCCFREQ_1, DCCFREQ_0_5, DCCFREQ_0_25, DCCFREQ_0_125 };
enum { RXBWMANT_16, RXBWMANT_20, RXBWMANT_24 };
enum { RXBWEXP_0, RXBWEXP_1, RXBWEXP_2,  RXBWEXP_3,  RXBWEXP_4, RXBWEXP_5,  RXBWEXP_6,  RXBWEXP_7 };

uint8_t thisValue, lastValue = 0;
uint32_t thisTime, lastTime = 0;
int thisGap = 0;
uint8_t bits, state = 0;
uint64_t buf = 0;

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
  while(!(spi_read(0x27)) & 0x80);
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

uint16_t calc_lfsr_id(uint32_t input) {
  // Initial state of LFSR - both output and mask change
  uint16_t output = 0;
  uint16_t mask = 0xDD38;

  // Cycles from MSB to LSB
  for (int i = 23; i >= 0 ; --i) {
    // XOR the mask contents at current position if that bit is set
    if (input >> i & 0x1)
      output ^= mask;
    
    // If mask has first bit set
    if (mask & 0x1) {
      // Shift and XOR against a fixed mask
      mask >>= 1;
      mask ^= 0x8810;
    } else {
      // Otherwise just do the shift
      mask >>= 1;
    }
  }
  return output;
}

void loop() {
  thisValue = digitalRead(RFM69HCW_G2_DIO2);

  if (thisValue != lastValue) {
    // Do the necessary state logging
    thisTime = micros();
    lastValue = thisValue;
   
    /* Rounds the gaps into 256us (+/-128us) chunks with comfortable tolerances
     * e.g. 128-384us = 0x0, 384-640us = 0x1, 4864-5120us = 0x13 
     * Avoids multiple if() and we can use a switch() instead, with cascading code reuse
     */
    thisGap = (thisTime - lastTime - 128) >> 8;

    // Shortest way to process Manchester Code without a clock and saving bits 
    switch (thisGap) {
      case 0x13: // 5040us
        // Reset, indicating a start of the 520us clock, so follow on thru...
        buf = 0x0;
        bits = 0; 
        // Set this to indicate that this current reception is clean, as we had a long pulse
        state = 1;   
      case 0x1: // 520us
        // Data coming on the 520us mark 
        lastTime = thisTime;
        bits++;
        // If we havent had the first bit of the Manchester Encoding 2-bit tuple, write it
        if (bits % 2) {
          buf <<= 1;
          // Use the last (true) value that corresponds to the pulse that we just had
          buf |= lastValue;    
        // Verify the 2nd bit is the exact opposite of the 1st (Manchester Encoding)
        } else if (buf & 0x1 == lastValue) {
          // If the 2nd bit matches the 1st bit, abort as this breaks the encoding
          bits = 0; 
          state = 0;
        }
      case 0x0: // 230us
        // 256us gap - ignore and effectively add the time, so we hit the next 520us clock
        break; 

      default:
        // Reset (abort) - out of time range
        bits = 0;
        lastTime = thisTime;
        state = 0;
        break;
    }
    // 104 bits == 52 Manchester encoded bits
    if (bits == 104) {
      if (state) {
        // Serial.println(buf, HEX);
        Serial.print("Status: 0x");
        Serial.print(buf >> 36 & 0xF, HEX);
        Serial.print(", temp1: ");
        Serial.print((int)(buf >> 26 & 0x3FF) - 532);
        Serial.print(", temp2: ");
        Serial.print((int)(buf >> 16 & 0x3FF) - 532);
        Serial.print(", UID: 0x");
        Serial.println((buf & 0xFFFF) ^ calc_lfsr_id(buf >> 16 & 0xFFFFFF), HEX);
      }
      bits = 0;
      state = 0;
    }
  }
}
