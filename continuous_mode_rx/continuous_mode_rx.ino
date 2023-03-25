/*
 * RFM69HCW Continuous Mode Receiving Example
 * Description:
 * - Continuous Mode (raw data) in various receiving conditions
 * - Only receiving, no transmission
 * - FSK or OOK modulation types
 * - Works with 433MHz & 868/915MHz ranges
 * - Bit synchroniser enabled (0x40) or disabled (0x60)
 * - Basic data encoding based on state or size of pulses
 * - DIO0 / G0 / IRQ is redundant
 * - DIO2 / G2 is used for raw data
 */
 
#include <SPI.h>

// As there are no packets, DIO2 is used for DATA, not DIO0
// Adafruit RFM69HCW Breakout / FeatherWing
#define RFM69HCW_G0_IRQ   5   // E - PayloadReady/CrcOk
#define RFM69HCW_CS       6   // D - SPI Chip Select
#define RFM69HCW_RST      9   // C - Reset
#define RFM69HCW_G2_DIO2  11  // A - Data

// SPI0 is on pins 21-25

/*
 * Mitsubishi Thermostat PAR-WT50R-E / PAR-WR51R-E (FSK PWM)
 */
#define MODULATION    FSK
#define FREQ          867.8032
#define FREQDEV       4800
#define BITRATE       9600
#define BANDWIDTH     DCCFREQ_4 << 5 | RXBWMANT_20 << 3 | RXBWEXP_5
#define AFC_BANDWIDTH DCCFREQ_4 << 5 | RXBWMANT_20 << 3 | RXBWEXP_5
#define PULSE_FORMAT    STATE
#define PULSE_SHORT_MIN 80
#define PULSE_LONG_MAX  130
#define PULSE_PREAMBLE  94
#define PULSE_SAMPLE    25
#define BIT_SYNC        0x40

/* 
 * Efergy Engage (FSK PWM)
 */
/*
#define MODULATION    FSK
#define FREQ          433.536
#define FREQDEV       42500
#define BITRATE       15000
#define BANDWIDTH     DCCFREQ_4 << 5 | RXBWMANT_24 << 3 | RXBWEXP_2
#define AFC_BANDWIDTH DCCFREQ_4 << 5 | RXBWMANT_24 << 3 | RXBWEXP_1
#define PULSE_FORMAT    WIDTH
#define PULSE_LONG_MAX  170
#define PULSE_SHORT_MIN 30
#define PULSE_PREAMBLE  100
#define PULSE_SAMPLE    70
#define BIT_SYNC        0x40
*/
/* 
 * RSL366T Hand Radio Remote Control (OOK) 
 */
/*#define MODULATION    OOK
#define FREQ          433.92
#define BITRATE       2356
#define BANDWIDTH     DCCFREQ_4 << 5 | RXBWMANT_16 << 3 | RXBWEXP_2
#define AFC_BANDWIDTH DCCFREQ_4 << 5 | RXBWMANT_16 << 3 | RXBWEXP_2
#define PULSE_FORMAT    BASIC
#define PULSE_SHORT_MIN 350
#define PULSE_LONG_MAX  1400
#define PULSE_PREAMBLE  20
#define BIT_SYNC        0x40
*/

// RFM69HCW clock speed - do not change
#define FXOSC 32000000.0

enum { FSK, OOK };
enum { BASIC, WIDTH, STATE };
enum { DCCFREQ_16, DCCFREQ_8, DCCFREQ_4, DCCFREQ_2, DCCFREQ_1, DCCFREQ_0_5, DCCFREQ_0_25, DCCFREQ_0_125 };
enum { RXBWMANT_16, RXBWMANT_20, RXBWMANT_24 };
enum { RXBWEXP_0, RXBWEXP_1, RXBWEXP_2,  RXBWEXP_3,  RXBWEXP_4, RXBWEXP_5,  RXBWEXP_6,  RXBWEXP_7 };

int pulses = 0;
uint8_t thisValue, lastValue, x, y = 0;
uint32_t thisGap, thisTime, lastTime = 0;
int sample[255];

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
  // Change 0x02 from 0x40 to 0x60 to switch off the bit-synchronizer
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

  if (modulation == OOK) {
    /*
     * Essential OOK Settings:
     * - 0x1B   RegOokPeak:  RSSI threshold set by peak RSSI value
     * - 0x1D   RegOokFix:   RSSI floor is -32dB of peak value
     */
    spi_write(0x1B, 0x40);
    spi_write(0x1D, 32);
    Serial.println(F("Entering OOK mode"));
  } else if (modulation == FSK) {  
    /*
     * Essential FSK Settings:
     * - 0x05-6 RegFdev*:      Set Frequency Deviation      
     * - 0x0B   RegAfcCtrl:    Improved AFC routine 
     * - 0x1E   RegAfcFei:     AFC on on Rx start
     * - 0x29   RegRssiThresh: RSSI Trigger @ -75dB
     * - 0x6F   RegTestDagc:   Fade improvement
     */
    uint16_t freqdev = FREQDEV / (FXOSC / 524288);
    spi_write(0x05, (freqdev >> 8) & 0x3F);
    spi_write(0x06, freqdev & 0xFF);
    spi_write(0x0B, 0x20);
    spi_write(0x1E, 0x04);
    spi_write(0x29, 75 * 2);
    spi_write(0x6F, 0x20);
    Serial.println(F("Entering FSK mode"));
  }
  
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

void loop() {
  thisValue = digitalRead(RFM69HCW_G2_DIO2);

  if (thisValue != lastValue) {
    // Do the necessary calculations / storage now
    thisTime = micros();
    thisGap = thisTime - lastTime;
    lastValue = thisValue;
    lastTime = thisTime;
    
    if (pulses > PULSE_PREAMBLE) {
      switch (PULSE_FORMAT) {
        // Just respond when the pulses are counted
        case BASIC:
          Serial.print("Detected pulses in range: ");
          Serial.println(pulses);
          pulses = 0;
          break;

        /* 
         * Interpret proceeding pulse size (short / long) as 0 or 1 
         * e.g. Manchester encoding
         */
        case WIDTH:
          if (x < PULSE_SAMPLE) {
            // Keep recording data until the limit is hit
            sample[x++] = thisGap;
          } else {
            int offset = (PULSE_LONG_MAX - PULSE_SHORT_MIN) / 2;
            // With those pulses...
            Serial.print("Data: ");
            // increment by 2 rather than 1, if there's a fixed length of 0/1
            for(x=0; x< PULSE_SAMPLE; x+=2) {
              // If it's longer than average
              if (sample[x] > (PULSE_SHORT_MIN + offset))
                // Long = 1
                Serial.print(1);
              else
                // Short = 0 
                Serial.print(0);
            }
            
            // Clean up
            Serial.println("");
            x=0;
            pulses = 0;
          }
          break;

        /* 
         *  Interpret proceeding pulse state (high / low) as 0 or 1
         *  e.g. non-return to zero (NRZ) and other variants
         */
        case STATE:
          if (x < PULSE_SAMPLE) {
            // Keep recording data until the limit is hit
            sample[x++] = thisGap;
          } else {
            int offset = (PULSE_LONG_MAX - PULSE_SHORT_MIN) / 2;
            // With those pulses...
            Serial.print("Data: ");
            for(x=0; x< PULSE_SAMPLE; x++) {
              // Round the gap to the nearest median pulse factor, then divide by median pulse
              for(y = ((sample[x] + offset) / (PULSE_SHORT_MIN + offset)); y>0; y--) {
                // In median pulse size chunks, print a 0 or 1, giving a read-out
                Serial.print((x % 2));
              }
            }

            // Clean up
            Serial.println("");
            x=0;
            pulses = 0;
          }
          break;
      }     
    } else if ((thisGap > PULSE_SHORT_MIN) && (thisGap < PULSE_LONG_MAX)) {
      // Main preamble pulse counter
      pulses++;
    } else {
      // Reset the pulse counter
      pulses=0;
    }
  }
}
