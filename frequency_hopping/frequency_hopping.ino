#include <SPI.h>

// Adafruit RFM69HCW Breakout / FeatherWing
#define RFM69HCW_G0_IRQ   5   // E - PayloadReady/CrcOK
#define RFM69HCW_CS       6   // D - SPI Chip Select
#define RFM69HCW_RST      9   // C - Reset
#define RFM69HCW_G2_DIO2  11  // A - Data

// Thsi example was able to diagnose there was two frequencies 433.647 & 433.812 
#define BASEFREQ      433.500
#define BITRATE       2000
// Minimum bandwidth to cover both frequencies - 166.7MHz
#define BANDWIDTH     DCCFREQ_4 << 5 | RXBWMANT_24 << 3 | RXBWEXP_0
#define AFC_BANDWIDTH DCCFREQ_4 << 5 | RXBWMANT_24 << 3 | RXBWEXP_0

#define PULSE_LEAD  0x1 // HIGH of 500ms (488-544)
#define PULSE_SHORT 0x3 // LOW of 1000ms (980-1012)
#define PULSE_LONG  0x7 // LOW of 2000ms (1956-1996)
#define PULSE_GAP   0xF // LOW of 4000ms 
#define BIT_COUNT   36 

#define HOP_TIME    90000 // 90secs between hops
#define FREQ_LIMIT  0.5   // +500kHz - hop range

// RFM69HCW clock speed - do not change
#define FXOSC 32000000.0

// Bandwidth Macro Enumeration
enum { DCCFREQ_16, DCCFREQ_8, DCCFREQ_4, DCCFREQ_2, DCCFREQ_1, DCCFREQ_0_5, DCCFREQ_0_25, DCCFREQ_0_125 };
enum { RXBWMANT_16, RXBWMANT_20, RXBWMANT_24 };
enum { RXBWEXP_0, RXBWEXP_1, RXBWEXP_2,  RXBWEXP_3,  RXBWEXP_4, RXBWEXP_5,  RXBWEXP_6,  RXBWEXP_7 };
 
uint8_t thisValue, lastValue, thisGap, pulses = 0;
uint32_t thisTime, lastTime, hopTime = 0;
float freqoff = 0.000;

uint64_t data;

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
  // Setup pins other than SPI, VDD and ground
  pinMode(RFM69HCW_CS, OUTPUT);
  pinMode(RFM69HCW_RST, OUTPUT);
  pinMode(RFM69HCW_G2_DIO2, INPUT);

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
  delay(10);
  
  // RegOpMode: Standby Mode & wait
  spi_write(0x01, 0x04);
  while(!(spi_read(0x27)) & 0x80);
  
  // Continuous, OOK
  spi_write(0x02, 0x48);
   
  // RegBitrate*: Bitrate needed for Bit Synchronizer
  uint16_t bitrate = FXOSC / BITRATE;
  spi_write(0x03, (bitrate >> 8) & 0xFF);
  spi_write(0x04, bitrate & 0xFF);

  // RegFrf*: Frequency across 3 bytes  
  uint32_t freqrf = ((BASEFREQ + freqoff) * 1000000) / (FXOSC / 524288);
  spi_write(0x07, (freqrf >> 16) & 0xFF);
  spi_write(0x08, (freqrf >> 8) & 0xFF);
  spi_write(0x09, freqrf & 0xFF);

  // 50 Ohm LNA
  spi_write(0x18, 0x88);

  // RegRxBw: Tuned bandwidth (narrow)
  spi_write(0x19, BANDWIDTH);
 
  // AFC bandwidth (wider)
  spi_write(0x1A, AFC_BANDWIDTH);

  // OOK Peak
  spi_write(0x1B, 0x40);
  
  // Threshold in -dB. Valid range for the sensor: -1 to -99dB
  spi_write(0x1D, 1);

  // Fast tail off with the DAGC - OOK specific
  spi_write(0x6F, 0x30);

  // RegOpMode: Receiving mode, auto sequencer disabled
  spi_write(0x01, 0x10);
  while(!(spi_read(0x27)) & 0x80);

  SPI.endTransaction();
  SPI.end();

  Serial.println();
  Serial.print("Frequency: ");
  Serial.print(BASEFREQ + freqoff, 3);
}

void setup() {
  // Serial console: e.g. cu -s 115200 -l /dev/ttyACM0
  Serial.begin(115200);
  delay(5000);
  
  // Configure the receiver
  rfminit();
}

void loop() {
  thisValue = digitalRead(RFM69HCW_G2_DIO2);
  thisTime = micros();
  
  if (thisValue != lastValue) {
    // Do the necessary calculations / storage now
    thisGap = (thisTime - lastTime - 127) >> 8;
    lastTime = thisTime;
    
    // Rising edge after x millisecs + with proceeding 500ms pulse
    switch (thisGap) {
      case PULSE_SHORT: // 1000 ms = 0
      case PULSE_LONG: // 2000 ms = 1
        if (thisValue){
          // Make room for a new bit
          data <<= 1;
          // Set data to whatever the 3rd bit is
          data |= (!(~thisGap & 0x04));
          // Increment the counter
          pulses++;
          break;
        }

      case PULSE_LEAD:
        // Wait for next bit if it's a 500 lead pulse
        if (~thisValue) {
          break;
        }

      default:
        // Reset if we get to here
        data = 0;
        pulses = 0;
    }

    if (pulses == BIT_COUNT){
      Serial.println(data, BIN);
      data = 0;
      pulses = 0;
    } 
  }

  if ((millis() - hopTime) >= HOP_TIME) {
    hopTime = millis();
    freqoff += 0.01;
            
    while (freqoff > FREQ_LIMIT) { true; }
    rfminit();
  }
}
