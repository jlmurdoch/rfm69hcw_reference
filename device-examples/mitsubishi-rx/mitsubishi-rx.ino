/*
 * Mitsubishi FTC3 / PAR-WT50R-E Receiver
 *
 * This uses a RFM69HCW to collect heating data
 * 
 * Synopsis
 * - Packet mode (variable, accept & ignore the CRC-16)
 * - Manual / visual checking of CRCs
 * 
 * gqrx: 49.6 LNA, Demod off, AGC: Fast, 240k input rate 
 * rtl_433 -f 868.276M -s 384k -Y minmax \
 *         -X "n=x,m=FSK_PCM,l=65,s=65,r=9000,preamble=0x2dd4"
 *
 * PACKET FORMAT 
 * Preamble: 555555555555555555555555 (12 bytes)
 * Syncword: 2DD4
 * Pkt Size: 1C (28 bytes)
 * Pkt Addr: 8B/0B (From RC / From FTC)
 *   UID:    06EC / 0762 (FTCID / RCID) 
 *   Dest ID:  00 (FTCID == 00, RCID >= 01)
 *   SourceID: 02
 *   Command?: 16FC
 *     Message:     4304031021A50200000000000000000000000000
 *   Msg CRC:  DE (CRC-8 Two's compliment: 0x100 - sum-of-bytes & 0xFF)
 * Pkt CRC:  6EB7 (CRC-16 BUYPASS)
 *
 * Example output of a RC / FTC comms of thermostat status check-in (43 > 63):
 * PS PA Base D  S  Cmd? Message                                  MC PCRC
 * 1C 8B 0762 00 02 16FC 4304031021A50200000000000000000000000000 DE 6EB7
 * 1C 0B 06EC 02 00 16FC 63040310000102A401000004010202FF00BD0000 19 3384
 */

#include <SPI.h>

// Adafruit RFM69HCW Breakout / FeatherWing
#define RFM69HCW_CS   6 // D - SPI Chip Select
#define RFM69HCW_RST  9 // C - Reset

// Transmission profile
#define MODULATION    FSK
#define BASEFREQ      868.299 // MHz
#define BITRATE       9600    // 65.1us pulses
#define FREQDEV       4800    // Usually bitrate divided by 2
#define FXOSC         32000000.0 // Clock speed

// Section 3.4.6 = Bandwidth = 12.5Khz
#define RXBW  DCCFREQ_4 << 5 | RXBWMANT_20 << 3 | RXBWEXP_5
#define AFCBW DCCFREQ_4 << 5 | RXBWMANT_20 << 3 | RXBWEXP_5

// Bandwidth Macro Enumeration
enum { DCCFREQ_16, DCCFREQ_8, DCCFREQ_4, DCCFREQ_2, 
       DCCFREQ_1, DCCFREQ_0_5, DCCFREQ_0_25, DCCFREQ_0_125 };
enum { RXBWMANT_16, RXBWMANT_20, RXBWMANT_24 };
enum { RXBWEXP_0, RXBWEXP_1, RXBWEXP_2,  RXBWEXP_3,  
       RXBWEXP_4, RXBWEXP_5,  RXBWEXP_6,  RXBWEXP_7 };

uint8_t packet[32];

// Single Read/Write Operation
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

// Single Write: Apply the wnr bit to write (Section 5.2.1) 
uint8_t spi_single_write(uint8_t addr, uint8_t value) {
  return spi_cmd(addr | 0x80, value);
}

// Single Read: Write zero to read
uint8_t spi_single_read(uint8_t addr) {
  return spi_cmd(addr, 0x00);
}

void rfminit() {
  // Setup pins other than SPI, VDD and ground
  pinMode(RFM69HCW_CS, OUTPUT);
  pinMode(RFM69HCW_RST, OUTPUT);
  // pinMode(RFM69HCW_G0_IRQ, INPUT_PULLDOWN); // for an LED

  // Section 5.2.1 - SPI Interface
  digitalWrite(RFM69HCW_CS, HIGH);
  
  // Section 7.2.2 - Manual Reset
  digitalWrite(RFM69HCW_RST, HIGH);
  delayMicroseconds(100);
  digitalWrite(RFM69HCW_RST, LOW);
  // Spec says 5ms, but takes longer
  delay(10);

  // RegVersion: if it's not 0x24, something is wrong
  if(spi_single_read(0x10) == 0x24)
    Serial.println("RFM69HCW: Chip found");
  else 
    Serial.println("RFM69HCW: ERROR - Chip not found!");
  
  spi_single_write(0x01, 0x04); // RegOpMode: Standby Mode
  while(!(spi_single_read(0x27) & 0x80)); // RegIrqFlags1: Check ModeReady == 1
    
  spi_single_write(0x02, 0x01); // RegDataModul: Packet, FSK, Gaussian shaping
   
  // RegBitrate*: Bitrate needed for Bit Synchronizer
  uint16_t bitrate = FXOSC / BITRATE;
  spi_single_write(0x03, (bitrate >> 8) & 0xFF);
  spi_single_write(0x04, bitrate & 0xFF);
 
  // RegFDev*: Frequency deviation over two bytes
  uint16_t freqdev = FREQDEV / (FXOSC / 524288);
  spi_single_write(0x05, (freqdev >> 8) & 0x3F);
  spi_single_write(0x06, freqdev & 0xFF);

  // RegFrf*: Reference frequency across 3 bytes  
  uint32_t freqrf = (BASEFREQ * 1000000) / (FXOSC / 524288);
  spi_single_write(0x07, (freqrf >> 16) & 0xFF);
  spi_single_write(0x08, (freqrf >> 8) & 0xFF);
  spi_single_write(0x09, freqrf & 0xFF);

  spi_single_write(0x0B, 0x20);  // RegAFCCtrl: Use AFC Improved

  // Receiver Registers
  spi_single_write(0x19, RXBW);  // RegRxBw: Receiver bandwidth
  spi_single_write(0x1A, AFCBW); // RegAFCBw: AFC bandwidth
  spi_single_write(0x1E, 0x04);  // RegAFCFei: AFC On at start

  // Interrupt / Pin Mapping Registers
  spi_single_write(0x29, 80*2); // RegRSSIThresh: dBm x 2 - errors at 80dBm

  // Packet Engine Registers
  spi_single_write(0x2E, 0x88); // RegSyncConfig: Syncword is two bytes
  spi_single_write(0x2F, 0x2D); // RegSyncValue: Syncword define = 0x2DD4
  spi_single_write(0x30, 0xD4);
  spi_single_write(0x37, 0x00); // RegPacketConfig1: Variable, no CRC, no addr
  spi_single_write(0x38, 0x1F); // RegPayloadLength: 31 bytes
  spi_single_write(0x39, 0x8B); // RegNodeAddress: Node Address = 0x8B
  spi_single_write(0x3A, 0x0B); // RegBroadcastAddress: Broadcast Address = 0x0B

  // Test Registers
  spi_single_write(0x6F, 0x20); // RegTestDAGC: Idx < 2 = 0x20, Idx > 2 = 0x20
  
  spi_single_write(0x01, 0x10); // RegOpMode: Receiving mode, auto seq disabled
  while(!(spi_single_read(0x27) & 0x80)); // ReqIrqFlags1: Check ModeReady == 1

  Serial.println("RFM69HCW: Configured");
}

// CRC-8: Two's Compliment
uint8_t crc8(uint8_t buf[], int begin, int end) {
  uint8_t crc = 0;

  for (int x = begin; x < end; x++)
      crc -= buf[x]; // Keep subtracting and rolling over

  return crc;
}

// CRC-16: BUYPASS
uint16_t crc16(uint8_t buf[], int begin, int end) {
  uint16_t crc = 0;

  for (int x = begin; x < end; x++) {
    crc ^= buf[x] << 8; // XOR byte onto MSB
    for (int y = 0; y < 8; y++) {
      if (crc & 0x8000) // If the biggest bit is 0x1
        crc = (crc << 1) ^ 0x8005; // Shift by 1 and XOR against polynomial
      else
        crc <<= 1; // Otherwise, shift by 1
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200); // Serial console: e.g. cu -s 115200 -l /dev/ttyACM0
  SPI.begin(); // Create SPI handle
  delay(5000); // Wait for a user connection

  rfminit(); // Configure the RFM69HCW receiver via SPI
}

void loop() {
  /*
   * Two options here: 
   * - Check SPI for PayloadReady (blocking, no additional config)
   * - Check DIO0 for PayloadReady (efficient, wiring & pin reconfig required)
   *
   * We use the SPI version for testing.
   */
  int len = 0;

  // RegIrqFlags2: PayloadReady, as we need all the bytes
  if (spi_single_read(0x28) & 0x04) {
    while(spi_single_read(0x28) & 0x40) // RegIrqFlags2: FifoNotEmpty
      packet[len++] = spi_single_read(0x00); // RegFIFO to memory until empty

    // Print the whole packet (without Preamble + Syncword)
    char octet[5];
    for (int x = 0; x < 31; x++) {
      sprintf(octet, "%.2X", packet[x]);
      Serial.print(octet);
      // Show a break between the packet header, message and CRC's
      if (x == 7 || x == 27 || x == 28)
        Serial.print(" ");
    }
    Serial.print(" crc ");

    // Print both CRCs
    sprintf(octet, "%.2X", crc8(packet, 8, 28));
    Serial.print(octet);
    Serial.print(" ");
    sprintf(octet, "%.4X", crc16(packet, 0, 29));
    Serial.print(octet);
    Serial.println();
  }
}
