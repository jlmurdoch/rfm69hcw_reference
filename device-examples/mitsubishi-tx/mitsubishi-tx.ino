/*
 * Mitsubishi FTC3 / PAR-WT50R-E Transmitter
 *  
 * This uses a RFM69HCW to send heating commands
 *
 * Synopsis: 
 * - Packet mode (variable, no CRC) 
 * - Embed a pre-generated CRC-16 at the end
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
#define FXOSC         32000000.0 // clock speed

// Example packet = (celcius*2 + 128)   
uint8_t packet[] = { 0x1C, 0x8b, 0x00, 0x00, 0x00, 0x06, 0x16, 0xfc,
                     0x44, 0x04, 0x03, 0x10, 0x22, 0xa3, 0xa5, 0x02, 
                     0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                     0x00, 0x00, 0x00, 0x00, 0x33, 0x59, 0x86};

// Burst Write Operation (Section 5.2.1)
uint8_t spi_burst_write(uint8_t addr, uint8_t buf[], size_t size) {
  uint8_t status;

  digitalWrite(RFM69HCW_CS, LOW);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  SPI.transfer(addr | 0x80);
  for (int i = 0; i < size; i++) {
    SPI.transfer(buf[i]);    
  }

  SPI.endTransaction();
  digitalWrite(RFM69HCW_CS, HIGH);
  Serial.println();
  
  return status;
}

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
  while(!(spi_single_read(0x27) & 0x80)); // ReqIrqFlags1: Check ModeReady == 1
  
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
   
  // Transmission Registers
  spi_single_write(0x11, 0x7F); // RegPaLevel: 0x1F=max dBm & 0x60 = PA1+2 on
  
  // Packet Engine Registers
  spi_single_write(0x2D, 0x0C); // RegPreambleLsb: 0xAA x 12
  spi_single_write(0x2E, 0x88); // RegSyncConfig: Syncword is two bytes
  spi_single_write(0x2F, 0x2D); // RegSyncValue: Syncword define = 0x2DD4
  spi_single_write(0x30, 0xD4);  
  spi_single_write(0x37, 0x00); // RegPacketConfig1: Variable, no CRC, no addr
  spi_single_write(0x38, 0x1F); // RegPayloadLength: 31 bytes
  spi_single_write(0x3C, 0x8F); // RegFifoThresh: Rec. default (Section 5.2.2.3)
  
  spi_single_write(0x01, 0x04); // RegOpMode: Standby mode
  while(!(spi_single_read(0x27) & 0x80)); // ReqIrqFlags1: Check ModeReady == 1

  Serial.println("RFM69HCW: Configured");
}

void setup() {
  Serial.begin(115200); // Serial console: e.g. cu -s 115200 -l /dev/ttyACM0
  SPI.begin(); // Create SPI handle
  delay(5000); // Wait for a user connection
    
  pinMode(LED_BUILTIN, OUTPUT); // For blinking the LED on transmission
  
  rfminit(); // Configure the RFM69HCW receiver via SPI
}

void loop() {
  delay(60000); // Wait a minute
  
  digitalWrite(LED_BUILTIN, HIGH); // LED on when transmitting

  // Section 3.3.7 - High Power Settings
  spi_single_write(0x13, 0x0F);    // RegOcp: Disable over-current protection
  spi_single_write(0x5A, 0x5D);    // RegTestPa1: PA1 High Power for +20dBm
  spi_single_write(0x5C, 0x7C);    // RegTestPa2: PA2 High Power for +20dBm

  spi_burst_write(0x00, packet, 31);      // RegFifo: Burst write data into FIFO
  
  spi_single_write(0x01, 0x0C);           // RegOpMode: Transmit Mode
  while(!(spi_single_read(0x28) & 0x08)); // RegIrqFlags2: PacketSent
  
  spi_single_write(0x01, 0x04);           // RegOpMode: Standby Mode
  while(!(spi_single_read(0x27) & 0x80)); // RegIrqFlags1: ModeReady

  // Section 3.3.7 - Safe PA0 / Receive Settings
  spi_single_write(0x13, 0x1A);    // RegOcp: Default / enabled
  spi_single_write(0x5A, 0x55);    // RegTestPa1: PA1 normal for +17dBm
  spi_single_write(0x5C, 0x70);    // RegTestPa2: PA2 normal for +17dBm

  digitalWrite(LED_BUILTIN, LOW);  // LED off when in Sleep / Standby
}
