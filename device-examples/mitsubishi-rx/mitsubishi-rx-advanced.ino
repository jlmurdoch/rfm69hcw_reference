/*
 * Mitsubishi FTC3 / PAR-WT50R-E Receiver 
 *
 * This uses a RFM69HCW to collect heating data
 *
 * Advanced version for ESP32 / RP2040:
 * - Uses interrupts and multicore (transparent on ESP32)
 * - RP2040: earlephilpower/arduino-pico 3.1.0 - previous versions had WiFi issues
 * - ESP32:  espressif/arduino-esp32 2.0.5
 * 
 * Synopsis
 * - Packet mode (variable size, ignore HW check of CRC-16)
 * - Do manual checks of both CRCs
 * - Filter using remote ID
 * - Send using syslog
 * 
 * GQRX: 49.6 LNA, Demod off, AGC: Fast, 240k input rate 
 * rtl_433 -f 868.276M -s 384k -Y minmax -X "n=x,m=FSK_PCM,l=65,s=65,r=9000,preamble=0x2dd4"
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

#if defined(ARDUINO_CHALLENGER_2040_WIFI_RP2040)
#include <WiFiEspAT.h>
#else
#include <WiFi.h>
#endif

#if defined(ARDUINO_ARCH_ESP32)
// No FIFO, so we use a queue
#include "queue.h"
QueueHandle_t xLongQueue;
#endif 

// Adafruit RFM69HCW Breakout / FeatherWing
#define RFM69HCW_RST    9 // C - Reset
#define RFM69HCW_CS     6 // D - SPI Chip Select
#define RFM69HCW_G0_IRQ 5 // E - PayloadReady

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
enum { DCCFREQ_16, DCCFREQ_8, DCCFREQ_4, DCCFREQ_2, DCCFREQ_1, DCCFREQ_0_5, DCCFREQ_0_25, DCCFREQ_0_125 };
enum { RXBWMANT_16, RXBWMANT_20, RXBWMANT_24 };
enum { RXBWEXP_0, RXBWEXP_1, RXBWEXP_2,  RXBWEXP_3,  RXBWEXP_4, RXBWEXP_5,  RXBWEXP_6,  RXBWEXP_7 };

// Wireless
#define SSID "myssid"
#define PASS "stupidpassword"
// Syslog
#define HOST "192.168.1.219"
#define PORT 514
// NTP
#define SNTP_SERVER_A "0.uk.pool.ntp.org"
#define SNTP_SERVER_B "time.nist.org"

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
  spi_single_write(0x25, 0x40);  // DIO0 = PayloadReady
  spi_single_write(0x29, 80*2);  // RegRSSIThresh: dBm x 2 - errors at 80dBm

  // Packet Engine Registers
  spi_single_write(0x2E, 0x88);  // RegSyncConfig: Syncword is two bytes
  spi_single_write(0x2F, 0x2D);  // RegSyncValue: Syncword define = 0x2DD4
  spi_single_write(0x30, 0xD4);
  spi_single_write(0x37, 0x08);  // RegPacketConfig1: Variable, expect & ignore CRC, no addr filter
  spi_single_write(0x38, 0x1F);  // RegPayloadLength: 31 bytes
  spi_single_write(0x39, 0x8B);  // RegNodeAddress: Node Address byte = 0x8B
  spi_single_write(0x3A, 0x0B);  // RegBroadcastAddress: Broadcast Address byte = 0x0B
  spi_single_write(0x3D, 0x10);  // Short wait, manual reset, no encryption

  // Test Registers
  spi_single_write(0x6F, 0x20);  // RegTestDAGC: FM ModIdx < 2 = 0x20, FM ModIdx > 2 = 0x20
  
  spi_single_write(0x01, 0x10);  // RegOpMode: Receiving mode, auto sequencer disabled
  while(!(spi_single_read(0x27) & 0x80)); // ReqIrqFlags1: Check ModeReady == 1
}

// CRC-8: Two's Compliment
uint8_t crc8(uint8_t buf[], int begin, int end) {
  uint8_t crc = 0;

  for (int x = begin; x < end; x++)
      crc -= buf[x]; // Keep subtracting and rolling over

  return crc;
}

// CRC-16: BUY-PASS
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

/*
 * Send data using TCP Syslog
 * Here is an example syslog-ng config:
 *   source s_net { tcp( ip(192.168.0.1) port(514) ); };
 *   destination d_iot { file("/var/log/iot.log"); };
 *   log { source(s_net); destination(d_iot); };
*/
void sendSyslog(uint8_t packet[32]) {
  time_t currentTime;
  struct tm timeinfo;
  char logline[109];
  WiFiClient client;

  // Generate timestamp - RFC3339: e.g. 2023-03-04T16:50:10+00:00
  currentTime = time(NULL);
  gmtime_r(&currentTime, &timeinfo);
  strftime(logline, sizeof(logline), "<34>%FT%T%z ecodan packet:", &timeinfo);

  // Append the packet payload only
  for (int x=8; x<29; x++)
    sprintf(logline + strlen(logline), " %.2X", packet[x]);

  if (WiFi.status() != WL_CONNECTED) {
    configWireless();
  }

  if (!client.connect(HOST, PORT)) {
    Serial.println(F("connection failed"));
    client.flush();
    client.stop();
    return;
  }

  // Send the syslog payload
  if (client.connected()) {
    client.println(logline);
  } else {
    Serial.println(F("sending failed"));
  }
  
  // Clean up and close out the connection
  client.flush();
  client.stop();
}

void configWireless () {
  time_t currentTime;

  //Special Startup for the Challenger 2040
#if defined(ARDUINO_CHALLENGER_2040_WIFI_RP2040)  
  Serial2.begin(115200);
  WiFi.init(Serial2, PIN_ESP_RST);
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("Wireless: No module found!!!"));
    while (true);
  }
  WiFi.disconnect();
  WiFi.setPersistent();
  WiFi.endAP();
#endif

  // Standard WiFi Setup
  Serial.print(F("Wireless: "));
  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, true);
    delay(500);
    digitalWrite(LED_BUILTIN, false);
    Serial.print(F("."));
  }
  Serial.println(F("connected"));

  // RP2040 & ESP32 NTP procedure
  Serial.print(F("NTP Time: "));
  configTime(300, 0, SNTP_SERVER_A, SNTP_SERVER_B);
  currentTime = time(NULL);
  while (currentTime < 86400) {
    digitalWrite(LED_BUILTIN, true);
    delay(500);
    digitalWrite(LED_BUILTIN, false);
    currentTime = time(NULL);
    Serial.print(F("."));
  }
  Serial.println(F("synced"));
}

// Interrupt in RAM for fast operation

#if defined(ARDUINO_ARCH_ESP32)
void IRAM_ATTR payloadReady() // ESP32
#else
void __not_in_flash_func(payloadReady)() // RP2040
#endif
{
  // Verify payloadReady is still active
  if (spi_single_read(0x28) & 0x04) {
    int len = 0;
    uint32_t buf = 0;

    // Put into Standby (stop RX)
    spi_single_write(0x01, 0x04);
    while(!(spi_single_read(0x27) & 0x80));

    // Read FIFO
    while(spi_single_read(0x28) & 0x40) { // RegIrqFlags2: FifoNotEmpty
      // Read each 8-bit value from LSB to MSB
      buf |= (spi_single_read(0x00) << ((len++ % 4) * 8));
      // Push when 32-bit value full
      if ((len % 4) == 0) {
        uint32_t value = buf;
#if defined(ARDUINO_ARCH_ESP32) 
        xQueueSendToBackFromISR(xLongQueue, &value, NULL);
#else
        rp2040.fifo.push(value);
#endif  
        buf = 0;
      }
      // Or push and break when complete packet is read
      if (len == 31) { 
        uint32_t value = buf;
#if defined(ARDUINO_ARCH_ESP32) 
        xQueueSendToBackFromISR(xLongQueue, &value, NULL);
#else
        rp2040.fifo.push(value);
#endif  
        break;
      }
    }

    // Reset if still in Payloadready
    if (spi_single_read(0x28) & 0x04) {
      spi_single_write(0x3D, 0x14);
    }
    // Set DIO0 to PayloadReady
    spi_single_write(0x25, 0x40);

    // Return to RX
    spi_single_write(0x01, 0x10);
    while(!(spi_single_read(0x27) & 0x80));
  }
}

void setup1() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  SPI.begin(); // Create SPI handle for talking to the RFM
  
  // Set up the payloadReady interrupt
  pinMode(RFM69HCW_G0_IRQ, INPUT);
  int irq = digitalPinToInterrupt(RFM69HCW_G0_IRQ);
  // SPI.usingInterrupt(irq); // Not implemented on RP2040 or ESP32
  attachInterrupt(irq, payloadReady, RISING); // Whenever we get a payloadReady, process it
  rfminit(); // Configure the RFM69HCW receiver via SPI
}

void loop1() {
  // Do nothing, as this core is completely interrupt driven
}

void setup() {
  Serial.begin(115200); // Serial console: e.g. cu -s 115200 -l /dev/ttyACM0
  delay(5000); // Wait for a user connection
  configWireless(); // WiFi can only work on Core0  

#if defined(ARDUINO_ARCH_ESP32)
  xLongQueue = xQueueCreate(16, sizeof(uint32_t));
  setup1();
#endif  
}

void loop() { 
  // Listen for 8 x 32-bit messages on the queue / FIFO
#if defined(ARDUINO_ARCH_ESP32) 
  if (uxQueueMessagesWaiting(xLongQueue) >= 8) {
#else
  if (rp2040.fifo.available() >= 8) {
#endif
    uint32_t buf;
    uint8_t pkt[32];
    
    // Put the LED on while we have a packet to process
    digitalWrite(LED_BUILTIN, true);

    // Process all 8 x 32-bit values
    buf = 0;
    for (int i=0; i < 8; i++) {
    // Pick values off the queue / FIFO
#if defined(ARDUINO_ARCH_ESP32) 
      xQueueReceive(xLongQueue, &buf, (TickType_t) 10);
#else
      buf = rp2040.fifo.pop();
#endif
      // Split each 32-bit values into 4 x 8-bit elements
      for (int y=0; y < 4; y++) {
        pkt[y + (i * 4)] = (buf >> (y * 8)) & 0xFF;
      }
    }

    // If the source or destination is remote 6 and the CRC is good
    if ((pkt[4] == 0x06 || pkt[5] == 0x06) // Remote ID
        && crc8(pkt, 8, 28) == pkt[28] // Packet CRC
        && crc16(pkt, 0, 29) == ((uint16_t)pkt[29] << 8 | pkt[30])) // Payload CRC
    {  
      // Send the message as syslog 
      sendSyslog(pkt);
    }

    // Switch off the LED
    digitalWrite(LED_BUILTIN, false);
  }
}
