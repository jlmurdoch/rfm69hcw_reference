/*
 * RFM69HCW Basic SPI Version Test
 * Description:
 * - Test the RFM69HCW SPI, checking the version register equals 0x24
 */

#include <SPI.h>

// Adafruit RFM69HCW Breakout / FeatherWing
#define RFM69HCW_G0_IRQ   D5   // E - PayloadReady/CrcOK
#define RFM69HCW_CS       D6   // D - SPI Chip Select
#define RFM69HCW_RST      D9   // C - Reset
#define RFM69HCW_G2_DIO2  D11  // A - Data

void setup() {
  // Setup pins needed for this sketch
  pinMode(RFM69HCW_CS, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
 
  Serial.begin(115200);
  delay(5000);
  Serial.println(F("Serial enabled"));

  // SPI fix for the broken Challenger 2040 WiFi
  SPI.setRX(PIN_WIRE0_SDA);
  
  // SPI Slave Select, Active is LOW
  digitalWrite(RFM69HCW_CS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  Serial.println(F("SPI enabled"));
}

void loop() {
  // Turn on LED to show activity
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Value should return 0x24
  Serial.print(F("Version register (0x10) should read 0x24. Actual value: 0x"));
  
  // SPI Activity Start
  digitalWrite(RFM69HCW_CS, LOW);
  // Select the version register
  SPI.transfer(0x10);
  // Read the selected register
  char retval = SPI.transfer(0x00);
  // SPI Activity End
  digitalWrite(RFM69HCW_CS, HIGH);

  // Print version and evaluate it
  Serial.print(retval, HEX);
  if (retval == 0x24)
    Serial.println(F(" **PASS**"));
  else
    Serial.println(F(" **FAIL**"));
  
  delay(1000);

  // Switch off the LED
  digitalWrite(LED_BUILTIN, LOW); 
  delay(9000);
}
