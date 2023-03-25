/*
 * RFM69HCW Basic SPI Version Test
 * Description:
 * - Test the RFM69HCW SPI, checking version register (0x10) equals 0x24
 * - Two LED blinks = pass
 * - One LED blink = fail
 * - Also print the result on serial
 * - Works on RP2040 and ESP32
 * - This will not be an effective test of RST / DIO / Gx pins 
 */

#include <SPI.h>

// Pin to perform Chip Select (6 = ESP32, D6 = RP2040)
#define RFM69HCW_CS 6 // Pin D on Adafruit RFM69HCW Featherwing

void setup() {
  // Setup pins needed for this sketch
  pinMode(RFM69HCW_CS, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  delay(5000);
  Serial.println(F("Serial enabled"));

#if defined(ARDUINO_CHALLENGER_2040_WIFI_RP2040)
  // SPI fix for the broken Challenger 2040 WiFi
  SPI.setRX(PIN_WIRE0_SDA); // Challenger 2040 WiFi fix
#endif
  
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

  // Print version & blink - 2 blinks: pass, 1 blink: fail
  Serial.print(retval, HEX);
  if (retval == 0x24) {
    Serial.println(F(" **PASS**"));
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
  } else
    Serial.println(F(" **FAIL**"));

  // Stop the blink(s)
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);

  // Wait about 10secs before next test
  delay(9900);
}
