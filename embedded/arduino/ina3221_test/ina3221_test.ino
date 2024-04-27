#include <Wire.h>

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire
#define WIRE1 Wire1

void setup() {
  WIRE.begin();
  WIRE1.begin();

  Serial.begin(115200);
  while (!Serial)
     delay(10);
  Serial.println("\nI2C Scanner");
}

void wire()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning Wire...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println(" !");
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void wire1()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning Wire1...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    WIRE1.beginTransmission(address);
    error = WIRE1.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println(" !");
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void loop() {
  wire();
  wire1();

  delay(5000);           // wait 5 seconds for next scan
}
