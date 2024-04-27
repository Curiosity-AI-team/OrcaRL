#include "INA3221.h"

// Constants for the number of sensors and their addresses
const uint8_t numSensors = 8;
const uint8_t addresses[numSensors] = {0x40, 0x41, 0x42, 0x43, 0x40, 0x41, 0x42, 0x43};
TwoWire* wires[numSensors] = {&Wire, &Wire, &Wire, &Wire, &Wire1, &Wire1, &Wire1, &Wire1};

// Array to hold sensor instances
INA3221 sensors[numSensors] = {
    INA3221(addresses[0], wires[0]),
    INA3221(addresses[1], wires[1]),
    INA3221(addresses[2], wires[2]),
    INA3221(addresses[3], wires[3]),
    INA3221(addresses[4], wires[4]),
    INA3221(addresses[5], wires[5]),
    INA3221(addresses[6], wires[6]),
    INA3221(addresses[7], wires[7])
};

void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("INA3221_LIB_VERSION: ");
  Serial.println(INA3221_LIB_VERSION);
  Serial.println();

  // Initialize I2C buses
  Wire.begin();
  Wire1.begin();

  // Initialize sensors
  for (int i = 0; i < numSensors; i++) {
    if (!sensors[i].begin()) {
      Serial.print("Could not connect to INA");
      Serial.print(i);
      Serial.println(". Fix and Reboot");
    } else {
      Serial.print("Found INA");
      Serial.print(i);
      Serial.print(" at: \t");
      Serial.println(sensors[i].getAddress());
    }
  }
}

void loop() {
  // Read and print values from each sensor
  Serial.println("\nReading INA3221 Sensors");
  for (int i = 0; i < numSensors; i++) {
    Serial.println("------------------------");
    readSensor(sensors[i]);
  }
  delay(1000);
}

// This function could be used to read and print sensor data
void readSensor(INA3221 &sensor) {
  Serial.println("\nCHAN\tBUS\tSHUNT\tCURRENT\tPOWER");
  for (int ch = 0; ch < 3; ch++) {
    Serial.print(ch);
    Serial.print("\t");
    Serial.print(sensor.getBusVoltage(ch), 3);
    Serial.print("\t");
    Serial.print(sensor.getShuntVoltage_mV(ch), 3);
    Serial.print("\t");
    Serial.print(sensor.getCurrent_mA(ch), 3);
    Serial.print("\t");
    Serial.print(sensor.getPower_mW(ch), 3);
    Serial.println();
  }
}
