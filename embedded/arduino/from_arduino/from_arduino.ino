#include <Wire.h>
#include <Servo.h>
#include "INA3221.h"

// Constants for the number of sensors and their addresses
const uint8_t numSensors = 8;
const uint8_t addresses[numSensors] = {0x40, 0x41, 0x42, 0x43, 0x40, 0x41, 0x42, 0x43};
TwoWire* wires[numSensors] = {&Wire, &Wire, &Wire, &Wire, &Wire1, &Wire1, &Wire1, &Wire1};

// Array to hold sensor instances
INA3221 sensors[numSensors];

// Multiplexer pins
const int muxSIG = A0;
const int muxS0 = 49;
const int muxS1 = 50;
const int muxS2 = 51;
const int muxS3 = 52;
const int muxEN = 53;

// Servo related variables
Servo servos[20];
int servoPins[20] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
int positions[20];
bool DEBUG = true;
int analog_n = 12;

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
    sensors[i] = INA3221(addresses[i], wires[i]);
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

  // Initialize multiplexer pins
  pinMode(muxEN, OUTPUT);
  pinMode(muxSIG, OUTPUT);
  pinMode(muxS0, OUTPUT);
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(muxS3, OUTPUT);
  digitalWrite(muxEN, LOW);

  // Initialize servos
  for (int i = 0; i < 20; i++) {
    servos[i].attach(servoPins[i]);
  }
  pinMode(13, OUTPUT);
  analogReadResolution(12);
}

void loop() {
  // INA3221 sensor readings
  Serial.println("\nReading INA3221 Sensors");
  for (int i = 0; i < numSensors; i++) {
    Serial.println("------------------------");
    readSensor(sensors[i]);
  }

  // Multiplexer operations
  SetMuxChannel(5);
  int val = analogRead(muxSIG);
  Serial.println(val);

  // Servo operations based on serial input
  if (Serial.available() > 0) {
    String inputString = Serial.readString();
    // Process inputString to update servo positions...
    // (Include the servo processing code here)
  }

  delay(1000); // Adjust delay as needed
}

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

int SetMuxChannel(byte channel) {
  digitalWrite(muxS0, bitRead(channel, 0));
  digitalWrite(muxS1, bitRead(channel, 1));
  digitalWrite(muxS2, bitRead(channel, 2));
  digitalWrite(muxS3, bitRead(channel, 3));
}
