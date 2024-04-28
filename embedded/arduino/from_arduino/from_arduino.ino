#include <Wire.h>
#include <Servo.h>
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

// Multiplexer pins
const int muxSIG = A0;
const int muxS0 = 49;
const int muxS1 = 50;
const int muxS2 = 51;
const int muxS3 = 52;
const int muxEN = 53;

// Servo related variables
Servo servos[20];
int servoPins[20] = {26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45};
int positions[20];
bool DEBUG = true;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(25);
  // Initialize I2C buses
  Wire.begin();
  Wire1.begin();

  // Initialize sensors
  // for (int i = 0; i < numSensors; i++) {
  //   sensors[i] = INA3221(addresses[i], wires[i]);
  //   if (!sensors[i].begin()) {
  //     Serial.print("Could not connect to INA");
  //     Serial.print(i);
  //     Serial.println(". Fix and Reboot");
  //   } else {
  //     Serial.print("Found INA");
  //     Serial.print(i);
  //     Serial.print(" at: \t");
  //     Serial.println(sensors[i].getAddress());
  //   }
  // }

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

void readSensor_all(INA3221 &sensor) {
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

void readSensor(INA3221 &sensor) {
  for (int ch = 0; ch < 3; ch++) {
    Serial.print(sensor.getCurrent_mA(ch), 3);
  }
}


int SetMuxChannel(byte channel) {
  digitalWrite(muxS0, bitRead(channel, 0));
  digitalWrite(muxS1, bitRead(channel, 1));
  digitalWrite(muxS2, bitRead(channel, 2));
  digitalWrite(muxS3, bitRead(channel, 3));
}


void loop() {

 digitalWrite(13, LOW);
 if (Serial.available() > 0) {
    digitalWrite(13, HIGH);
    // Read the incoming string
    String inputString = Serial.readString();

    // Split the string into positions
    int index = 0;
    int from = 0;
    int to = inputString.indexOf(',');
    bool okay = false;
    while (to != -1) {
      positions[index++] = inputString.substring(from, to).toInt();
      from = to + 1;
      to = inputString.indexOf(',', from);
    }
    positions[index] = inputString.substring(from).toInt(); // Last position

    // Update servo positions and prepare feedback string
    String feedback;
    feedback += ",";
    for (int i = 0; i <= index; i++) { // Note: Use <= index to iterate through all positions including the last
      if (index != 19) {
        if (DEBUG) {
          Serial.println("There're should be 20 inputs");
        }
        break;
      }
      if (!((positions[i] >= 0) && (positions[i] <= 180))) {
        if (DEBUG) {
          Serial.println("Input string position are not in [0,180]");
        }
        break;
      } else {
        okay = true;
      }
    }

    if (okay == true) {
      for (int i = 0; i <= index; i++) { // Note: Use <= index to iterate through all positions including the last
        servos[i].write(positions[i]); // Move each servo to its new position
        feedback += positions[i];
        if (i < index) {
          feedback += ",";
        } else {
          feedback += ";";
        }
      }

      // INA3221 sensor readings
      for (int i = 0; i < numSensors; i++) {
        for (int ch = 0; ch < 3; ch++) {
          feedback += String(sensors[i].getCurrent_mA(ch));
          if (!( (i == (numSensors-1)) &&  (ch == 2)   )) {
            feedback += ",";
          }
          else {
            feedback += ";";
          }
        }
      }

      for (int i = 0; i < 16; i++) { // Assuming A0 and multiplexer
          // Multiplexer operations
          SetMuxChannel(i);
          int val = analogRead(muxSIG);
          feedback += String(val);
          if (i < 16) {
            feedback += ",";
          }
      }
      for (int i = 1; i < 5; i++) { // Assuming A1 to A4
          int sensorValue = analogRead(i);
          feedback += String(sensorValue);
          if (i < 4) {
            feedback += ",";
          }
      }
    }

    Serial.println(feedback);
 }

}
