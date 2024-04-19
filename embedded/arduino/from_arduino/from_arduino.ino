#include <Servo.h>

Servo servos[20]; // Create an array to hold 20 servo objects
int servoPins[20] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21}; // PWM pins
int positions[20]; // Array to store positions for each servo
bool DEBUG = true;
int debug_c = 0;
int analog_n = 12;

void setup() {
 Serial.begin(115200); // Start serial communication at 9600 baud rate
 Serial.setTimeout(25);
 for (int i = 0; i < 20; i++) {
    servos[i].attach(servoPins[i]); // Attach each servo to its pin
 }
 pinMode(13, OUTPUT);
 analogReadResolution(12);
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
          feedback += ";"; // End the feedback string with a newline character
        }
      }
    }
    
    for (int i = 0; i < analog_n; i++) { // Assuming A0 to A5
        int sensorValue = analogRead(i);
        feedback += String(sensorValue);
        if (i < analog_n) {
          feedback += ",";
        }
    }
    Serial.println(feedback);
 }

}
