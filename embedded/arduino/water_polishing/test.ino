#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

const char* ssid     = "MTS_GPON_b1ae68";
const char* password = "Jba4b5a3";

//const char* ssid     = "Galaxy S10";
//const char* password = "123456789";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 10 *  3600, 60000);

int ledPin = 12; // Change this to your actual GPIO pin
int motorPin = 14; // Change this to your actual GPIO pin

void setup(){
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    pinMode(motorPin, OUTPUT);
  
    digitalWrite(ledPin, HIGH);
    digitalWrite(motorPin, HIGH);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    
    Serial.println("Connected to WiFi");
    timeClient.begin();
}

void loop(){
    timeClient.update();
    int date = timeClient.getDay();
    int hours = timeClient.getHours();
    int minutes = timeClient.getMinutes();
    int seconds = timeClient.getSeconds();
    
    //  Serial.printf("Current date: %s\n", formattedDate.c_str());
    Serial.printf("Current time: %02d:%02d:%02d:%02d\n", date, hours, minutes, seconds);
  
    if ((date == 2 || date == 5) && (hours == 1 && minutes == 1))
    {
        Serial.printf("Time to watering\n");
        digitalWrite(motorPin, LOW);
    
        delay(2000);
        
        digitalWrite(motorPin, HIGH);
        Serial.printf("Motor off\n");
        
        delay(500000); // Wait for 500 second
    }
    else
    {
        digitalWrite(motorPin, HIGH); // Turn the motor off
    }
  
    if (hours > 0 && hours < 14)
    {
        Serial.printf("Time to flashing\n");
        digitalWrite(ledPin, LOW); // Turn on the LED
    }
    else
    {
        digitalWrite(ledPin, HIGH); // Turn off the LED
    }
  
    delay(58000); // Wait for 58 second
  
}