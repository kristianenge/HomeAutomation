/*
 * Temperature Reading
 *
 * Reads sensor value and publishes through MQTT.
 * MQTT Arduino PubSubClient http://knolleary.net/arduino-client-for-mqtt/
 *
 */
 
#include <SPI.h>
#include <PubSubClient.h>
#include <Ethernet.h>
 
// Pins
const int tempPinIn = 0; // Analog 0 is the input pin
 
// Variables
char* tempC;
char* lightC;
unsigned long time;
char message_buffer[100];

int photocellPin = 2; // the cell and 10K pulldown are connected to a0
int photocellReading; // the analog reading from the sensor divider
int LEDpin = 11; // connect Red LED to pin 11 (PWM pin)
int LEDbrightness; // 
 
// Network Settings
// MAC address of ethernet shield
// Look for it on a sticket at the bottom of the shield. 
// Old Arduino Ethernet Shields or clones may not have a dedicated MAC address. Set any hex values here.
byte MAC_ADDRESS[] = {  0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x54 };
 
// IP address of MQTT server
byte MQTT_SERVER[] = { 192, 168, 1, 136 }; //192.168.1.136
 
EthernetClient ethClient;
PubSubClient client(MQTT_SERVER, 1883, callback, ethClient);
 
void setup()
{  
  // Initilize serial link for debugging
  Serial.begin(9600);
  
  if (Ethernet.begin(MAC_ADDRESS) == 0)
  {
    Serial.println("Failed to configure Ethernet using DHCP");
    return;
  }
}
 
void loop()
{
  if (!client.connected())
  {
    //client.connect("clientID", "mqtt_username", "mqtt_password");
    client.connect("sfo-arduino02");
    client.publish("sfo/arduino02/alive", "I'm alive!");
  }
  photocellReading = analogRead(photocellPin); 

 // We'll have a few threshholds, qualitatively determined
  if (photocellReading < 10) {
    lightC = "Dark";
    Serial.println(" - Dark");
  } else if (photocellReading < 200) {
    lightC = "Dim";
    Serial.println(" - Dim");
  } else if (photocellReading < 500) {
    lightC = "Light";
    Serial.println(" - Light");
  } else if (photocellReading < 800) {
    lightC = "Bright";
    Serial.println(" - Bright");
  } else {
    lightC = "Very bright";
    Serial.println(" - Very bright");
  }
 
  // Publish sensor reading every X milliseconds
  if (millis() > (time + 60000)) {
    time = millis();
    client.publish("arduino02/lightRaw",dtostrf(photocellReading, 5, 2, message_buffer));
    client.publish("arduino02/light",lightC);
  }
    
  // MQTT client loop processing
  client.loop();
}
 
// Handles messages arrived on subscribed topic(s)
void callback(char* topic, byte* payload, unsigned int length) {
}
