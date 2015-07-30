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
 
// Variables
char* tempC;
char* lightC;
char* door1C;
char* door2C;
char* door3C;
char* door4C;

unsigned long time;
char message_buffer[100];
int Door_Led_Pin = 13;   // choose the pin for the LED
int Door_Sensor_Pin1 = 3; // choose the Door_Sensor_Pin
int Door_Sensor_Pin2 = 4; // choose the Door_Sensor_Pin
int Door_Sensor_Pin3 = 5; // choose the Door_Sensor_Pin
int Door_Sensor_Pin4 = 6; // choose the Door_Sensor_Pin



int val = 0;   // variable for reading the 
unsigned int oldvalue  = 0xFFFF;


int photocellPin = 2; // the cell and 10K pulldown are connected to a0
int photocellReading; // the analog reading from the sensor divider
 
// Network Settings
// MAC address of ethernet shield
// Look for it on a sticket at the bottom of the shield. 
// Old Arduino Ethernet Shields or clones may not have a dedicated MAC address. Set any hex values here.
byte MAC_ADDRESS[] = {  0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x50 };
 
// IP address of MQTT server
byte MQTT_SERVER[] = { 192, 168, 1, 136 }; //192.168.1.136
 
EthernetClient ethClient;
PubSubClient client(MQTT_SERVER, 1883, callback, ethClient);
 
void setup()
{  
  pinMode(Door_Led_Pin, OUTPUT);    // declare Door_Led_Pin as output
  pinMode(Door_Sensor_Pin, INPUT);  // declare Door_Sensor_Pin as input
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
  
  
  val = digitalRead(Door_Sensor_Pin1);  // read Door_Sensor_Pin
  if (val == HIGH) {
    door1C = "Open";
  } else {
    door1C = "Closed";
  }
  
  val = digitalRead(Door_Sensor_Pin2);  // read Door_Sensor_Pin
  if (val == HIGH) {
    door2C = "Open";
  } else {
    door2C = "Closed";
  }
  
  val = digitalRead(Door_Sensor_Pin3);  // read Door_Sensor_Pin
  if (val == HIGH) {
    door3C = "Open";
  } else {
    door3C = "Closed";
  }
  
  val = digitalRead(Door_Sensor_Pin4);  // read Door_Sensor_Pin
  if (val == HIGH) {
    door4C = "Open";
  } else {
    door4C = "Closed";
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
    client.publish("arduino02/door1",door1C);
    client.publish("arduino02/door2",door2C);
    client.publish("arduino02/door3",door3C);
    client.publish("arduino02/door4",door4C);
  }
  else if (val != oldvalue) {
    client.publish("arduino02/door1",door1C);
    client.publish("arduino02/door2",door2C);
    client.publish("arduino02/door3",door3C);
    client.publish("arduino02/door4",door4C);

    oldvalue = val;
  }
 
    
  // MQTT client loop processing
  client.loop();
}
 
// Handles messages arrived on subscribed topic(s)
void callback(char* topic, byte* payload, unsigned int length) {
}
