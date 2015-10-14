/*******************Demo for MQ-2 Gas Sensor Module V1.0*****************************
Support:  Tiequan Shao: support[at]sandboxelectronics.com

Lisence: Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)

Note:    This piece of source code is supposed to be used as a demostration ONLY. More
         sophisticated calibration is required for industrial field application. 

                                                    Sandbox Electronics    2011-04-25
************************************************************************************/
 
#include <SPI.h>
#include <PubSubClient.h>
#include <Ethernet.h>

/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (0)     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation

/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)

/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms
String gasS;
int gasI;

//light settings
char* lightC;
int photocellPin = 2; // the cell and 10K pulldown are connected to a0
int photocellReading; // the analog reading from the sensor divider

// Network Settings
// MAC address of ethernet shield
// Look for it on a sticket at the bottom of the shield. 
// Old Arduino Ethernet Shields or clones may not have a dedicated MAC address. Set any hex values here.
byte MAC_ADDRESS[] = {  0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x54 };
char clientName[ ] = "sfo-arduino02";
char topic_heartbeat[ ] = "arduino02/heartbeat";
char topic_gas[ ] = "arduino02/gas";
char topic_co[ ] = "arduino02/co";
char topic_smoke[ ] = "arduino02/smoke";
char topic_light[ ] = "arduino02/light";
char topic_temp[ ] = "arduino02/temp";
char message_buffer[100]; //msg buffer for publish message
// IP address of MQTT server
byte MQTT_SERVER[] = { 192, 168, 1, 136 }; 
EthernetClient ethClient;
PubSubClient client(MQTT_SERVER, 1883, callback, ethClient);

//timing settings
unsigned long time; // variable to remember last publish

//temperature setting
int tempPin=3;
String tempString;
int tempReading;

void setup()
{
  Serial.begin(9600);                               //UART setup, baudrate = 9600bps
  Serial.print("Calibrating...\n");                
  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                    //when you perform the calibration                    
  Serial.print("Calibration is done...\n"); 
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
  //initEthernet();
}



void loop()
{
   
 // ConnectionInit();
//  MQTTLoop();   

  TestLoop();
}

void TestLoop()
{
  Serial.print("Photo:");
  Serial.print(PhotoSensorRead());
  Serial.print("\n");
  Serial.print("Gas:");
  Serial.print(GasSensorRead());
  Serial.print("\n");
  Serial.print("CO:");
  Serial.print(COSensorRead());
  Serial.print("\n");
  Serial.print("Smoke:");
  Serial.print(SmokeSensorRead());
  Serial.print("\n");
  Serial.print("Temp:");
  Serial.print(TempSensorRead());
  Serial.print("\n");
  delay(800);
}

void MQTTLoop()
{
   // Publish sensor reading every X milliseconds
  if (millis() > (time + 60000)) {
    time = millis();
    client.publish(topic_light,PhotoSensorRead());
    client.publish(topic_gas,GasSensorRead());
    client.publish(topic_co,COSensorRead());
    client.publish(topic_smoke,SmokeSensorRead());
    client.publish(topic_temp,TempSensorRead());
  }
   
  // MQTT client loop processing
  client.loop();
}

void initEthernet()
{
  if (Ethernet.begin(MAC_ADDRESS) == 0)
  {
    Serial.println("Failed to configure Ethernet using DHCP");
    return;
  }
}

void ConnectionInit()
{
  if (!client.connected())
   {
    //client.connect("clientID", "mqtt_username", "mqtt_password");
    client.connect(clientName);
    client.publish(topic_heartbeat, "I'm alive!");
   }
}

char* TempSensorRead()
{
  char charBuf[50];
  tempReading = analogRead(tempPin); 
  tempString = String(tempReading);
  tempString.toCharArray(charBuf,50);
  return charBuf;
}

char* COSensorRead()
{
   char charBuf[50];
   gasS = String(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO));
   gasS.toCharArray(charBuf,50);
   return charBuf;
}
char* SmokeSensorRead()
{
   char charBuf[50];
   gasS = String(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE));
   gasS.toCharArray(charBuf,50);
   return charBuf;
}
char* GasSensorRead()
{
   char charBuf[50];
   gasS = String(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG));
   gasS.toCharArray(charBuf,50);
   return charBuf;
}

char* PhotoSensorRead()
{
  photocellReading = analogRead(photocellPin); 
  if (photocellReading < 10) {
    lightC = "Dark";
  } else if (photocellReading < 200) {
    lightC = "Dim";
  } else if (photocellReading < 500) {
    lightC = "Light";
  } else if (photocellReading < 800) {
    lightC = "Bright";
  } else {
    lightC = "Very bright";
  }
  return lightC;
}

// Handles messages arrived on subscribed topic(s)
void callback(char* topic, byte* payload, unsigned int length) {
}

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value

  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 

  return val; 
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;  
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    

  return 0;
}

/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
