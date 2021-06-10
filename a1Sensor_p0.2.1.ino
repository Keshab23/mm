/**************************************************************
 *
 * @SYNOPSIS:
 *    This is the application file for a1Sensor with GPRS mode.
 * 
 * @VERSION: 0.2-Prod
 *            0.2.1: With Device Attributes
  */

 /**************************************************************/
const String currentFWVersion= "0.2.1";
//Gets boot up time
uint32_t bootupTime = millis();

 /********************* Project setup macros *****************************************/
 //#define Debug_Mode
 #define board_ESP8266
 #define network_GPRS_M66
 #define mm_broker

/************** Include section starts here **************************/
#include <mm_lib.h>

#include <LTR303.h>
#include <Wire.h>
#include <SparkFun_SHTC3.h>
#include <MQ135.h>

/******************** Global vars here ******************************************/
gprsAttr_t attr;
boolean attrFlag = true; //Flag to publish the attributes once after reset
int dhtOnReboot = 1;
uint32_t lastReconnectAttempt = 0;
float lastH,lastT,h,t=0;

//For MQ Sensor
const int aPin = A0;
//Define selection lines
#define SEL1  0//D3
#define SEL2  15//D8

SHTC3 mySHTC3;

// Create an LTR303 object, here called "light":
LTR303 light;

unsigned char gain;     // Gain setting, values = 0-7 
unsigned char integrationTime;  // Integration ("shutter") time in milliseconds
unsigned char measurementRate;  // Interval between DATA_REGISTERS update


/* Function to convert LTR303-ALS-01 CH data to LUX */
double luxVal(double d0, double d1){
  Dbg_println("Calculating LUX value..");
  double ratio,lux;
  // We will need the ratio for subsequent calculations
  ratio = d1 / (d0+d1);
  Dbg_println(ratio);
  
  int ALS_GAIN=1;
  double ALS_INT=1;

  // Determine lux per datasheet equations:
//  if(d0<40){
//    lux=d0;
//  }
  if (ratio < 0.45) {
    lux = ((1.7743 * d0) + (1.1059 * d1))/ALS_GAIN/ALS_INT/1.33;
    if(lux>100){
      lux = lux-50;
    }
    Dbg_println(lux);
  }

  else if ((ratio < 0.64) && (ratio >= 0.45)){
    lux = ((4.2785 * d0) - (1.9548 * d1))/ALS_GAIN/ALS_INT/1.33;
    if(lux>20){
      lux = lux-10;
    }
    Dbg_println(lux);
  }

  else if ((ratio < 0.85) && (ratio >= 0.64)){
    lux = ((0.5926 * d0) + (0.1185 * d1))/ALS_GAIN/ALS_INT;
    Dbg_println(lux);
  }

  // if (ratio >= 0.85)
  else {  
    lux = 0.0;
    Dbg_println(lux);
  }
  Dbg_println(lux);
  return lux;
}

/******************** setup() starts here ******************************************/

void setup() {
  
  // Set console baud rate
  Serial.begin(115200);
  while(!Serial);
  //Status LED Setup
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed,HIGH);
  //Network Setup
  networkInit();
  //wifiInit();
  mqttInit();
  //Checks FW upgrade
  if(connectToWifi()) {
    //publish_mqtt_log("WiFi:Connected, Checking FW Upgrade");
    checkForFwUpdate();
  }
  //setConfigVars();
  /**************** Sensors Setup *************************/  
  //For MQ type sensor
  pinMode(aPin, INPUT);

  //For I2C Sensors
  light.begin(); //I2C address (0x29)
  errorDecoder(mySHTC3.begin()); //I2C address (0x70)
  delay(5000);

  //TODO: I2C part no. read for light sensor
  Dbg_println(F("Setting sensor parameters..."));
  publish_mqtt_log("Setting sensor parameters...");
  
  gain = 0;
  light.setControl(gain, false, false);
  unsigned char time = 0;
  light.setMeasurementRate(time,3);
  light.setPowerUp();

  //MUX Selection lines setup
  pinMode(SEL1,OUTPUT);
  pinMode(SEL2,OUTPUT);
  
}

void loop() {

  String jsonPayload,temperature,humidity,mqData,light_data,powr,batV;
  double lux;
  boolean wifiFlag = false;
  float t,rh,ppm,cppm,batVolt,batVoltinP=0;
  unsigned int data0, data1;
  //Running on mains: powerMode=1 and for battery powerMode=0; 
  boolean powerMode=false;

  struct dev_attr_t {
    String mac;
    String imei;
    String ccid;
    String imsi;
  }dev_attr;
  
  //Blinks once for each iteration
  showStatus(statusLed,1);
  //Resets board at 30 min interval
  resetBoard(1000*60*30L);

  //Select ch0: for mq135 sensor
  digitalWrite(SEL1,LOW); digitalWrite(SEL2,LOW); 
  delay(1000);
  MQ135 mq135 = MQ135(aPin);
  //ppm = mq135.getPPM();
  //cppm = mq135.getCorrectedPPM(t,rh);
  //  float r0 = mq135.getRZero();
  //  Dbg_print("R Zero: ");Dbg_println(r0);
  //  float rS = mq135.getRZero();
  //  Dbg_print("Rs: ");Dbg_println(rS);
  mqData = analogRead(aPin);

  //Select ch2: for PWR_SENS
  digitalWrite(SEL1,LOW); digitalWrite(SEL2,HIGH); 
  delay(1000);
  float pwrSens = analogRead(aPin);
  //Sets power mode here
  (pwrSens>800)?powerMode=true:powerMode=false;
  Dbg_print("Power Sense: ");Dbg_println(pwrSens);
  delay(1000);

  //Select ch1: for BAT_VSENSE
  digitalWrite(SEL1,HIGH); digitalWrite(SEL2,LOW); 
  delay(1000);
  int batVoltAdc = analogRead(aPin);
  batVolt = (3.3/1024)*batVoltAdc;
  Dbg_print("Bat Volt: ");Dbg_println(batVolt);
  delay(1000);

  /* Deep sleep implementation for critical battery (mains: powerMode=1, battery powerMode=0) */
  if((batVolt<2.2) &&(powerMode==0) ){
    //if(modem.poweroff()){
    if(modem.sleepEnable()){
      Dbg_println("Modem powered off");
      delay(500);
      ESP.deepSleep(0);
    }
  }

  /* Converting Battery Voltage to Percentage */
  if(batVolt>2.53){
    batVoltinP=100;
  }
  else if(batVolt>=2.2) {
    batVoltinP = (100*(batVolt-2.2))/0.33;
  }
  else {
    batVoltinP = 0;
  }

  //Connect to network

  //Checks for WIFI
  if(!connectToWifi()){
    //publish_mqtt_log("WiFi:Not Connected");
    
  }
  else {
    publish_mqtt_log("WiFi:Connected");
    wifiFlag = true;
  }

  if((!wifiFlag) || (wifiFlag && mqttLogging)){
    if(!connectToGprs()){
      publish_mqtt_log("GPRS:Not connected");
      return;
    }
    else {
      publish_mqtt_log("GPRS:Connected");
    }
    
  }
  
 
  //Sensor reading starts here
  
  if (!(light.getData(data0,data1))) {
    // getData() returned false because of an I2C error, inform the user.
    byte error = light.getError();
    printError(error);
    
  }
  Dbg_print(data0);Dbg_println(data1);
  lux = luxVal(data0,data1);

  //Read from SHT sensor
  SHTC3_Status_TypeDef result = mySHTC3.update();
  getSHTData(rh,t);

  //Validates sensor readings
  if (isnan(rh) || isnan(t) || isnan(lux)) {
    Dbg_println(F("Sensor reading validation failed!"));
    publish_mqtt_log("Sensor reading validation failed!");
    return;
  }

  temperature = String(t,2);
  humidity = String(rh,2);
  light_data = String(lux);
  //light_data = String(cppm);
  powr = String(powerMode);
  batV = String(batVoltinP,2);
  //batV += ',';
  //batV += String(batVolt);
  
  Dbg_print("Temperature: ");Dbg_println(t);
  Dbg_print("Relative Humidity: ");Dbg_println(rh);
  Dbg_print("Light Data(LUX): ");Dbg_println(lux);
  //Dbg_print("CO2 PPM: ");Dbg_println(ppm);
  //Dbg_print("CO2 Corrected PPM:");Dbg_println(cppm);
  //light_data = (String)cppm;

  publish_mqtt_log("Done: Sensor Reading");
  
  //delay(1000);

  //Select ch2: for BAT_VSENSE
//  digitalWrite(SEL1,HIGH); digitalWrite(SEL2,LOW); 
//  delay(1000);
//  ldrData = analogRead(aPin);
//  Serial.print("Light Data: ");Serial.println(ldrData);
//  delay(1000);

  //Read attributes
  attr = getGprsAttributes();

  //Prepares payload to publish
  String keys[] = {"rssi","t","h","aq","l","p","batt"};
  String vals[] = {attr.rssi,temperature,humidity,mqData,light_data,powr,batV};
  jsonPayload = convertToJson(keys,vals,7);

  Dbg_print("Sensor Data:");Dbg_println(jsonPayload);
  
  //Branch for action using WiFi
  if(wifiFlag){
    //Checks MQTT connectivity
    if (!mqtt.connected()) {
      Dbg_println(" MQTT: Not Connected ");
      // Reconnect every 10 seconds
      uint32_t t = millis();
      if (t - lastReconnectAttempt > 1000L) {
        lastReconnectAttempt = t;
        if (mqttConnect()) {
          lastReconnectAttempt = 0;
        }
      }
      delay(1000);
      return;
    }
    else {
      publish_mqtt_log("MQTT:Connected");
      //Send attributes at boot time
      if(attrFlag){
        //Get device details
        dev_attr.mac  = macAddr;
        dev_attr.imei = modem.getIMEI();
        dev_attr.ccid = modem.getSimCCID();
        dev_attr.imsi = modem.getIMSI();
        //Dbg_print("Device Attrs:");Dbg_println(dev_attr.ccid);
        //Sent device attributes
        String attr_keys[] = {"fw","mac","imei","ccid","imsi"};
        String attr_vals[] = {currentFWVersion,dev_attr.mac,dev_attr.imei,dev_attr.ccid,dev_attr.imsi};
        String pl = convertToJson(attr_keys,attr_vals,5);
        if(mqtt.publish(attrTopic, pl.c_str())){
          Dbg_println("Device attributes sent over WiFi");
          attrFlag = false;
        }
      }
      //Publish data using WiFi, when mqtt logging is not enabled
      if(!mqttLogging){
        if(mqtt.publish(dataTopic, jsonPayload.c_str())){
          //Dbg_print(F("Sensor Data: "));Dbg_println(jsonPayload);
          Dbg_println("Sensor data published over WiFi");
          publish_mqtt_log("Sensor data published over WiFi");
          //showStatus(statusLed,2);
        }
      }
    }
  }
  if((!wifiFlag) || (wifiFlag && mqttLogging)) {
    if(mqttPublishOverGprs(jsonPayload.c_str())){
      delay(1000);
      Dbg_println("Sensor data published over GPRS");
      publish_mqtt_log("Sensor data published over GPRS");
      //showStatus(statusLed,2);
    }
    else {
      Dbg_println("Failed to publish over GPRS");
      publish_mqtt_log("Failed to publish over GPRS");
      //modem.poweroff();
      return;
    }
  }
  showStatus(statusLed,2);
  //delay(1000*60*4L);
  //modem.poweroff();
  modem.gprsDisconnect();
  powerMode?delay(1000*60*4L):delay(1000*60*10L);
  
}


///////////////////////
// Utility Functions //
///////////////////////
void getSHTData(float &rh,float &t)
{
  if(mySHTC3.lastStatus == SHTC3_Status_Nominal)              // You can also assess the status of the last command by checking the ".lastStatus" member of the object
  { 
    rh = mySHTC3.toPercent();                        // "toPercent" returns the percent humidity as a floating point number
    t  = mySHTC3.toDegC();                           // "toDegF" and "toDegC" return the temperature as a flaoting point number in deg F and deg C respectively 
  }
  else
  {
    Dbg_print("Update failed, error: "); 
    errorDecoder(mySHTC3.lastStatus);
    Dbg_println();
  }
}

void errorDecoder(SHTC3_Status_TypeDef message)                             // The errorDecoder function prints "SHTC3_Status_TypeDef" resultsin a human-friendly way
{
  switch(message)
  {
    case SHTC3_Status_Nominal : Dbg_println("Nominal"); break;
    case SHTC3_Status_Error : Dbg_println("Error"); break;
    case SHTC3_Status_CRC_Fail : Dbg_println("CRC Fail"); break;
    default : Dbg_println("Unknown return code"); break;
  }
}

void printError(byte error) {
  // If there's an I2C error, this function will
  // print out an explanation.

  Dbg_print("I2C error: ");
  Dbg_print(error,DEC);
  Dbg_print(", ");
  
  switch(error) {
    case 0:
      Dbg_println("success");
      break;
    case 1:
      Dbg_println("data too long for transmit buffer");
      break;
    case 2:
      Dbg_println("received NACK on address (disconnected?)");
      break;
    case 3:
      Dbg_println("received NACK on data");
      break;
    case 4:
      Dbg_println("other error");
      break;
    default:
      Dbg_println("unknown error");
  }
}
