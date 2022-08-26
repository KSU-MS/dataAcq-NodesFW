#include <CanNetwork.h>
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Metro.h>
#include <KS2eCAN.hpp>

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
//
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;
//
  int32_t scaledRoll = 0;
  int32_t scaledPitch = 0;
  int32_t scaledHeading = 0; 
Metro getSensorData = Metro(10);
// Can chip
#define CAN_CS 10
#define CAN_IRQ 14
float packet_timestamp;
uint16_t packet_id=ID_BHNODE_MSG;
uint16_t packet1_id=packet_id+1;
byte blank_packet[8]={0xFF};
CanNetwork can = CanNetwork(CAN_CS);
CanPacket packet = {timestamp : packet_timestamp, id : packet_id, data : {0xFF}, delim : CAN_PACKET_DELIM};
CanPacket packet1 = {timestamp : packet_timestamp, id : packet1_id, data : {0xFF}, delim : CAN_PACKET_DELIM};
Metro canSend = Metro(100);
#define debug_mode false
Metro debugtim = Metro(1000);
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}

void setup()
{

  if (debug_mode)
  {
    Serial.begin(115200);
    // Wait for slow serial on the teensy, but only for 5 seconds max so we never get stuck here
    unsigned long timeout = millis();
    while (!Serial && millis() - timeout < 5000)
      delay(1);

    Serial.println("Setup begin...");
    Serial.println("Debug mode on.");
    can.debug();
  }
  Serial.println(F("Adafruit 10 DOF Pitch/Roll/Heading Example")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();
  // Init CAN system
  can.init(CAN_250KBPS);

}
unsigned long last = 0;
unsigned long lastP = 0;
void loop()
{

  // CanPacket packet = {timestamp : millis(), id : 100, data : {0x1,0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8}, delim : CAN_PACKET_DELIM};

  /* Calculate pitch and roll from the raw accelerometer data */
  if(getSensorData.check()){
    accel.getEvent(&accel_event);
    dof.accelGetOrientation(&accel_event, &orientation);
    mag.getEvent(&mag_event);
    dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
    scaledHeading=(orientation.heading*100);
    scaledPitch=(orientation.pitch*100);
    scaledRoll=(orientation.roll*100);  }
  if(debugtim.check()){
      Serial.print(F("Roll: "));
      Serial.println(orientation.roll,10);
      Serial.print(F("Heading: "));
      Serial.println(orientation.heading,10);
      Serial.print(F("Pitch: "));
      Serial.println(orientation.pitch,10);
      Serial.print(F("ScaledRoll: "));
      Serial.println(scaledRoll,10);
      Serial.print(F("ScaledHeading: "));
      Serial.println(scaledHeading,10);
      Serial.print(F("ScaledPitch: "));
      Serial.println(scaledPitch,10);
  }
    
  if(canSend.check()){
    memcpy(&packet.data[0],&scaledRoll,sizeof(scaledRoll));
    memcpy(&packet.data[4],&scaledHeading,sizeof(scaledHeading));
    memcpy(&packet1.data[0],&scaledPitch,sizeof(scaledPitch));
    can.send(&packet);
    can.send(&packet1);
  }

}