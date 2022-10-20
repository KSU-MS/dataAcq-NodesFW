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
#include <Adafruit_LIS3DH.h>
#include <SimpleKalmanFilter.h>
#define GYRO_Q 0.01
SimpleKalmanFilter accelXKalman(1,1,0.01); //accelerometer X
SimpleKalmanFilter accelYKalman(1,1,0.01); //accelerometer Y
SimpleKalmanFilter accelZKalman(1,1,0.01); //accelerometer Z
//
SimpleKalmanFilter gyrorollKalman(1,1,GYRO_Q); //gyro roll
SimpleKalmanFilter gyropitchKalman(1,1,GYRO_Q); //gyro pitch
SimpleKalmanFilter gyroheadingKalman(1,1,GYRO_Q); //gyro heading
//
SimpleKalmanFilter brakepress1kalman(1,1,0.01); //gyro roll
SimpleKalmanFilter brakepress2kalman(1,1,0.01); //gyro pitch
SimpleKalmanFilter steeringkalman(1,1,0.01); //gyro heading

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
//
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
//
sensors_event_t lis3event;
int16_t accelX=0;
int16_t accelY=0;
int16_t accelZ=0;
//
uint16_t brakePress1=0;
uint16_t brakePress2=0;
uint16_t steering=0;
Metro getSensorData = Metro(10);
// Can chip
#define CAN_CS 10
#define CAN_IRQ 14
uint32_t packet_timestamp;
uint16_t packet_id=ID_BHNODE_MSG;
uint16_t packet1_id=packet_id+1;
CanNetwork can = CanNetwork(CAN_CS);
CanPacket packet = {timestamp : packet_timestamp, id : packet_id, data : {0xFF}, delim : CAN_PACKET_DELIM};
CanPacket packet1 = {timestamp : packet_timestamp, id : packet1_id, data : {0xFF}, delim : CAN_PACKET_DELIM};
Metro canSend = Metro(100);
//
#define debug_mode false
Metro debugtim = Metro(1000);
unsigned long getSensorDataEndTime;
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
Serial.println("LIS3DH test!");

  if (! lis.begin(0x19)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  //lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  }
}

void setup()
{
  delay(5000);
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
void loop()
{
  /* Calculate pitch and roll from the raw accelerometer data */
  if(getSensorData.check()){
    unsigned long getSensorDataStartTime=millis(); //time how long getting sensor info takes
    //get 10 dof shit here
    accel.getEvent(&accel_event);
    dof.accelGetOrientation(&accel_event, &orientation);
    mag.getEvent(&mag_event);
    dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
    lis.read();
    lis.getEvent(&lis3event);
        accelX=lis3event.acceleration.x*100;
      int16_t estimated_accelx=accelXKalman.updateEstimate(accelX);
        accelY=lis3event.acceleration.y*100;
      int16_t estimated_accely=accelYKalman.updateEstimate(accelY);
        accelZ=lis3event.acceleration.z*100;
      int16_t estimated_accelz=accelZKalman.updateEstimate(accelZ);
    lis.readADC(1);
    lis.readADC(2);
    lis.readADC(3);
    scaledHeading=(orientation.heading*10);
      int16_t estimated_heading = gyroheadingKalman.updateEstimate(scaledHeading);
    scaledPitch=(orientation.pitch*10);
      int16_t estimated_pitch = gyropitchKalman.updateEstimate(scaledPitch);
    scaledRoll=(orientation.roll*10);
      int16_t estimated_roll = gyrorollKalman.updateEstimate(scaledRoll);


    getSensorDataEndTime = millis() - getSensorDataStartTime; //see how long it took
    //Serial.printf("%d,%d,%d,%d,%d,%d\n",accelX,estimated_accelx,accelY,estimated_accely,accelZ,estimated_accelz);
    Serial.printf("%d,%d,%d,%d,%d,%d\n",scaledHeading,scaledPitch,scaledRoll,estimated_heading,estimated_pitch,estimated_roll);\
    analogWrite(3,map(estimated_heading,0,3600,0,255));
    analogWrite(4,map(estimated_pitch,-1800,1800,0,255));
    analogWrite(6,map(estimated_roll,-1800,1800,0,255));
  }
  //pin 3 = heading, 4 = pitch, 6=roll

  

  // if(debugtim.check()){
  //     Serial.print(F("Roll: "));
  //     Serial.println(orientation.roll,10);
  //     Serial.print(F("Heading: "));
  //     Serial.println(orientation.heading,10);
  //     Serial.print(F("Pitch: "));
  //     Serial.println(orientation.pitch,10);
  //     Serial.print(F("ScaledRoll: "));
  //     Serial.println(scaledRoll,10);
  //     Serial.print(F("ScaledHeading: "));
  //     Serial.println(scaledHeading,10);
  //     Serial.print(F("ScaledPitch: "));
  //     Serial.println(scaledPitch,10);
  //     Serial.print(F("Sensor data get time: "));
  //     Serial.println(getSensorDataEndTime);
  //       /* Display the results (acceleration is measured in m/s^2) */
  //     Serial.print("\tX: "); Serial.print(lis3event.acceleration.x);
  //     Serial.print(" \tY: "); Serial.print(lis3event.acceleration.y);
  //     Serial.print(" \tZ: "); Serial.print(lis3event.acceleration.z);
  //     Serial.println(" m/s^2 ");
  //     Serial.print("\tX:  "); Serial.print(lis.x);
  //     Serial.print("  \tY:  "); Serial.print(lis.y);
  //     Serial.print("  \tZ:  "); Serial.println(lis.z);
  // }
    
  if(canSend.check()){
    memcpy(&packet.data[0],&scaledRoll,sizeof(scaledRoll));
    memcpy(&packet.data[4],&scaledHeading,sizeof(scaledHeading));
    memcpy(&packet1.data[0],&scaledPitch,sizeof(scaledPitch));
    can.send(&packet);
    can.send(&packet1);
  }

}