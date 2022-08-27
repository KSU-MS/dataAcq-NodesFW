#include <CanNetwork.h>
#include <Metro.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MLX90640.h>
#include <FreqMeasureMulti.h>
#include <Arduino.h>
#include <math.h>
#include <SimpleKalmanFilter.h>
#define RPM_TIMEOUT 100
#define BOARD_ID 0x4CF
#define SEND_DELAY 100
Metro canSend = Metro(SEND_DELAY); //set freq here
Metro tireTemptimer = Metro(1000);
// Can chip defines
#define CAN_CS 10
#define CAN_IRQ 14 //interrupt not used here, nor on board
CanNetwork can = CanNetwork(CAN_CS);
#define debug_mode true
//Peripheral defines
Adafruit_MLX90640 mlx;
Adafruit_ADS1115 ads;
int16_t adc0, adc3;
float volts0, volts3;
FreqMeasureMulti freq1;
uint16_t current_rpm = 0;
float prev_rpm;
unsigned long current_rpm_change_time;
int sum = 0;
int count = 0;
unsigned long time1;
float frame[32*24]; // buffer for full frame of temperatures

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01); //filter def
// uncomment *one* of the below
#define PRINT_TEMPERATURES
//#define PRINT_ASCIIART
//TT Lists are based off the excel plot of all the array points 
//THese are to make it easy to average the points in an area of the tire
int tt1list[42]={ 200,201,202,203,204,205,206,
                  232,233,234,235,236,237,238,
                  264,265,266,267,268,269,270,
                  296,297,298,299,300,301,302,
                  328,329,330,331,332,333,334,
                  360,361,362,363,364,365,366};

int tt2list[42]={ 392,393,394,395,396,397,398,
                  424,425,426,427,428,429,430,
                  456,457,458,459,460,461,462,
                  488,489,490,491,492,493,494,
                  520,521,522,523,524,525,526,
                  552,553,554,555,556,557,558};

int tt3list[42]={ 207,208,209,210,211,212,213,
                  239,240,241,242,243,244,245,
                  271,272,273,274,275,276,277,
                  303,304,305,306,307,308,309,
                  335,336,337,338,339,340,341,
                  367,368,369,370,371,372,373};                  

void setup()
{
//  pinMode(LED_BUILTIN,OUTPUT);
  delay(10);
  freq1.begin(23);
  ads.setGain(GAIN_TWOTHIRDS);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    //while (1);
  }
  if (debug_mode)
  {
    Serial.begin(115200);
    // Wait for slow serial on the teensy, but only for 5 seconds max so we never get stuck here
    unsigned long timeout = millis();
    // while (!Serial && millis() - timeout < 5000){
    //   delay(1);
    // }
    Serial.println("Setup begin...");
    Serial.println("Debug mode on.");
    can.debug();
  }

  // Init CAN system
  can.init(CAN_250KBPS); //NOTE: can will init at half the speed defined
  // because lib is for 16mhz crystal and can breakout boards come in w/8mhz
   Serial.println("Adafruit MLX90640 Simple Test");
  if (! mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 not found!");
    //while (1) delay(10);
  }
  Serial.println("Found Adafruit MLX90640");
  Serial.print("Serial number: ");
  Serial.print(mlx.serialNumber[0], HEX);
  Serial.print(mlx.serialNumber[1], HEX);
  Serial.println(mlx.serialNumber[2], HEX);
  
  //mlx.setMode(MLX90640_INTERLEAVED);
  mlx.setMode(MLX90640_CHESS);
  Serial.print("Current mode: ");
  if (mlx.getMode() == MLX90640_CHESS) {
    Serial.println("Chess");
  } else {
    Serial.println("Interleave");    
  }

  mlx.setResolution(MLX90640_ADC_16BIT);
  Serial.print("Current resolution: ");
  mlx90640_resolution_t res = mlx.getResolution();
  switch (res) {
    case MLX90640_ADC_16BIT: Serial.println("16 bit"); break;
    case MLX90640_ADC_17BIT: Serial.println("17 bit"); break;
    case MLX90640_ADC_18BIT: Serial.println("18 bit"); break;
    case MLX90640_ADC_19BIT: Serial.println("19 bit"); break;
  }

  mlx.setRefreshRate(MLX90640_2_HZ);
  Serial.print("Current frame rate: ");
  mlx90640_refreshrate_t rate = mlx.getRefreshRate();
  switch (rate) {
    case MLX90640_0_5_HZ: Serial.println("0.5 Hz"); break;
    case MLX90640_1_HZ: Serial.println("1 Hz"); break; 
    case MLX90640_2_HZ: Serial.println("2 Hz"); break;
    case MLX90640_4_HZ: Serial.println("4 Hz"); break;
    case MLX90640_8_HZ: Serial.println("8 Hz"); break;
    case MLX90640_16_HZ: Serial.println("16 Hz"); break;
    case MLX90640_32_HZ: Serial.println("32 Hz"); break;
    case MLX90640_64_HZ: Serial.println("64 Hz"); break;
  }
}
void loop()
{
  // digitalWrite(LED_BUILTIN,LOW);
  //adc0 = ads.readADC_SingleEnded(0); //grab raw adc reading as 16 bit uint
  adc0 = analogRead(A0); //trying to get an ads1113 reading will lock up teensy if it fails
  uint16_t estimated_adc0 = simpleKalmanFilter.updateEstimate(adc0);
  unsigned long test = millis();
  if((test - current_rpm_change_time) > RPM_TIMEOUT) {
    // rpm.data = 0;
    current_rpm = 0;

  }
  if (freq1.available()) {
    // average several reading together
    sum = sum + freq1.read();
    count = count + 1;
    current_rpm_change_time = millis();
    if (count > 1) {
        float testRpm = freq1.countToFrequency(sum / count)*60 /18; //18=number of teeth
        current_rpm = (int16_t)testRpm;

        /*if ( testRpm - prev_rpm < 1)
        {
          current_rpm = (double)testRpm;
          prev_rpm = testRpm;
        }*/
        // Serial.printf("Current RPM: %f \n", testRpm);
      sum = 0;
      count = 0;
      //prev_rpm = testRpm;
    }
  }
  //acquire le sauce
  if(tireTemptimer.check()){
   if (mlx.getFrame(frame)) { //get IR sensor array
    Serial.println("Failed");
    return; //TODO find out how long this shit takes because it is prob blocking
  }
  }
  int tiretemp1=0,tiretemp2=0,tiretemp3=0;
  //get first tiretemp
  for(int i=0;i<42;i++){
    uint8_t lol = (int)frame[tt1list[i]];
    tiretemp1+=lol;
  }
  uint8_t tiretemp1avg=tiretemp1/42;

  //get second TT
  for(int i=0;i<42;i++){
    uint8_t lol = (int)frame[tt2list[i]];
    tiretemp2+=lol;
  }
  uint8_t tiretemp2avg=tiretemp2/42;

  //get third TT
  for(int i=0;i<42;i++){
    uint8_t lol = (int)frame[tt3list[i]];
    tiretemp3+=lol;
  }
  uint8_t tiretemp3avg=tiretemp3/42;

  // // This is how you create a packet                         //0           1             2             3   4    5    6   7

  if(canSend.check()){
    // digitalWrite(LED_BUILTIN,HIGH);
    CanPacket packet = {timestamp : 0, id : 0x68, data : {tiretemp1avg,tiretemp2avg, tiretemp3avg, 0x4, 0x5, 0x6, 0x7, 0x8}, delim : 0};
    memcpy(&packet.data[3], &current_rpm, sizeof(current_rpm)); //byte 3,4 for wheel speed
    memcpy(&packet.data[5], &estimated_adc0, sizeof(estimated_adc0)); //byte 5,6 for shock pot
    can.send(&packet);
  }
}