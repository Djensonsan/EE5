
#include <SPI.h>
#include "df_can.h"
#include <SD.h>
#include<Wire.h>
#include "MPU6050.h"

MPU6050 MPU;

const int MPU_addr = 0x68;

int16_t arx, ary, arz, grx,gry, grz;
float accX, accY, accZ, alpha;
float gyroX,gyroY, gyroZ;
float x, y, z;
//float prev_x,
float prev_z;

float delta_t, t, prev_t;

union sensor_data_t //struct to split data into bytes to be sent over CAN  bus
{
  float value;
  byte bytes[4];
};

//sensor_data_t sensor_data_x;
sensor_data_t sensor_data_z;

int i;
float gyroScale = 131; //gyro scale factor from datasheet

MCPCAN CAN(10);      // Set CS pin of CAN shield
char  sd_cspin = 4; //Set CS pin of SD card
File myFile;

void setup()
{
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); //address of IMU
  Wire.write(0);
  Wire.endTransmission(true);

  int count = 50;                                     // the max numbers of initializint the CAN-BUS, if initialize failed first!.
  Serial.print("Initializing can controlor...");
  do {
    CAN.init();   //must initialize the Can interface here!
    if (CAN_OK == CAN.begin(CAN_500KBPS))                  // init can bus : baudrate = 500k
    {
      Serial.println("DFROBOT's CAN BUS Shield init ok!");
      break;
    }
    else
    {
      Serial.println("DFROBOT's CAN BUS Shield init fail");
      Serial.println("Please Init CAN BUS Shield again");

      delay(100);
      if (count <= 1)
        Serial.println("Please give up trying!, trying is useless!");
    }

  } while (count--);

  /*Serial.print("Initializing SD card..."); //SD card code if local loging is implemented

 if (!SD.begin(sd_cspin)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization success!");
  myFile = SD.open("Node0x01.txt", FILE_WRITE);
  if (!myFile)
  {
    Serial.println("open Node0x01.text failed!");
  }
  else
  {
    Serial.println("open Node0x01.text success!");
  }*/

  //prev_x=0;
  prev_z=0;
  i=1;
}

void loop()
{
  prev_t = t;
  t = millis();
  delta_t = (t - prev_t) / 1000; //time to integrate gyroscope output

  MPU.getMotion6(&arx, &ary, &arz, &grx, &gry, &grz); //get the raw values from the accelerometer and gyroscope
  
  accX = RAD_TO_DEG * (atan2(-ary, -arz) + PI); //accelerometer euler angles
  accY = RAD_TO_DEG * (atan2(-arx, -arz) + PI); 
  accZ = RAD_TO_DEG * (atan2(-ary, -arx) + PI);

  grx = grx/gyroScale; 
  gry = gry/gyroScale;   
  grz = grz/gyroScale;
  
  if (i == 1) {
    gyroX = accX; //initialize gyroscope to the accelerometer value 
    gyroY = accY;
    gyroZ = accZ;
    i++;
  }
  else{
    gyroX = x + (delta_t * grx); //integrate gyroscope reading to get angular displacement
    gyroY = y + (delta_t * gry);
    gyroZ = z + (delta_t * grz);
  } 

  alpha = 1/(1+delta_t);

  x = accX;//(alpha * gyroX) + ((1-alpha)* accX);
  y = (alpha * gyroY) + ((1-alpha)* accY); //complementary filter to get more accurate values
  z = (alpha * gyroZ) + ((1-alpha)* accZ);

  /*if (abs(x - prev_x )> 5 && abs(x - prev_x )<355 ) //send x-axis data over CAN bus
  {
    prev_x = x;
    sensor_data_x.value = x;
    Serial.println("x-axis");
    Serial.print(sensor_data_x.value); 
    byte data_x[4] = {sensor_data_x.bytes[0], sensor_data_x.bytes[1], sensor_data_x.bytes[2], sensor_data_x.bytes[3]};
    CAN.sendMsgBuf(0x01, 0, 4, data_x);
  }*/
  
 if (abs(z - prev_z )> 5 && abs(z - prev_z )<355  ) //send z-axis data over CAN bus
  {
    prev_z = z;
    sensor_data_z.value = z;
    Serial.println("z_axis");
    Serial.println(sensor_data_z.value);
    byte data_z[4] = {sensor_data_z.bytes[0], sensor_data_z.bytes[1], sensor_data_z.bytes[2], sensor_data_z.bytes[3]};
    CAN.sendMsgBuf(0x03, 0, 4, data_z);
  }
}
