#include <Kalman.h>
#include <SPI.h>
#include "df_can.h"
#include<Wire.h>
#include "MPU6050.h"

#define MODE 0 //mode=0 kalman filter, MODE=1 complimentary filter
#define DEGREE_DIFFERENCE 5
#define CAN_ID 0x03
#define OFFSET 12

#define BUFFER_LENGTH 128

/************Kalman filter*************/
/*variables used for display*/
float prevAngleZ_Kal;
/*variables for the final angle*/
float kalAngleZ; // Calculated angle using a Kalman filter
Kalman kalmanZ; // Create a Kalman instance

/***********Complementary filter*************/
/*variables used for display*/
float prevAngleZ_Com;
/*variables for the final angle*/
float compAngleZ; // Calculated angle using a complementary filter
/*variables used inside the complementary filter*/
float prev_gyroz;

/****************MPU6050*********************/
MPU6050 MPU;
const int MPU_addr = 0x68;

/*variables used for raw values */
int16_t arx, ary, arz, grx, gry, grz;
int gyroScale = 131; //gyro scale factor from datasheet

/*variables used for angles read from accelerometers*/
float accX, accY, accZ;

/*variables for circular buffer*/
float average_buffer[BUFFER_LENGTH] = {0};
float  sum = 0;
float average = 0;

/*timer and counter*/
long int t;
int buffer_counter = 0;

/*CAN Bus*/
MCPCAN CAN(10);      // Set CS pin of CAN shield
union sensor_data_t //struct to split data into bytes to be sent over CAN  bus
{
  float value;
  byte bytes[4];
};

void setup()
{
  Serial.begin(9600); //setup serial
  while (!Serial) { // wait for serial port to connect. Needed for Leonardo only
  }

  canSetup();             //initialize CAN bus shield
  
  mpuSetup();             //initialize MPU6050

  initializeVariables();  //initialize all global variables

  initializeAngle();      //initialize the angle

  initializeBuffer();     //initialize the circular buffer
  
  t = millis();
}

void loop()
{
  updateBuffer(buffer_counter);

  buffer_counter++;

  average = sum / BUFFER_LENGTH;

  Serial.println(average);
  
  if (buffer_counter >= BUFFER_LENGTH)
  {
    buffer_counter = 0;
  }

  //display the result when there is a difference bigger than 5 and send through the CAN bus
  showDifference(average, DEGREE_DIFFERENCE);
}

/*********************************MPU setup****************************************************/
void mpuSetup()
{
  
  Wire.setClock(400000);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); //address of IMU
  Wire.write(0);
  Wire.endTransmission(true);
}

/*****************************CAN setup***********************************************************/
void canSetup()
{
  int count = 50;                                            // the max numbers of initializint the CAN-BUS, if initialize failed first!.
  Serial.print("Initializing can controlor...");
  do {
    CAN.init();   //must initialize the Can interface here!
    if (CAN_OK == CAN.begin(CAN_500KBPS))                    // init can bus : baudrate = 500k
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
}

/********************************************initialize variable***********************************************/
void initializeVariables()
{
  if (MODE == 0)                   //kalman
  {
    prevAngleZ_Kal = 0;
  }
  else                             //default complementary
  {
    prevAngleZ_Com = 0;
  }
}

/******************************************initialize buffer****************************************************/
void initializeAngle()
{
  //read 100 data and calculate the average value of accX to get an accurate result
  float initialAngleSum = 0;
  for (int i = 0; i < 100; i++)
  {
    MPU.getMotion6(&arx, &ary, &arz, &grx, &gry, &grz); //get the raw values from the accelerometer and gyroscope
    mapRawValues();
    initialAngleSum = initialAngleSum + accZ;
  }
  accZ = initialAngleSum / 100;

  if (MODE == 0)                                       //kalman
  {
    kalmanZ.setAngle(accZ);                            //Set starting angle for Kalman filter
  }
  else                                                 //default complementary
  {
    initializeComplementary();                         //initialize the complementary filter
  }

}

/***********************************Initialize the circular buffer******************************************/
void initializeBuffer()
{
  for (int i = 0; i < BUFFER_LENGTH; i++)
  {
    updateBuffer(i);
  }
}

/********************************************Read avg*******************************************************/
void readAvg()
{
  long int sum_accz = 0;
  long int sum_grz = 0;
  for (int i = 0; i < 4; i++)
  {
    MPU.getMotion6(&arx, &ary, &arz, &grx, &gry, &grz);   //get the raw values from the accelerometer and gyroscope
    sum_accz = sum_accz + arz;
    sum_grz = sum_grz + grz;
  }
  arz = sum_accz / 4;
  grz = sum_grz / 4;
}

/*******************************Initialize the complementary filter**********************************/
void initializeComplementary()
{
  compAngleZ = accZ;
  prev_gyroz = grz;
}

/********************************Map Raw values****************************************************/
void mapRawValues()
{
  accZ = RAD_TO_DEG * (atan2(-ary, -arx) + PI);    //accelerometer euler angles
  accZ = accZ - 90;
  grz = grz / gyroScale;                          //in degree/sec

}

/**********************************fill in the buffer***********************************************/
void updateBuffer(int i)
{
  float tempZ;
  
  readAvg();
  mapRawValues();

  //calculate dt
  double dt = (double)(millis() - t) / 1000;                                //time to integrate gyroscope output
  Serial.println(dt);
  
  t = millis();

  if (MODE == 0) //kalman
  {
    kalAngleZ = kalmanZ.getAngle(accZ, grz, dt);
    tempZ = kalAngleZ;
  }

  else
  {
    compAngleZ = complementaryFilter(compAngleZ, accZ, grz, prev_gyroz, dt);  // Calculate the angle using a complementary filter
    tempZ = compAngleZ;
  }

  //circular buffer & average
  sum = sum - average_buffer[i];
  average_buffer[i] = tempZ;

  sum = sum + average_buffer[i];
}

/************************************************Complementary filter*************************************************/
float complementaryFilter(float lastAngle, float accZ, float grz, float prev_grz, double dt)
{

  float alpha = 0.96;                                                                    //filter coefficient
  float z = (alpha * (lastAngle + dt * 0.5 * (grz + prev_grz)) + ((1 - alpha) * accZ));  //trapezoidal ruel
  return z;
}

/******************************Only print out when there is difference*******************************************/
void showDifference(float value, float difference) {
  if (MODE == 0) //kalman
  {
    if (abs(value - prevAngleZ_Kal ) > difference  ) //send x-axis data over CAN bus
    {
      prevAngleZ_Kal = value;
      Serial.print("Circular buffer Kalman Z to the ground ");
      Serial.println(value+OFFSET);
      sendThroughCanBus(value+OFFSET);
    }
  }
  else
  {
    if (abs(value - prevAngleZ_Com ) > difference  ) //send x-axis data over CAN bus
    {
      prevAngleZ_Com = value;
      Serial.print("Circular buffer Complementary Z to the ground ");
      Serial.println(value+OFFSET);
      sendThroughCanBus(value+OFFSET);
    }
  }
}

void sendThroughCanBus(float data)
{
  sensor_data_t sensor_data_z;
  sensor_data_z.value = data;
  byte data_z[4] = {sensor_data_z.bytes[0], sensor_data_z.bytes[1], sensor_data_z.bytes[2], sensor_data_z.bytes[3]};
  CAN.sendMsgBuf(CAN_ID, 0, 4, data_z);
}

