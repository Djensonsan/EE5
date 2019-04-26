#include <Kalman.h>
#include <SPI.h>
#include "df_can.h"
#include<Wire.h>
#include "MPU6050.h"

#define MODE 0 //mode=0 kalman filter, MODE=1 complimentary filter
#define DEGREE_DIFFERENCE 5

#define BUFFER_LENGTH 128

/************Kalman filter*************/
/*variables used for display*/
float prevAngleX_Kal;
/*variables for the final angle*/
float kalAngleX; // Calculated angle using a Kalman filter
Kalman kalmanX; // Create a Kalman instance

/***********Complementary filter*************/
/*variables used for display*/
float prevAngleX_Com;
/*variables for the final angle*/
float compAngleX; // Calculated angle using a complementary filter
/*variables used inside the complementary filter*/
float prev_gyrox;

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
  mpuSetup();             //initialize MPU6050

  canSetup();             //initialize CAN bus shield

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
  Serial.begin(9600);
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
    prevAngleX_Kal = 0;
  }
  else                             //default complementary
  {
    prevAngleX_Com = 0;
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
    initialAngleSum = initialAngleSum + accX;
  }
  accX = initialAngleSum / 100;

  if (MODE == 0)                                       //kalman
  {
    kalmanX.setAngle(accX);                            //Set starting angle for Kalman filter
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
  long int sum_accx = 0;
  long int sum_grx = 0;
  for (int i = 0; i < 4; i++)
  {
    MPU.getMotion6(&arx, &ary, &arz, &grx, &gry, &grz);   //get the raw values from the accelerometer and gyroscope
    sum_accx = sum_accx + arx;
    sum_grx = sum_grx + grx;
  }
  arx = sum_accx / 4;
  grx = sum_grx / 4;
}

/*******************************Initialize the complementary filter**********************************/
void initializeComplementary()
{
  compAngleX = accX;
  prev_gyrox = grx;
}

/********************************Map Raw values****************************************************/
void mapRawValues()
{
  accX = RAD_TO_DEG * (atan2(-ary, -arz) + PI);    //accelerometer euler angles
  if (accX <= 180)
  {
    accX = accX + 180;
  }
  else
  {
    accX = accX - 180;
  }
  grx = grx / gyroScale;                          //in degree/sec

}

/**********************************fill in the buffer***********************************************/
void updateBuffer(int i)
{
  float tempX;
  
  readAvg();
  mapRawValues();

  //calculate dt
  double dt = (double)(millis() - t) / 1000;                                //time to integrate gyroscope output
  t = millis();

  if (MODE == 0) //kalman
  {
    kalAngleX = kalmanX.getAngle(accX, grx, dt);
    tempX = kalAngleX;
  }

  else
  {
    compAngleX = complementaryFilter(compAngleX, accX, grx, prev_gyrox, dt);  // Calculate the angle using a complementary filter
    tempX = compAngleX;
  }

  //circular buffer & average
  sum = sum - average_buffer[i];
  average_buffer[i] = tempX;

  sum = sum + average_buffer[i];
}

/************************************************Complementary filter*************************************************/
float complementaryFilter(float lastAngle, float accX, float grx, float prev_grx, double dt)
{

  float alpha = 0.96;                                                                    //filter coefficient
  float x = (alpha * (lastAngle + dt * 0.5 * (grx + prev_grx)) + ((1 - alpha) * accX));  //trapezoidal ruel
  return x;
}

/******************************Only print out when there is difference*******************************************/
void showDifference(float value, float difference) {
  if (MODE == 0) //kalman
  {
    if (abs(value - prevAngleX_Kal ) > difference  ) //send x-axis data over CAN bus
    {
      prevAngleX_Kal = value;
      Serial.print("Circular buffer Kalman X to the ground ");
      Serial.println(180 - value);
      sendThroughCanBus(180 - value);
    }
  }
  else
  {
    if (abs(value - prevAngleX_Com ) > difference  ) //send x-axis data over CAN bus
    {
      prevAngleX_Com = value;
      Serial.print("Circular buffer Complementary X to the ground ");
      Serial.println(180 - value);
      sendThroughCanBus(180 - value);
    }
  }
}

void sendThroughCanBus(float data)
{
  sensor_data_t sensor_data_x;
  sensor_data_x.value = data;
  byte data_x[4] = {sensor_data_x.bytes[0], sensor_data_x.bytes[1], sensor_data_x.bytes[2], sensor_data_x.bytes[3]};
  CAN.sendMsgBuf(0x01, 0, 4, data_x);
}

