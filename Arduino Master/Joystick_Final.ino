#include <SPI.h>
#include <SD.h>
#include "voltage_config.h"

#define CS 10
#define SD_CS 8
#define z_pin A0 
#define y_pin A1
#define x_pin A2
#define button_up 6
#define button_down 7

unsigned int z_in, y_in, x_in;
unsigned int z_out, y_out, x_out;
bool up, down;

float min_voltage = MIN;
float max_voltage = MAX;
unsigned int min, max;

float min_button_voltage = MINBUTTON;
float max_button_voltage = MAXBUTTON;
float mid_button_voltage = (MAXBUTTON+MINBUTTON)/2;
unsigned int min_button, max_button, mid_button;

bool amplifier_on = AMPLIFY ; 



void setup() {
 Serial.begin(9600);
 SPI.begin(); // initialization of SPI port
 SPI.setBitOrder(MSBFIRST);
 SPI.setDataMode(SPI_MODE1); // configuration of SPI communication in mode 0
 SPI.setClockDivider(SPI_CLOCK_DIV4); 
 pinMode(CS, OUTPUT);
 
 digitalWrite(CS, LOW); // setup external reference
 SPI.transfer(0b00001000); 
 delay(1);
 SPI.transfer(0);
 delay(1);
 SPI.transfer(0);
 delay(1);
 SPI.transfer(0b00000000); 
 delay(1);
 digitalWrite(CS, HIGH); 
 delay(1);

 if(amplifier_on) 
 {
  min_voltage = 1;
  max_voltage = 3;
  min_button_voltage = 1;
  max_button_voltage = 3;
 }
 min = (min_voltage/5)*4096; //min 12 bit value
 max = (max_voltage/5)*4096; //max 12 bit value

 min_button = (min_button_voltage/5)*4096; //min 12 bit value
 max_button = (max_button_voltage/5)*4096; //max 12 bit value
 mid_button = (mid_button_voltage/5)*4096;


 pinMode(button_up, INPUT);
 pinMode(button_down, INPUT);
 
/*pinMode(SD_CS, OUTPUT);
 digitalWrite(SD_CS, 1);
 Serial.print("Initializing SD card...");*/

 // see if the card is present and can be initialized:
 /* if (!SD.begin(SD_CS)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");*/
}

void loop()
{ 
  z_in = analogRead(z_pin);
  if(z_in>495 && z_in<525) z_in = 510;
  z_out=map(z_in,0, 1023, min, max);
  SPI_send(z_out, 0);

  y_in = analogRead(y_pin);
  if(y_in>495 && y_in<525) y_in = 510;
  y_out=map(y_in, 0, 1023, min, max);
  SPI_send(y_out, 1);
  
  x_in = analogRead(x_pin);
  if(x_in>495 && x_in<525) x_in = 510;
  x_out=map(x_in, 0, 1023, min, max);
  SPI_send(x_out, 2);

  up = digitalRead(button_up);
  down = digitalRead(button_down);
 

  if(up & ! down) SPI_send(max_button, 3);
  else if(down & !up) SPI_send(min_button, 3);
  else SPI_send(mid_button, 3); 

// joystick_log("z-axis: ", String(z_in), " y-axis: ", String(y_in), " x-axis: ", String(x_in), " up: ", String(up), " down: ", String(down));
}
 
void SPI_send(unsigned int value, int channel)
{
  byte low = (value & 0x00FF);
  value  = value >> 8;
  byte high = (value & 0x000F);
  switch(channel)
  {
    case 0:
    high+=0b00010000;
    break;
    case 1:
    high+=0b00110000;
    break;
    case 2:
    high+=0b01010000;
    break;
    case 3:
    high+=0b01110000;
    break;
  }
  digitalWrite(CS, LOW); 
  delay(1);
  SPI.transfer(0b00000011);
  SPI.transfer(high);
  SPI.transfer(low);
  SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  delay(1);
}

void joystick_log(String label1, String value1,String label2,String value2, String label3, String value3, String label4, String value4, String label5, String value5)
{
  File dataFile = SD.open("joystick.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(label1); dataFile.print(value1);
    dataFile.print(label2); dataFile.print(value2);
    dataFile.print(label3); dataFile.print(value3);
    dataFile.print(label4); dataFile.print(value4);
    dataFile.print(label5); dataFile.print(value5);
    dataFile.println();
    dataFile.close();;
  }
  else {
    Serial.println("error opening joystick.txt"); 
  }
}
