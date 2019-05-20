/* This sketch contains the code for the 'Slave' Arduino; implementing the GPS, DA and Joystick functionalities.
*/
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "Wire_Jens.h"
#include "voltage_config.h"

#define CS 10
#define z_pin A0
#define y_pin A1
#define x_pin A2
#define button_up 6
#define button_down 7

String joystick_data;
String GPS_data;
String data;

unsigned int z_in, y_in, x_in;
unsigned int z_out, y_out, x_out;
bool up, down;

/*
   Assign values from config file
*/
float min_voltage = MIN;
float max_voltage = MAX;
unsigned int min, max;
float min_button_voltage = MINBUTTON;
float max_button_voltage = MAXBUTTON;
float mid_button_voltage = (MAXBUTTON + MINBUTTON) / 2;
unsigned int min_button, max_button, mid_button;
bool amplifier_on = AMPLIFY ; //If voltage range 3-9 is used, this value is set to 1

static const int RXPin_GPS = 4, TXPin_GPS = 3;

TinyGPSPlus gps;
SoftwareSerial serial_GPS(RXPin_GPS, TXPin_GPS);

void setup()
{
  /* Serial Initialisation of GPS and Monitor.
  */
  Serial.begin(9600);
  serial_GPS.begin(9600);

  /* SPI Initialisation.
     Configuration of SPI communication in mode 0.
  */
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  pinMode(CS, OUTPUT);

  /* SPI Initialisation.
    Configuration of SPI communication in mode 0.
  */
  if (amplifier_on)
  {
    min_voltage = 1;
    max_voltage = 3;
    min_button_voltage = 1;
    max_button_voltage = 3;
    mid_button_voltage = 2;
  }

  /*
     Calculate 16 bit minimum and maximum values to send using - nr. of bits depends on DAC resolution
  */
  min = (min_voltage / 5) * 65535;
  max = (max_voltage / 5) * 65535;

  min_button = (min_button_voltage / 5) * 65535;
  max_button = (max_button_voltage / 5) * 65535;
  mid_button = (mid_button_voltage / 5) * 65535;

  //Set joystick buttons as digital inputs
  pinMode(button_up, INPUT);
  pinMode(button_down, INPUT);

  /* I2C Initialisation.
     Join I2C bus with address 8.
     Set function 'interrupt_routine()' as the ISR for I2C communication.
  */
  Wire.begin(8);
  Wire.onRequest(interrupt_routine);
  Serial.println("Setup Done");
}

void loop()
{
  /* 1. Read analog value
     2. If Value is fluctuating around midpoint, set output voltage to the midpoint voltage (avoid vibrations when arm is still)
     3. Map analog value to a value within the desired range
     4. Send value to the coressponding DAC channel
  */

  z_in = analogRead(z_pin);
  if (z_in > 495 && z_in < 525) z_in = 510;
  z_out = map(z_in, 0, 1023, min, max);
  SPI_send_DA(z_out, 0);

  y_in = analogRead(y_pin);
  if (y_in > 495 && y_in < 525) y_in = 510;
  y_out = map(y_in, 0, 1023, min, max);
  SPI_send_DA(y_out, 1);

  x_in = analogRead(x_pin);
  if (x_in > 495 && x_in < 525) x_in = 510;
  x_out = map(x_in, 0, 1023, min, max);
  SPI_send_DA(x_out, 2);

  /*If the up button is pressed, send the maximum digital voltage set in config file
     If the down button is pressed, send the minimum digital voltage set in the config file
     If both or bo buttons are pressed, send the midpoint voltage
  */
  up = digitalRead(button_up);
  down = digitalRead(button_down);

  if (up & ! down) SPI_send_DA(max_button, 3);
  else if (down & !up) SPI_send_DA(min_button, 3);
  else SPI_send_DA(mid_button, 3);

  joystick_data = String(x_in) + ">" + String(y_in) + ">" + String(z_in) + ">" + String(up) + ">" + String (down) + ">";

  /* Check if GPS is available
     You have to repeatedly funnel the characters to the GPS object from the GPS module using the encode() method
     FormatGPSData() formats the stored data in the GPS object for I2C transmission
  */
  while (serial_GPS.available() > 0) {
    if (gps.encode(serial_GPS.read()))
      formatGPSData();
  }

  data = GPS_data + joystick_data + "#";
  Serial.println("Data: " + data);
}

/*  The ISR (Interrupt Service Routine) for the I2C communication.
    A data buffer of 96 bytes is initialised, the limit of 96 bytes is imposed by the I2C library but can be changed inside the library!
    The Arduino string object is transformed to a char array.
    The data is send over I2C using the write function.
*/
void interrupt_routine() {
  char data_buf [96] = {};
  data.toCharArray(data_buf, 96);
  Wire.write(data_buf, 96);
}

void SPI_send_DA(unsigned int value, int channel)
{
  // Split value into two bytes as the SPI functions sends one byte at a time
  byte low = (value & 0x00FF);
  value  = value >> 8;
  byte high = (value & 0x00FF);
  digitalWrite(CS, LOW);
  delay(1);

  //Select DAC channel
  switch (channel)
  {
    case 0: //z-axis
      SPI.transfer(0b00010000);
      break;
    case 1://y-axis
      SPI.transfer(0b00010010);
      break;
    case 2://x-axis
      SPI.transfer(0b00010100);
      break;
    case 3://buttons
      SPI.transfer(0b00010110);
      break;
  }
  //Send value
  SPI.transfer(high);
  SPI.transfer(low);
  digitalWrite(CS, HIGH);
  delay(1);
}

/*
    This function returns a string holding the formatted GPS data.
*/
void formatGPSData() {
  String GPSData;

  if (gps.date.isValid())
  {
    GPSData += gps.date.month();
    GPSData += "/";
    GPSData += gps.date.day();
    GPSData += "/";
    GPSData += gps.date.year();
  }
  else
  {
    GPSData += "INVALID";
  }

  GPSData += " ";

  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) GPSData += "0";
    GPSData += gps.time.hour();
    GPSData += ":";
    if (gps.time.minute() < 10) GPSData += "0";
    GPSData += gps.time.minute();
    GPSData += ":";
    if (gps.time.second() < 10) GPSData += "0";
    GPSData += gps.time.second();
    GPSData += ".";
    if (gps.time.centisecond() < 10) GPSData += "0";
    GPSData += gps.time.centisecond();
  }
  else
  {
    GPSData += "INVALID";
  }
  GPSData += ">";
  if (gps.location.isValid())
  {
    GPSData += String(gps.location.lat(), 6);
    GPSData += ">";
    GPSData += String(gps.location.lng(), 6);
    GPSData += ">";
  }
  else
  {
    GPSData += "0.000000>0.000000>";
    //12/12/2012 00:00:00.00
  }
  GPS_data = GPSData;
}
