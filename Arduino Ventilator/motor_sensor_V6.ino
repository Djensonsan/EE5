//This is a FSM-controlled application
#include "DualG2HighPowerMotorShield.h"
#include "df_can.h"
#include <SPI.h>
#include <Timer.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define max_speed 400
#define min_speed -400
#define Temperature_difference 5  //Send temperature through CAN when there is a change > than this value

#define IDLE 57
#define CLEAN_MOTOR 58
#define COOL_MOTOR 59
#define SLOW_END 61

#define threshold_high_temperature 21 // Start ventilator above this temperature
#define threshold_low_temperature 20 // Stop ventilator below this temperature
#define clean_time 10000 //in milliseconds for ventilator: 30 seconds
#define cool_time 10000 //in milliseconds for ventilator: 30 seconds
#define temperature_update_time 2000 //in milliseconds every X seconds read the temperature once

/*CAN Bus*/
MCPCAN CAN(10);      //Set CS pin of CAN shield
union sensor_data_t   //struct to split data into bytes to be sent over CAN  bus
{
  float value;
  byte bytes[4];
};

unsigned long timer_counter = 0;
unsigned long temperature_counter = 0;

const int prescale  = 1024;
const int ocr2aval  = 15;

const float iinterval = prescale * (ocr2aval + 1) / (F_CPU / 1.0e6);

int state = IDLE; //state variable initialized at Idle
int motor_speed;
int temperature = A3; //A3 as analog input to detect temperature
int power = 4; //digital 4 as digital output to provide power for the temperature sensor
int led_indicator = 5; //digital 5 as LED indicator to show everything is fine

bool temp_flag = true; //true: to take a temperature reading
bool can_state_flag = true; //true: to send the current state through CAN
bool motor_counter_flag = false; //true: to start counting the time

//control_1 control_2: 00=>timer to count update time/ 01=>timer to count power time/ 10=>timer to count average/
bool temp_control_1 = false;
bool temp_control_2 = false;

float sum_temperature = 0; //the sum of sampled temperature, used for calculating the avg_temperature
float avg_temperature; //the avg temperature value used for control the motor
float previous_avgtem; //used for CANBus sending

int previous_working_state;

DualG2HighPowerMotorShield18v22 md; //the motor we control


void setup() {

  Serial.begin(9600); //setup serial
  while (!Serial) { // wait for serial port to connect. Needed for Leonardo only
  }
  timerSetup();          //initialize timer2 to count 1ms

  canSetup();             //initialize CAN bus shield

  pinMode(power, OUTPUT); //turn off the power of the temperature sensor
  digitalWrite(power, LOW);

  pinMode(led_indicator, OUTPUT); //turn off the LED in setup
  digitalWrite(led_indicator, LOW);

  md.init();
  md.calibrateCurrentOffsets();

  previous_avgtem = 0; //initialize the avg_temp
  motor_speed = 0;
  
  delay(10);
  md.enableDrivers();
  delay(1);  // The drivers require a maximum of 1ms to elapse when brought out of sleep mode.
}


void loop() {

  digitalWrite(led_indicator, HIGH); //as long as inside the loop, LED is on
  sendStateThroughCan();            //check the state flag, send state through can if the flag is set

  switch (state) {
    case IDLE: //M1 doesn't work in Idle state
      /*nothing is done here*/
      if (avg_temperature >= threshold_high_temperature)
      {
        state = CLEAN_MOTOR;
        can_state_flag = true;
        timer_counter = 0; //reset timer
      }
      break;

    case SLOW_END:
      slow_end();
      if(motor_speed == 0){
      timer_counter = 0; //reset timer
      if (check_temperature())
      {
        state = IDLE;
        can_state_flag = true;
      }
      else {
        switch (previous_working_state)
        {
          case COOL_MOTOR:
            state = CLEAN_MOTOR;
            can_state_flag = true;
            break;
          case CLEAN_MOTOR:
            state = COOL_MOTOR;
            can_state_flag = true;
            break;
        }
      }
      }
      break;

    case CLEAN_MOTOR:
      clean_motor();

      if (timer_counter >= clean_time)
      {
        state = SLOW_END;
        can_state_flag = true;
        timer_counter = 0; //reset timer
        previous_working_state = CLEAN_MOTOR;
      }
      break;

    case COOL_MOTOR:
      cool_motor();

      if (timer_counter >= cool_time)
      {
        state = SLOW_END;
        can_state_flag = true;
        timer_counter = 0; //reset timer
        previous_working_state = COOL_MOTOR;
      }
      break;
  }

}


// ISR For Timer 2 Compare-match overflow
ISR(TIMER2_COMPA_vect)
{
  timer_counter++;

  //control_1 control_2: 00=>timer to count update time/ 01=>timer to count power time/ 11=>timer to count average
  if ((!temp_control_1) & (!temp_control_2)) //00 count update time
  {
    temperature_counter++;
    if (temperature_counter >= temperature_update_time)
    {
      temp_control_2 = true; //now becomes 01
      temperature_counter = 0; //reset temperature_counter
      digitalWrite(power, HIGH);
    }
  }

  if ((!temp_control_1) & temp_control_2) //01 count power time
  {
    temperature_counter++;
    if (temperature_counter >= 100)
    {
      temp_control_1 = true;    //now becomes 11
      temperature_counter = 0;  //reset temperature_counter
    }
  }

  if (temp_control_1 & temp_control_2) //11=>timer to count average
  {
    temperature_counter++;
    if (temperature_counter % 16 == 0) //every 16ms read a temperature value
    {
      sum_temperature = sum_temperature + calculateT(calculateR(analogRead(temperature)));
    }
    if (temperature_counter >= 256) //calculate avg when having 16 samples
    {
      avg_temperature = sum_temperature / 16;

      Serial.print("Temperature is ");
      Serial.print(avg_temperature);
      Serial.println("C");

      if (abs(avg_temperature - previous_avgtem) >= Temperature_difference)
      {
        sendThroughCanBus(9, avg_temperature); //send the new temperature through CANbus
        previous_avgtem = avg_temperature;
      }

      temp_control_1 = false;
      temp_control_2 = false;   //now becomes 00
      digitalWrite(power, LOW); //turn off the power
      temperature_counter = 0;  //reset temperature_counter
      sum_temperature = 0;      //clear the last sum
    }
  }
}

//timer setup
void timerSetup()
{
  cli();  //disable global interrupts while initializing counter registers.
  TCCR2A = (1 << WGM21); // Set Timer 2 CTC mode
  TCCR2B = 0b00000111;   // Set prescaler to 1024
  TIMSK2 = (1 << OCIE2A); // Enable Compare-match register A interrupt for timer2
  OCR2A = ocr2aval; // This value determines the interrupt interval
  sei(); // Enable global interrupts

}

// Calculate Resistance
float calculateR(double value)
{
  float resistance = 10000 * value / (1023 - value);
  return resistance;
}


// Calculate Temperature
float calculateT(float resistor)
{
  float temp = log(resistor / 10000);
  temp = temp / 3435;
  temp = temp + 1 / (25 + 273.15);
  temp = 1 / temp;
  temp = temp - 273.15;
  return temp;
}

//return true when the tank is cooled
bool check_temperature()
{
  return avg_temperature < threshold_low_temperature;
}


// Default behavior when a fault occurs: Disable drivers (shutdown ventilator) and don't ever start up = manual reset needed!
void stopIfFault()
{
  if (md.getM1Fault())
  {
    md.disableDrivers();
    delay(1);
    Serial.println("M1 fault");
    while (1)
    {
      digitalWrite(led_indicator, LOW);
      delay(1000);
      digitalWrite(led_indicator, HIGH);
      delay(1000);
      sendThroughCanBus(16, 0);
    }
  }
}


// Slow end state
void slow_end()
{
  if(timer_counter == 16)
  {
  if (motor_speed > 0) {
    motor_speed--;
    md.setM1Speed(motor_speed);
  }
  else if (motor_speed < 0) {
    motor_speed++;
    md.setM1Speed(motor_speed);
  }
  timer_counter = 0;
  }
}


// Routine for cooling the motor
void cool_motor()
{
  if(motor_speed != max_speed)
  {
     if(timer_counter%100==0)
    {
    motor_speed++;
    md.setM1Speed(motor_speed);
    }
    } 
    else
  {md.setM1Speed(max_speed);}
   
  stopIfFault();
   }


// Routine for cleaning the motor
void clean_motor()
{
  if(motor_speed != min_speed)
  {
    if(timer_counter%100==0)
    {
    motor_speed--;
    md.setM1Speed(motor_speed);
    }
    } 
    else
  {md.setM1Speed(min_speed);}
  stopIfFault();
 
  

}

/*****************************Function to send data through CAN**********************************************/
void sendThroughCanBus(int id, float data) //id == 9:send temperature, id == 10: send workingstate, id == 16: send M1Fault
{
  sensor_data_t sensor_data_x;
  sensor_data_x.value = data;
  byte data_x[4] = {sensor_data_x.bytes[0], sensor_data_x.bytes[1], sensor_data_x.bytes[2], sensor_data_x.bytes[3]};
  CAN.sendMsgBuf(id, 0, 4, data_x);
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

/*******************check the state flag and send current state if the flag is true**********************/
void sendStateThroughCan()
{
  if (can_state_flag == true)
  {
    sendThroughCanBus(10, float(state));
    Serial.print("Going to ");
    Serial.println(state);
    can_state_flag = false;
  }

}


