#include "Wire_Jens.h"
#include "WiFiEsp.h"
#include <SPI.h>
#include <SD.h>
#include "df_can.h"

#define SD_CS 4
#define CAN_CS 10
#define Mower_ID 1

MCPCAN CAN(CAN_CS);
union sensor_data_t
{
  float value;
  byte bytes[4];
};

const float sensor1_value = 20;
float sensor2_value;
float sensor3_value;
float sensor4_value;
float sensor5_value;
float temp;
int mowerID = Mower_ID;
boolean fault = 0;
String can_message = "idle";

uint32_t time_start;
uint32_t time_now;

bool can_update;
bool joystick_update;
bool sync;

unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];

/*
   Set network SSID (network name)
   Set network password
   Set wifi radio status
   reqCount saves the number of requests made
*/
char ssid[] = "ARDUINO_SERVER";
char pass[] = "123456789";
int status = WL_IDLE_STATUS;
int reqCount = 0;
int record;
String data;

WiFiEspServer server(8080);

void setup(){

  /*  Initialize Serial for monitor
      Initialize Serial1 for ESP module (Wifi)
      Initialize the I2C communication channel
  */
  Serial.begin(9600);
  Serial1.begin(115200);
  WiFi.init(&Serial1);
  Wire.begin();

  /*
      Count = The max numbers of initializations for the CAN-BUS, if initialize failed first!
      can_update = boolean set if a new CAN message has been received
      joystick_update = boolean set when no CAN message has been received for some time
      sync = boolean for starting the sync of data on SD card with the phone
  */
  int count = 50;
  can_update = false;
  joystick_update = false;
  sync = false;

  /*
     Initialize the CAN interface and shield
     Set CAN bus baudrate at 500kbps
  */
  do {
    CAN.init();
    if (CAN_OK == CAN.begin(CAN_500KBPS))
    {
      Serial.println("Status: CAN bus initialized");
      break;
    }
    else
    {
      Serial.println("Error: CAN bus initialization failed");
      delay(100);
      if (count <= 1)
        Serial.println("Error: CAN Bus initialization failed x15");
      Serial.println("Status: Please check the setup");
    }
  } while (count--);

  /* Set CAN bus Interrupt
  */
  attachInterrupt(0, MCP2515_ISR, FALLING);


  /* WIFI MODULE
     Initialize ESP module, configure access point
     Start the TCP server on port 8080
  */
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("Error: WiFi shield not present"));
    while (true);
  }
  Serial.print(F("Status: Attempting to start AP: "));
  Serial.println(ssid);

  IPAddress localIp(192, 168, 1, 1);
  WiFi.configAP(localIp);
  status = WiFi.beginAP(ssid, 10, pass, ENC_TYPE_WPA2_PSK);

  Serial.println(F("Status: Access point started "));
  printWifiStatus();

  server.begin();
  Serial.println(F("Status: TCP Server started"));
  delay(3000);
  time_start = millis();


  /* SD CARD
     See if the card is present and can be initialized
     If SD Card fails, program will do nothing: infinite while-loop
  */
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, 1);
  Serial.println("Status: Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("Error: Card failed, or not present");
    while (1);
  }
  Serial.println("Status: card initialized.");
  SD.remove("data_1.txt");
}

void loop() {

  /*
     Listen for incoming client connections
     When there is a client connected: ONLINE SCENARIO
     1. Sync any data from the SD card to the phone
     2. Update the joystick_update boolean: When 2 seconds have passed, joystick_update is set to true.
     3. Check if any CAN bus data had been written in the CAN buffer: Check this with can_update
     4. If 3. was true -> ask for GPS and Joystick values from slave, concatenate with new CAN data and send to driver Phone
     5. If 3. was false -> ask for GPS and Joystick values from slave and send to driver Phone
  */
  WiFiEspClient client = server.available();
  
  while (client) {
    SD_sync(client);
    String data;
    if(fault) data= "fault>";
    data += String(mowerID) + ">";
    time_now = millis();
    if (time_now - time_start > 2000) {
      joystick_update = true;
      time_start = millis();
    } else {
      joystick_update = false;
    }

    if (can_update) {
      Wire.requestFrom(8, 96);
      char c;
      while (Wire.available())
      {
        c = Wire.read();
        if (strcmp(c, '#') == 0) break;
        data += c;
      }
      data += String(sensor1_value) + ">" + String(sensor2_value) + ">" + String(sensor3_value) + ">" + String(sensor4_value) + ">" + String(sensor5_value) + ">" + String(temp) + ">" + can_message;
      if (client.connected()) {
        sendResponse(client, data);
        delay(10);
      } else {
        // client.stop();
        Serial.println(F("Status: Client disconnected"));
      }
      can_update = false;
    } else if (joystick_update) {
      Wire.requestFrom(8, 96);
      char c;
      while (Wire.available())
      {
        c = Wire.read();
        if (strcmp(c, '#') == 0) break;
        data += c;
      }
       data += String(sensor1_value) + ">" + String(sensor2_value) + ">" + String(sensor3_value) + ">" + String(sensor4_value) + ">" + String(sensor5_value) + ">" + String(temp) + ">" + can_message;
      joystick_update = false;

      if (client.connected()) {
        sendResponse(client, data);
        delay(10);
      } else {
        // client.stop();
        Serial.println(F("Status: Client disconnected"));
      }
    }
    CanGetData();
  }

  /*
    When there is no client connected: OFFLINE SCENARIO
    1. Update the joystick_update boolean: When 2 seconds have passed, joystick_update is set to true.
    2. Check if any CAN bus data had been written in the CAN buffer: Check this with can_update
    3. If 2. was true -> ask for GPS and Joystick values from slave, concatenate with new CAN data and log on SD card
    4. If 2. was false -> ask for GPS and Joystick values from slave and log on SD card
  */
  String data;
  if (fault) data= "fault>";
  data += String(mowerID) + ">";
  time_now = millis();
  if (time_now - time_start > 2000) {
    joystick_update = true;
    time_start = millis();
  } else {
    joystick_update = false;
  }

  if (can_update) {
    Wire.requestFrom(8, 96);
    char c;
    while (Wire.available())
    {
      c = Wire.read();
      if (strcmp(c, '#') == 0) break;
      data += c;
    }
    data += String(sensor1_value) + ">" + String(sensor2_value) + ">" + String(sensor3_value) + ">" + String(sensor4_value) + ">" + String(sensor5_value) + ">" + String(temp) + ">" + can_message ;
    SD_log(data);
    can_update = false;
  } else if (joystick_update) {
    Wire.requestFrom(8, 96);
    char c;
    String joystick_data;
    while (Wire.available())
    {
      c = Wire.read();
      if (strcmp(c, '#') == 0) break;
      data += c;
    }
     data += String(sensor1_value) + ">" + String(sensor2_value) + ">" + String(sensor3_value) + ">" + String(sensor4_value) + ">" + String(sensor5_value) + ">" + temp + ">" + can_message;
    joystick_update = false;
    SD_log(data);
  }

  CanGetData();
  sync = true;
}


void MCP2515_ISR()
{
  flagRecv = 1;
}

/*
   flagrecv is set in the intterupt routine of the CAN bus buffer
   CAN data is parsed and stored into global sensor values
   can_update is set true to send data to phone or log data on SD card in next cycle
*/
void CanGetData()
{
  if (flagRecv)
  {
    int id;
    while (CAN_MSGAVAIL == CAN.checkReceive())
    {
      CAN.readMsgBuf(&len, buf);
      id = CAN.getCanId();
      sensor_data_t sensor_data;
      for (int i = 0; i < len; i++)
      {
        sensor_data.bytes[i] = buf[i];
      }
      //Serial.println(sensor_data.value);
      switch (id) {
        case 1:
          sensor2_value = sensor_data.value;
          break;
        case 2:
          sensor3_value = sensor_data.value;
          break;
        case 3:
          sensor5_value = sensor_data.value;
          break;
        case 7:
          sensor4_value = sensor_data.value;
          break;
        case 9:
          temp = sensor_data.value;
          break;
        case 10:
          switch ((int) sensor_data.value) {
            case 58:
              can_message = "clean";
              fault = 0;
              break;
            case 59:
              can_message = "cooling";
              fault = 0;
              break;
            case 57:
              can_message = "idle";
              fault = 0;
              break;
          }
          break;
        case 16:
          can_message = "motor fault";
          fault = 1;
          break;
      }
      flagRecv = 0;
      can_update = true;
    }
  }
}


bool sendResponse(WiFiEspClient client, String data)
{
  int chars_send = client.println(data);
  return chars_send>0;
}


void printWifiStatus()
{
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);
}


void SD_log(String data)
{
  File dataFile = SD.open("data_1.txt", FILE_WRITE);
  if (dataFile) {
    Serial.println(data);
    dataFile.println(data);
    dataFile.close();
  } else {
    Serial.println("Error: File could not be opened");
  }
}

bool SD_sync(WiFiEspClient client)
{
  String buff;
  bool sync = true;

  File dataFile = SD.open("data_1.txt", FILE_READ);
  while (sync) {
    if (dataFile.available() > 0) {
      Serial.println("In Sync");
      buff = dataFile.readStringUntil('\n');
      //Serial.println(buff); // Debugging
    } else {
      dataFile.close();
      SD.remove("data_1.txt");
      sync = false;
      break;
    }

    if (client.connected()) {
      sendResponse(client, buff);
      delay(10);
    } else {
      // client.stop();
      // client = -1;
      sync = false;
    }
  }
  dataFile.close();
  return sync;
}
