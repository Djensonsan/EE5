// Receive data over I2C and send it over wifi to phone.
#include "Wire_Jens.h"
#include "WiFiEsp.h"
#include <SPI.h>
#include "df_can.h"

const int SPI_CS_PIN = 10;
MCPCAN CAN(SPI_CS_PIN); // Set CS pin

union sensor_data_t
{
  float value;
  byte bytes[4];
};

float sensor1_value;
float sensor2_value;
float sensor3_value;

uint32_t time_start;
uint32_t time_now;

bool can_update;
bool joystick_update;

unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];

char ssid[] = "ARDUINO_SERVER";         // your network SSID (name)
char pass[] = "123456789";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status
int reqCount = 0;   // number of requests received
int record;
String data;

WiFiEspServer server(8080);

void setup() {
  // WIFI MODULE
  Serial.begin(9600);
  Serial1.begin(115200);    // initialize serial for ESP module
  WiFi.init(&Serial1);
  Wire.begin();


  int count = 50;                                     // the max numbers of initializint the CAN-BUS, if initialize failed first!.
  can_update = false;
  joystick_update = false;
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


  attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("WiFi shield not present"));
    while (true); // don't continue
  }
  // initialize ESP module
  Serial.print(F("Attempting to start AP "));
  Serial.println(ssid);

  IPAddress localIp(192, 168, 1, 1);
  WiFi.configAP(localIp);

  // start access point
  status = WiFi.beginAP(ssid, 10, pass, ENC_TYPE_WPA2_PSK);

  Serial.println(F("Access point started"));
  printWifiStatus();

  // start the TCP server on port 8080
  server.begin();
  Serial.println(F("TCP Server started"));
  delay(3000);
}

void loop() {
  WiFiEspClient client = server.available();

  while (client) {
    String data;
    // Check timer for Joystick Update; Either restart timer or wait.
    time_now = millis();
    if (time_now - time_start > 2000) {
      joystick_update = true;
      time_start = millis();
    } else {
      joystick_update = false;
    }

    if (can_update) {
      Wire.requestFrom(8, 96); // request 96 bytes from slave device maximum #8, slave may send less than requested
      char c;
      while (Wire.available())   // slave may send less than requested
      {
        c = Wire.read();
        if (strcmp(c, '#') == 0) break;
        data += c;
      }
      data += " S1: " + String(sensor1_value) + " S2: " + String(sensor2_value) + " S3: " + String(sensor3_value);
      can_update = false;

      if (client.connected()) {
        sendResponse(client, data);
        delay(10);
      } else {
        client.stop();
        Serial.println(F("Client disconnected"));
      }
    } else if (joystick_update) {
      Wire.requestFrom(8, 96); // request 96 bytes from slave device maximum #8, slave may send less than requested
      char c;
      while (Wire.available())   // slave may send less than requested
      {
        c = Wire.read();
        if (strcmp(c, '#') == 0) break;
        data += c;
      }
      joystick_update = false;

      if (client.connected()) {
        sendResponse(client, data);
        delay(10);
      } else {
        client.stop();
        Serial.println(F("Client disconnected"));
      }
    }
  }

  if (flagRecv)
  {
    int id;
    flagRecv = 0;
    while (CAN_MSGAVAIL == CAN.checkReceive())
    {
      CAN.readMsgBuf(&len, buf);
      id = CAN.getCanId();
      sensor_data_t sensor_data;
      for (int i = 0; i < len; i++)
      {
        sensor_data.bytes[i] = buf[i];
      }
      Serial.println(sensor_data.value);
      switch (id) {
        case 1:
          sensor1_value = sensor_data.value;
          break;
        case 2:
          sensor2_value = sensor_data.value;
          break;
        case 3:
          sensor3_value = sensor_data.value;
          break;
      }
      can_update = true;
    }
  }
}

void MCP2515_ISR()
{
  flagRecv = 1;
}

void sendResponse(WiFiEspClient client, String data)
{
  client.println(data);
}


void printWifiStatus()
{
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);
}
