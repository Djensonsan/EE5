// Adapted By Jens Leysen
#include "DualG2HighPowerMotorShield.h"

// Shield Version
// Implementation for only M1
DualG2HighPowerMotorShield18v22 md;

void stopIfFault()
{
  if (md.getM2Fault())
  {
    md.disableDrivers();
  delay(1);
    Serial.println("M2 fault");
    while (1);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual G2 High Power Motor Shield");
  md.init();
  md.calibrateCurrentOffsets();

  delay(10);
}

void loop()
{
  md.enableDrivers();
  delay(1);  // The drivers require a maximum of 1ms to elapse when brought out of sleep mode.

  for (int i = 0; i <= 400; i++)
  {
    md.setM2Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
  }
    delay(2);
  }

  for (int i = 400; i > 0; i--)
  {
    md.setM2Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }

  md.disableDrivers(); // Put the MOSFET drivers into sleep mode.
  delay(500);
}
