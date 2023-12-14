#include <Romi32U4.h>
#include "IR_sensor.h"

void IRsensor::Init(void)
{
    pinMode(pin_IR, INPUT);
}

void IRsensor::PrintData(void)
{
    Serial.println(ReadData());
}

float IRsensor::ReadData(void)
{
  //sensor transfer function is approx. inverse (Vout=C/dist; dist=C/Vout)
  return 28.70226626 / (analogRead(pin_IR)*5.0/1023); // 
}