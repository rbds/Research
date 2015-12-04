/*
Program to calibrate Sharp GP2Y0A60SZLF Analog Distance Sensor 10-150cm, 5V.
Robot begins 4 inches (10 cm) from obstacle and averages 20 sensor readings.
Robot then drives back 1 inch and takes readings again.
Readings taken every inch from 4 to 60 inches (~ 10-150 cm).
Created Conor Lyman July 2015
Last update: 18 September 2015
*/


#include "DualVNH5019MotorShield.h"
#include "Encoder.h"
#include <Servo.h>

float sensorsum = 0;
float sensorValue = 0;
float n = 1.0;
float mean = 0;
int lastmean = 0;
float distance;
int i = 1;
float encAve = 0;
float pan_angle = 90; //ensure sensor pointed straight
int encoderInch = 791; //791 counts/inch
int startDist = 4; //begin 4 inches from obstacle

Servo pan;
Encoder enc1(2,3); //motor 1
Encoder enc2(18,19); //motor 2
Encoder enc3(20,21); //motor 3
DualVNH5019MotorShield md(22, 23, 24, A2, 28, 26, 25, A1); //pin assignments: (INA1, INB1, EN1DIAG1, CS1, INA2, INB2, EN2DIAG2, CS2) 

// The setup routine runs once when you press reset:
void setup() {
  Serial3.begin(38400);
  md.init();
  pan.attach(4);
  pan.write(pan_angle);
}

// The loop routine runs over and over again forever:
void loop() {
  while(distance<=60)
  {
    while(n <= 20)
    {
      sensorValue = analogRead(A15) * 5.0 / 1023.0;
      sensorsum = (sensorsum + sensorValue);
      mean = (sensorsum / n);
      n = n + 1;
      delay(20);
    }
    if(n > 20)
    {
      sensorsum = 0; 
      n = 1;
    }
    delay(100);
    encAve = (abs(enc1.read()) + abs(enc2.read())) / 2;
    distance = (encAve / encoderInch) + startDist;
    while((encAve/encoderInch) <= i)
    {
      md.setM1Speed(90);
      md.setM2Speed(-90);
      encAve = (abs(enc1.read()) + abs(enc2.read())) / 2;
    }
    md.setM1Brake(400);
    md.setM2Brake(400);
    i++;
    delay(1000);
    Serial3.print(mean);
    Serial3.print(", ");
    Serial3.println(distance);
    delay(25);
    }
}
