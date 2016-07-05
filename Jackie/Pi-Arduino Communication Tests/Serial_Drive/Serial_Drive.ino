// Serial Communication Driving Test
// Created Conor Lyman 27 October 2015
// Last update Conor Lyman 27 October 2015

#include "DualVNH5019MotorShield.h"

#define SIZE 5
int data[SIZE] = {5, 5, 5, 5, 5};
int i = 0;
int check = 0;
int checkCharacter = '$';
int inWaiting;
int vel = 150;

DualVNH5019MotorShield md(22, 23, 24, A7, 28, 26, 25, A1); //pin assignments: (INA1, INB1, EN1DIAG1, CS1, INA2, INB2, EN2DIAG2, CS2) 

void setup() {
  // put your setup code here, to run once:
  Serial3.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  inWaiting = 0;
  delay(100);
  check = Serial3.read();
  inWaiting = Serial3.available();
  if (check == checkCharacter && inWaiting == SIZE) {
    //Serial.println("Success!");
    for (i = 0; i < SIZE; i++) {
      data[i] = Serial3.read();
      data[i] = data[i] - 48;
    }
  }
  else if (check != checkCharacter && inWaiting != SIZE) {
    //Serial.println("Error");
    while (Serial3.available()) { Serial3.read(); }
  }
  drive();
  delay(50);
  data[0] = 2;
}

void drive() {
  if (data[0] == 1) {
    Serial3.print(1);
    md.setM1Speed(vel);
    md.setM2Speed(-vel);
  }
  else if (data[0] == 0) {
    brake();
  }
}

void brake() {
  Serial3.print(0);
  md.setM1Brake(400);
  md.setM2Brake(400);
}

