// Serial Communication Driving with Raspberry Pi
// Created Conor Lyman 27 October 2015
// Last update Conor Lyman 4 November 2015

// Raspberry Pi sends commands to Arduino of where to drive. Arduino has checks to brake if 
// an obstacle is discovered while driving

#include "DualVNH5019MotorShield.h"
#include "Encoder.h"

DualVNH5019MotorShield md(22, 23, 24, A7, 28, 26, 25, A1); //pin assignments: (INA1, INB1, EN1DIAG1, CS1, INA2, INB2, EN2DIAG2, CS2) 
Encoder enc1(2,3);
Encoder enc2(18,19);
Encoder enc3(20,21);

String time;

#define SIZE 5
int data[SIZE] = {5, 5, 5, 5, 5};
int i = 0;
int j = 0;
int check = 0;
int checkCharacter = '$';
int inWaiting;
int vel = 150;
int turnSpeed = 200;

int encPos1, encPos2, encPos3 = 0;
float encCount1, encCount2, encCount3 = 0;
int oldCount1, oldCount2, oldCount3 = 0;
int encTurn = 0;
int countsDegree = 120;
int turnAngle = 30 * countsDegree;
float angleTurned;
int turnMult = 0;
float encTotalTurn = 0;

String index;
String encoder1, encoder2, encoder3;
String turn;


void setup() {
  // put your setup code here, to run once:
  Serial3.begin(38400);
  md.init();
  delay(200);
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  time = String(long(millis()));
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
  for (i = 0; i<SIZE; i++) { data[i] = 9; }
}


void drive() {
  if (data[0] == 1) {
    //Serial3.print(1);
    md.setM1Speed(-vel);
    md.setM2Speed(vel);
    delay(50);
    encCount1 = encPos1 - oldCount1;
    encCount2 = encPos2 - oldCount2;
    encCount3 = encPos3 - oldCount3;
    oldCount1 = encPos1;
    oldCount2 = encPos2;
    oldCount3 = encPos3;
    delay(50);
    msg(1);
  }
  else if (data[0] == 0) { brake(); }
}


void brake() {
  //Serial3.print(0);
  md.setM1Brake(400);
  md.setM2Brake(400);
  encPos1 = abs(enc1.read());
  encPos2 = abs(enc2.read());
  encPos3 = abs(enc3.read());
  encCount1 = encPos1 - oldCount1;
  encCount2 = encPos2 - oldCount2;
  encCount3 = encPos3 - oldCount3;
  time = String(long(millis())); //continue reading time when stationary panning.
  msg(0); //(0) indicates robot is stationary.
  enc1.write(0); //reset encoders for next motion.
  enc2.write(0);
  enc3.write(0);
  encCount1 = 0;
  encCount2 = 0;
  encCount3 = 0;
  oldCount1 = 0;
  oldCount2 = 0;
  oldCount3 = 0;
  delay(200);
  if (data[1] == 1) { Turn(); }
}


void Turn() {
  turnMult = data[2];
  while (j < turnMult) {
    while (encTurn < turnAngle) {
      time = String(long(millis()));
      md.setM1Speed(turnSpeed);
      md.setM2Speed(turnSpeed);
      encCount1 = abs(enc1.read());
      encCount2 = abs(enc2.read());
      encTurn = (encCount1 + encCount2) / 2;
    }
    j++;
    encTotalTurn += encTurn;
    encTurn = 0;
    encCount1 = 0;
    encCount2 = 0;
    enc1.write(0);
    enc2.write(0);
  }
  j = 0;
  md.setM1Brake(400);
  md.setM2Brake(400);
  msg(3);
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
  encCount1 = 0;
  encCount2 = 0;
  encTurn = 0;
  encTotalTurn = 0;
}


void msg(int motionParam) {
  index = String(int(motionParam)); // 0 = Brake; 1 = Forward; 3 = Turn;

  // Distance traveled linearly 
  encoder1 = String(abs(int(encCount1)));
  encoder2 = String(abs(int(encCount2)));
  encoder3 = String(abs(int(encCount3)));
  if (encoder1.length() == 1) { encoder1 = 000000 + encoder1; }
  if (encoder1.length() == 2) { encoder1 = 00000 + encoder1; }
  if (encoder1.length() == 3) { encoder1 = 0000 + encoder1; }
  if (encoder1.length() == 4) { encoder1 = 000 + encoder1; }
  if (encoder1.length() == 5) { encoder1 = 00 + encoder1; }
  if (encoder1.length() == 6) { encoder1 = 0 + encoder1; }
  if (encoder2.length() == 1) { encoder2 = 000000 + encoder2; }
  if (encoder2.length() == 2) { encoder2 = 00000 + encoder2; }
  if (encoder2.length() == 3) { encoder2 = 0000 + encoder2; }
  if (encoder2.length() == 4) { encoder2 = 000 + encoder2; }
  if (encoder2.length() == 5) { encoder2 = 00 + encoder2; }
  if (encoder2.length() == 6) { encoder2 = 0 + encoder2; }
  if (encoder3.length() == 1) { encoder3 = 000000 + encoder3; }
  if (encoder3.length() == 2) { encoder3 = 00000 + encoder3; }
  if (encoder3.length() == 3) { encoder3 = 0000 + encoder3; }
  if (encoder3.length() == 4) { encoder3 = 000 + encoder3; }
  if (encoder3.length() == 5) { encoder3 = 00 + encoder3; }
  if (encoder3.length() == 6) { encoder3 = 0 + encoder3; }

  // Degrees turned
  angleTurned = encTotalTurn / countsDegree;
  turn = String(int(abs(angleTurned)));
  if (turn.length() == 1) { turn = 000 + turn; }
  if (turn.length() == 2) { turn = 00 + turn; }
  if (turn.length() == 3) { turn = 0 + turn; }

  if(time.length() == 1) { time = 000000 + time; }
  if(time.length() == 2) { time = 00000 + time; }
  if(time.length() == 3) { time = 0000 + time; }
  if(time.length() == 4) { time = 000 + time; }
  if(time.length() == 5) { time = 00 + time; }
  if(time.length() == 6) { time = 0 + time; }

  Serial3.print(index + " ");
  Serial3.print(encoder1 + " ");
  Serial3.print(encoder2 + " ");
  Serial3.print(encoder3 + " ");
  Serial3.print(turn + " ");
  Serial3.println(time);
}


void stopIfFault()
  {
    if (md.getM1Fault())
    {
      Serial.println("M1 fault");
      while(1);
    }
    if (md.getM2Fault())
    {
      Serial.println("M2 fault");
      while(1);
    }
  }
