//Created Conor Lyman 10 June 2015
//Last Update: 11 September 2015
/*Program has robot drive until it approaches object, then turn and continue driving. 
Information is sent over Xbee and plotted in MATLAB or Python */

#include "DualVNH5019MotorShield.h"
#include "Encoder.h"
#include <Servo.h>
#include "RunningMedian.h"

#define ARRAY_SIZE 57 // Size of arrays that store IR sensor data

// Strings to send over XBee
String index; //Indicates whether robot is moving straight or turning
String dist;
String encoder1; 
String encoder2;
String encoder3;
String turn; //Data readings are converted to strings when sent over Xbee
String voltsSolar; //Stores voltage reading of solar panel
String voltsThermo;
String Servoangle; //Stores angle of pan servo
String time;

Servo pan;
Encoder enc1(2,3); //motor 1 encoder pins
Encoder enc2(18,19); //motor 2 encoder pins
Encoder enc3(20,21); //motor 3 encoder pins
DualVNH5019MotorShield md(22, 23, 24, A7, 28, 26, 25, A1); //pin assignments: (INA1, INB1, EN1DIAG1, CS1, INA2, INB2, EN2DIAG2, CS2) 
RunningMedian readings = RunningMedian(5); //Define that taking 5 readings

int i = 0;
int j = 0;

// IR sensor
float distances[ARRAY_SIZE] = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60};
float voltages[ARRAY_SIZE] = {3.65,  2.95, 2.54, 2.25, 2.04, 1.84, 1.72, 1.59, 1.5,  1.43, 1.39, 1.32, 1.26, 1.23, 1.19, 1.16, 1.13, 1.09, 1.06, 1.03, 1.01, 1,  0.98, 0.96, 0.94, 0.93, 0.92, 0.91, 0.89, 0.87, 0.88, 0.86, 0.86, 0.85, 0.82, 0.82, 0.82, 0.81, 0.79, 0.79, 0.77, 0.78, 0.77, 0.78, 0.77, 0.76, 0.76, 0.74, 0.74, 0.74, 0.74, 0.72, 0.72, 0.72, 0.72, 0.7,  0.7};
float distanceToObject = 0; //stores distance to an object as read by the IR sensor
float m;
float reading = 0;
float median = 0;

// Encoders
float enc1SCount = 0; // encPos1 - oldCount1. Gives distance travelled since last reading
float enc2SCount = 0;
float enc3SCount = 0; 
float enc1TCount = 0; // Encoder counts when robot is turning
float enc2TCount = 0;
float enc3TCount = 0;
float encTotalTCount = 0; // Average of three encoder readings when turning
float angleTurned = 0; // Angle robot has turned based on encTotalTCount
int oldCount1 = 0; // Previous encoder count when driving straight
int oldCount2 = 0;
int oldCount3 = 0;
int encPos1; // Reading of encoders
int encPos2;
int encPos3;

// Speeds
int turnSpeed = 250; // Turning speed
int straightSpeed = 100; // Straight driving speed

// Panning
int pos_pan = 90; // Starting position of pan (90 degrees is straight)
float pan_angle; // Stores pan angle
int servoTurn = 0; // Variable to tell servo which direction to pan in

// Turning
float isObstacle = 0;
int turnAngle = 90 * 115; // Amount robot turns. 115 encoder counts/degree

// Energy harvesting
float voltageSolar = 0; // Measured voltage from solar panel.

void setup() {
  pan.attach(4);
  pan.write(pos_pan); //begin pointing straight
  pan_angle = pan.read(); //pan starts at 90 degrees
  Serial3.begin(38400);
  md.init();
  delay(200);
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
}

void loop() {
  time = String(long(millis())); //begin timing
  dynamicPan();
  if (distanceToObject > 20 || distanceToObject < 4) {
    driveStraight();
  }
  else if (distanceToObject <= 20 && distanceToObject >= 4) {
    brake();
    delay(200);
    stationaryPan();
    if (isObstacle == 1) { robotTurn(); }
    delay(200);
    distanceSense(); 
  }
  stopIfFault();
}


void driveStraight() { //only sends encoder counts traversed since last data reading.
  md.setM1Speed(-straightSpeed);
  md.setM2Speed(straightSpeed);
  delay(50);
  encPos1 = abs(enc1.read());
  encPos2 = abs(enc2.read());
  encPos3 = abs(enc3.read());
  enc1SCount = encPos1 - oldCount1;
  enc2SCount = encPos2 - oldCount2;
  enc3SCount = encPos3 - oldCount3;
  oldCount1 = encPos1;
  oldCount2 = encPos2;
  oldCount3 = encPos3;
  delay(50);
  data(1); //(1) indicates straight motion in data
}


void brake() { //must read encoders in brake function to account for encoder counts while slowing down.
  md.setM1Brake(400);
  md.setM2Brake(400);
  encPos1 = abs(enc1.read());
  encPos2 = abs(enc2.read());
  encPos3 = abs(enc3.read());
  enc1SCount = encPos1 - oldCount1;
  enc2SCount = encPos2 - oldCount2;
  enc3SCount = encPos3 - oldCount3;
  data(0); //(0) indicates robot is stationary.
  enc1.write(0); //reset encoders for next motion.
  enc2.write(0);
  enc3.write(0);
  enc1SCount = 0;
  enc2SCount = 0;
  enc3SCount = 0;
  oldCount1 = 0;
  oldCount2 = 0;
  oldCount3 = 0;
  delay(1000);
}


void robotTurn() {
  pos_pan = 90;
  pan.write(pos_pan); //reset pan position
  delay(50);
  pan.detach(); //servo was moving during turning unless detached
  servoTurn = 0;
  while (encTotalTCount < turnAngle) {
    time = String(long(millis()));
    md.setM1Speed(turnSpeed);
    md.setM2Speed(turnSpeed);
    enc1TCount = abs(enc1.read());
    enc2TCount = abs(enc2.read());
    encTotalTCount = (enc1TCount + enc2TCount) / 2;
  }
  md.setM1Brake(400);
  md.setM2Brake(400);
  data(3);
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
  encTotalTCount = 0;
}


void dynamicPan() { //panning while driving
  delay(50);
  if (pan.attached() == false) { //if the servo has been detached (from turning), reattach it
    pan.attach(4);
    delay(50);
  }
  if (servoTurn == 0) { //0 indicates panning to robot's left
    pos_pan -= 20; //pan by increments of 15 degrees to the left
    pan.write(pos_pan);
    pan_angle = pan.read();
    delay(50); 
    distanceSense(); 
    if (pos_pan == 70) { servoTurn = 1; } //begin panning to the right when pan angle reaches 70 degrees
  }
  else if (servoTurn == 1) { //1 indicates panning to robot's right
    pos_pan += 20; //pan by increments of 15 degrees to the right.
    pan.write(pos_pan);
    pan_angle = pan.read();  
    delay(50); 
    distanceSense(); 
    if (pos_pan == 110) { servoTurn = 0; } //begin panning to the left when pan angle reaches 110 degrees
  } 
}


void stationaryPan() { //panning while robot is stationary
  median = 0;
  isObstacle = 0;
  if (pan.attached() == false) {
    pan.attach(4);
    delay(50);
  }
  for (pos_pan = 60; pos_pan < 120; pos_pan = pos_pan + 5) { //pan by increments of 5 degrees.
    pan.write(pos_pan);
    pan_angle = pan.read();
    delay(100);
    while(j < 5) //take 5 distance readings and store them in array "readings"
    {
      distanceSense();
      readings.add(distanceToObject);
      j++;  
      if(j == 5) //once 5 distance readings are taken, find the median of the 5 readings.
      {
        median = readings.getMedian();
        if (median < 20 && median > 4 && isObstacle == 0) { isObstacle = 1; }
        distanceToObject = median;
        data(2); //(2) indicates stationary panning. 
        readings.clear(); //reset "readings" array.
      }
    }
    j = 0; //reset counter
    time = String(long(millis())); //continue reading time when stationary panning.
    delay(100);
    }
    pos_pan = 90;
}


void distanceSense() {
  reading = analogRead(A15);
  delay(50);
  reading = reading * 5 / 1024; //convert reading to a voltage between 0 and 5
  if (reading < 0.9 || reading > 3.8) {
    distanceToObject = 0;
    return;
  }
  delay(50);
  for (i = 0; i <= ARRAY_SIZE; i++){
    if (reading >= voltages[i] && i > 0) {
      m = (distances[i] - distances[i-1]) / (voltages[i] - voltages[i-1]); // inverse of slope of linear interpolation line
      distanceToObject = (m)*reading - (m)*voltages[i-1] + distances[i-1];
      distanceToObject = float(distanceToObject);
      break;
    }
  }
  if (reading > voltages[i] && i == ARRAY_SIZE) {
    Serial3.println("Something went wrong!!!");
  }
  i = 0;
}


void data(int motParam) {
  // Read voltage of solar panel
  int solarSensorValue = analogRead(A3);
  float voltageSolar = solarSensorValue * (5.0 / 1023.0);
  voltsSolar = String(float(voltageSolar),4);
  int thermoSensorValue = analogRead(A9);
  float voltageThermo = thermoSensorValue * (5.0 / 1023.0);
  voltsThermo = String(float(voltageThermo),4);
  
  index = String(int(motParam)); //0 = stationary, 1 = forward, 3 = turning
  
  //Distance from obstacle
  dist = String(float(distanceToObject));
  if (dist.length() == 4) { dist = 00 + dist; }
  if (dist.length() == 5) { dist = 0 + dist; }

  //Encoder data used to determine distance traveled straight
  encoder1 = String(abs(int(enc1SCount)));
  encoder2 = String(abs(int(enc2SCount)));
  encoder3 = String(abs(int(enc3SCount)));
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

  //Caculate number of degrees turned
  angleTurned = encTotalTCount / 115;
  turn = String(int(abs(angleTurned)));
  if (turn.length() == 1) { turn = 000 + turn; }
  if (turn.length() == 2) { turn = 00 + turn; }
  if (turn.length() == 3) { turn = 0 + turn; }

  //Angle of servo pan
  Servoangle = String(float(pan_angle));
  if (Servoangle.length() == 4) { Servoangle = 00 + Servoangle; }
  if (Servoangle.length() == 5) { Servoangle = 0 + Servoangle; }
  
  if(time.length() == 1) { time = 000000 + time; }
  if(time.length() == 2) { time = 00000 + time; }
  if(time.length() == 3) { time = 0000 + time; }
  if(time.length() == 4) { time = 000 + time; }
  if(time.length() == 5) { time = 00 + time; }
  if(time.length() == 6) { time = 0 + time; }

  //print everything
  Serial3.print(index + " ");
  Serial3.print(dist + " ");
  Serial3.print(encoder1 + " ");
  Serial3.print(encoder2 + " ");
  Serial3.print(encoder3 + " ");
  Serial3.print(turn + " ");
  Serial3.print(voltsSolar + " ");
  Serial3.print(voltsThermo + " ");
  Serial3.print(Servoangle + " ");
  Serial3.println(time);

  delay(100);
}


void stopIfFault()      //another mechanism besides the Serial Monitor needs to be used to signify a motor fault if the Arduino is not plugged into a computer.
                        //possibly use the XBees to make it show up on the laptop screen.
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
