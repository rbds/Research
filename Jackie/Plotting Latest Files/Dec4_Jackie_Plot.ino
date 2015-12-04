//Created Conor Lyman 10 June 2015
//Last Update: 4 December 2015
/*Program has robot drive until it approaches object, then turn and continue driving. 
Information is sent over Xbee and plotted in MATLAB or Python */

#include "DualVNH5019MotorShield.h"
#include "Encoder.h"
#include <Servo.h>
#include "RunningMedian.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

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
DualVNH5019MotorShield md(22, 23, 24, A7, 28, 26, 25, A8); //pin assignments: (INA1, INB1, EN1DIAG1, CS1, INA2, INB2, EN2DIAG2, CS2) 
RunningMedian readings = RunningMedian(5); //Define that taking 5 readings
Adafruit_INA219 ina219; //Current sensor

int i = 0;
int j = 0;

// IR sensor
float distances[ARRAY_SIZE] = {4, 5.19666666666667, 6.25666666666667, 7.22333333333333, 8.22333333333333, 9.19333333333333, 10.2133333333333, 11.2133333333333, 12.2133333333333, 13.1733333333333, 14.2133333333333, 15.2333333333333, 16.2333333333333, 17.2166666666667, 18.2033333333333, 19.2, 20.25,  21.2666666666667, 22.2066666666667, 23.2133333333333, 24.1966666666667, 25.24,  26.2066666666667, 27.2, 28.16,  29.2033333333333, 30.21,  31.2333333333333, 32.19,  33.19,  34.2, 35.2266666666667, 36.25,  37.21,  38.2266666666667, 39.165, 40.24,  41.21,  42.2033333333333, 43.19,  44.2233333333333, 45.25,  46.2366666666667, 47.2366666666667, 48.2066666666667, 49.2, 50.25,  51.2733333333333, 52.2233333333333, 53.2466666666667, 54.2033333333333, 55.2566666666667, 56.24,  57.24,  58.2066666666667, 59.2166666666667, 60.26};  
float voltages[ARRAY_SIZE] = {3.58666666666667,  2.92666666666667, 2.52, 2.27333333333333, 2.03666666666667, 1.86333333333333, 1.73666666666667, 1.61666666666667, 1.53333333333333, 1.46333333333333, 1.40333333333333, 1.32666666666667, 1.28666666666667, 1.24333333333333, 1.21, 1.17, 1.15, 1.11333333333333, 1.09, 1.06333333333333, 1.04333333333333, 1.02666666666667, 1.01, 0.983333333333333,  0.97, 0.96, 0.94, 0.93, 0.91, 0.903333333333333,  0.893333333333333,  0.883333333333333,  0.87, 0.86, 0.85, 0.84, 0.84, 0.83, 0.813333333333333,  0.816666666666667,  0.8,  0.8,  0.796666666666667,  0.79, 0.783333333333333,  0.78, 0.776666666666667,  0.77, 0.766666666666667,  0.76, 0.763333333333333,  0.756666666666667,  0.75, 0.75, 0.74, 0.74, 0.74};
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
int countsDegree = 120; // Encoder counts per degree
int turnAngle = 90 * countsDegree; // Counts in 90 degrees

// Energy harvesting/monitoring
float voltageSolar = 0; // Measured voltage from solar panel.
float current_mA = 0; //stores current reading from sensor for 5V
float M1Current = 0; //Motor 1 current 
float M2Current = 0; //Motor 2 current 

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
  uint32_t currentFrequency; //current sensor initialization
  ina219.begin();
  ina219.setCalibration_32V_1A();
}

void loop() {
  time = String(long(millis())); //begin timing
  dynamicPan();
  if (distanceToObject > 20 || distanceToObject < 4) {
    driveStraight();
  }
  else if (distanceToObject <= 20 && distanceToObject >= 4) {
    brake();
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
  time = String(long(millis())); //continue reading time when stationary panning.
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
  delay(500);
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
        time = String(long(millis())); //continue reading time when stationary panning.
        data(2); //(2) indicates stationary panning. 
        readings.clear(); //reset "readings" array.
      }
    }
    j = 0; //reset counter
    delay(100);
    }
    pos_pan = 90;
}


void distanceSense() {
  reading = analogRead(A14);
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

  //Current reading
  current_mA = ina219.getCurrent_mA();
  String current = String(float(current_mA), 4);
  M1Current = md.getM1CurrentMilliamps();
  M2Current = md.getM2CurrentMilliamps();
  String motor1current = String(float(M1Current), 4);
  String motor2current = String(float(M2Current), 4);
  
  index = String(int(motParam)); //0 = brake, 1 = forward, 2 = stationary panning, 3 = turning
  
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
  angleTurned = encTotalTCount / countsDegree;
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
  Serial3.print(current + " ");
  Serial3.print(motor1current + " ");
  Serial3.print(motor2current + " ");
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
