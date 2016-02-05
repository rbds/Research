// Serial Communication Driving with Raspberry Pi
// Created Conor Lyman 27 October 2015
// Last update Conor Lyman 23 January 2016

// Raspberry Pi (Python) sends commands to Arduino of where to drive. 
// Arduino has (NEEDS) checks to brake if an obstacle is discovered while driving.

#include "DualVNH5019MotorShield.h" // Motorshield library
#include "Encoder.h" // Encoder library
#include <Wire.h> 
#include <Adafruit_INA219.h> // Current sensor library
#include <Servo.h>
#include "RunningMedian.h"

DualVNH5019MotorShield md(22, 23, 24, A7, 28, 26, 25, A1); //pin assignments: (INA1, INB1, EN1DIAG1, CS1, INA2, INB2, EN2DIAG2, CS2) 
Encoder enc1(2,3); // Encoder 1 pin assignments
Encoder enc2(18,19); // Encoder 2 pin assignments
Encoder enc3(20,21); // Encoder 3 pin assignments
Adafruit_INA219 ina219; // Current sensor
RunningMedian readings = RunningMedian(10); // Takes 10 readings and finds the median

String time;

#define ARRAY_SIZE 57 // Size of array that stores IR sensor data
#define DATASIZE 5 // Size of array of driving commands from Python
int data[DATASIZE] = {9, 9, 9, 9, 9}; // Stores commands sent to Arduino
int i, j, k = 0; // Counters
                 // i = Command string index
                 // j = Turn multiplier counter
                 // k = Drive multiplier counter
int check = 0; // Command initialization check. If 1, commands are executed
int checkCharacter = '$'; // First character of command must match symbol
int inWaiting; // Number of bytes in serial buffer
int vel = 150; // Driving velocity
int turnSpeed = 200; // Turning velocity
float straightGain = 791; // 791 encoder counts per inch 

// Encoders 
float encCount1, encCount2, encCount3 = 0; // Position of encoders
float encDrive, encTurn = 0; // Averages of encoder counts
int countsDegree = 120; // 120 encoder counts per degree of turn
int turnAngle = 30 * countsDegree; // Turn by factors of 30 degrees
float angleTurned; // Store angle turned by robot
int turnMult = 0; // Command sent to tell robot how many degrees to turn
                  // Degrees turned by robot = turnMult*30
int driveMult = 0; // Command sent to tell robot how many inches to drive forward
float encTotalTurn = 0; // Total encoder count from turning
float encDriveTotal = 0; // Total encoder count from driving 

// Strings sent by Arduino to command computer
String index; // Specifies if robot is braking, driving, or turning
String encoder1, encoder2, encoder3; // Encoder counts
String turn; // Angle turned
String current; // Current read by current sensor

float current_mA = 0; // Stores current read by current sensor in milliamps

// IR Sensor
float distances[ARRAY_SIZE] = {4, 5.19666666666667, 6.25666666666667, 7.22333333333333, 8.22333333333333, 9.19333333333333, 10.2133333333333, 11.2133333333333, 12.2133333333333, 13.1733333333333, 14.2133333333333, 15.2333333333333, 16.2333333333333, 17.2166666666667, 18.2033333333333, 19.2, 20.25,  21.2666666666667, 22.2066666666667, 23.2133333333333, 24.1966666666667, 25.24,  26.2066666666667, 27.2, 28.16,  29.2033333333333, 30.21,  31.2333333333333, 32.19,  33.19,  34.2, 35.2266666666667, 36.25,  37.21,  38.2266666666667, 39.165, 40.24,  41.21,  42.2033333333333, 43.19,  44.2233333333333, 45.25,  46.2366666666667, 47.2366666666667, 48.2066666666667, 49.2, 50.25,  51.2733333333333, 52.2233333333333, 53.2466666666667, 54.2033333333333, 55.2566666666667, 56.24,  57.24,  58.2066666666667, 59.2166666666667, 60.26};  
float voltages[ARRAY_SIZE] = {3.58666666666667,  2.92666666666667, 2.52, 2.27333333333333, 2.03666666666667, 1.86333333333333, 1.73666666666667, 1.61666666666667, 1.53333333333333, 1.46333333333333, 1.40333333333333, 1.32666666666667, 1.28666666666667, 1.24333333333333, 1.21, 1.17, 1.15, 1.11333333333333, 1.09, 1.06333333333333, 1.04333333333333, 1.02666666666667, 1.01, 0.983333333333333,  0.97, 0.96, 0.94, 0.93, 0.91, 0.903333333333333,  0.893333333333333,  0.883333333333333,  0.87, 0.86, 0.85, 0.84, 0.84, 0.83, 0.813333333333333,  0.816666666666667,  0.8,  0.8,  0.796666666666667,  0.79, 0.783333333333333,  0.78, 0.776666666666667,  0.77, 0.766666666666667,  0.76, 0.763333333333333,  0.756666666666667,  0.75, 0.75, 0.74, 0.74, 0.74};
  // Arrays store output voltages of IR sensor that correspond to distance to obstacle
  // Distances are determined by reading voltage from sensor and determining distance using 
  // linear interpolation
float distanceToObject = 0; //stores distance to an object as read by the IR sensor
float m; // Slope of interpolation line between data points
float isObstacle = 0;
float reading = 0; // Stores reading from distance sensor
String dist;
int distanceLeft = 0; // How much more is left to drive when avoiding obstacle

void setup() {
  Serial3.begin(38400); // Baudrate = 38400
  md.init(); // Motor driver initialization
  uint32_t currentFrequency; // Current sensor setup
  ina219.begin(); // Current sensor setup
  delay(200);
  enc1.write(0); // Write 0 to the encoders
  enc2.write(0);
  enc3.write(0);
}

void loop() {
  time = String(long(millis())); // Begin tracking time
  readData(); // Read the serial port
  if (data[0] == 1) { // If the first command is 1, drive
    driveMult = (data[1] * 10) + data[2]; // Drive works by driving the robot one inch times the 
                                          //driveMult. driveMult is made up of 
                                          //the second and third commands of the data array.
    drive(driveMult); // Pass the driveMult (the number of inches to drive) to the drive function
    }
  else if (data [0] == 0) { brake(); } // If the first command is 0, use the brake function
  else { Serial3.println("Something went wrong!"); } // The first command MUST be 0 or 1
  delay(50);
  for (i = 0; i < DATASIZE; i++) { data[i] = 9; } // Reset data array to "meaningless" numbers
}


void readData() {
  inWaiting = 0; // Number of incoming bytes in the serial buffer
  delay(100);
  check = Serial3.read(); // Read first byte in serial port
  inWaiting = Serial3.available(); // Store how many bytes are in serial buffer
  if (check == checkCharacter && inWaiting == DATASIZE) { // Read remaing bytes only if the 
                                                      //first character equals the check 
                                                      //and the number of bytes equals the desired SIZE
    for (i = 0; i < DATASIZE; i++) {
      data[i] = Serial3.read();
      data[i] = data[i] - 48; // Read serial and "convert" to ints
    }
  }
  else if (check != checkCharacter && inWaiting != DATASIZE) {
    while (Serial3.available()) { Serial3.read(); } // If conditions not met, read serial port
                                                    //while there is data to read in order to clear it
  }
}


void drive(int driveMult) { 
  // ************** NEED TO ADD DISTANCE SENSE FUNCTION CALL INTO DRIVE() *********************
  while (k < driveMult) { // Number of times loop runs = driveMult
    while (encDrive < straightGain) { // Loop runs until robot has driven one inch 
                                      // May need to change to something larger
                                      // Possible that one inch is too small
      time = String(long(millis()));
      md.setM1Speed(-vel);
      md.setM2Speed(vel);
      delay(50); // See if taking out delay helps "one inch" issue
      encCount1 = abs(enc1.read());
      encCount2 = abs(enc2.read());
      encCount3 = abs(enc3.read());
      encDrive = (encCount1 + encCount2 + encCount3) / 3; // Read and average encoders
                                                          // ... encoder 3 may be unused right now
                                                          // ... this would be contributing to issue
      delay(50); // Again, see if removing helps issue
    }
    distanceSense(); // Take a distance reading
    k++; // Iterate after every inch
    msg(1); // Send data; index 1 indicates driving
    encDriveTotal += encDrive; // Add to the total number of encoder counts and then reset for next loop
    encDrive = 0;
    encCount1 = 0;
    encCount2 = 0;
    encCount3 = 0;
    enc1.write(0);
    enc2.write(0);
    enc3.write(0);
    if (distanceToObject <= 20 && distanceToObject >= 4) {
      brake();
      verify();
      if (isObstacle == 1) {
        avoidObstacle(); // Function will drive robot to right away from obstacle
                         // a defined distance (5 inches?).
                         // The robot will then turn back and
                         // then check again for obstacles again.
                         // The robot will then drive forward (if there is no obstacle)
                         // a given distance before turning back to desired path.
        break;
      }
    }
  }
  k = 0; // Reset k when finished
  md.setM1Brake(400); // Stop driving
  md.setM2Brake(400);
  encDriveTotal = 0; // Why is encDriveTotal being reset... it's never used...
  encDrive = 0;
  encCount1 = 0;
  encCount2 = 0;
  encCount3 = 0;
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
}


void brake() {
  md.setM1Brake(400);
  md.setM2Brake(400);
  encCount1 = abs(enc1.read()); // Should be zero...
  encCount2 = abs(enc2.read());
  encCount3 = abs(enc3.read());
  time = String(long(millis())); 
  msg(0); // Send data; index 0 indicates robot is stationary
  enc1.write(0); // Reset encoders 
  enc2.write(0);
  enc3.write(0);
  encCount1 = 0;
  encCount2 = 0;
  encCount3 = 0;
  delay(200);
  if (data[3] == 1) { Turn(data[4]); } // Execture turn function if prompted
                                // Turn function is only executed in brake function to ensure
                                // brakes are set before turning. Prevents going directly from driving
                                // to braking
}


void Turn(int turnMult) {
  while (j < abs(turnMult)) {
    while (encTurn < turnAngle) { // Loop runs until robot has turned 30 degrees
      time = String(long(millis()));
      if (turnMult > 0) {
        md.setM1Speed(turnSpeed); // Robot turns to right (CW when viewed from above)
        md.setM2Speed(turnSpeed);
      }
      else if (turnMult < 0) { // Robot turns left (CCW when viewed from above)
        md.setM1Speed(-turnSpeed);
        md.setM2Speed(-turnSpeed);
      }
      encCount1 = abs(enc1.read());
      encCount2 = abs(enc2.read());
      encTurn = (encCount1 + encCount2) / 2; // Can add encoder 3 for turning... if connected
      // **************** DELAYS USED IN DRIVE LOOP NOT USED HERE ***************************
      // Suggests may be ok to take them out of drive loop
    }
    j++;
    encTotalTurn += encTurn; // Again... not convinced this is ever actually used
    encTurn = 0;
    encCount1 = 0;
    encCount2 = 0;
    enc1.write(0);
    enc2.write(0);
  }
  j = 0;
  md.setM1Brake(400);
  md.setM2Brake(400);
  msg(3); // Send data; index 3 indicates turning
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
  encCount1 = 0;
  encCount2 = 0;
  encTurn = 0;
  encTotalTurn = 0;
}


void distanceSense() {
  reading = analogRead(A14);
  delay(5);
  reading = reading * 5 / 1024; // Convert reading to a voltage between 0 and 5
  if (reading < 0.9 || reading > 3.8) {
    distanceToObject = 0;
    return;
  }
  delay(5);
  for (i = 0; i <= ARRAY_SIZE; i++) { // Loop through array and compare sensed voltage to 
                                      // array values
    if (reading >= voltages[i] && i > 0) {
      m = (distances[i] - distances[i-1]) / (voltages[i] - voltages[i-1]); // Inverse of slope of linear interpolation line
      distanceToObject = (m)*reading - (m)*voltages[i-1] + distances[i-1];
      distanceToObject = float(distanceToObject);
      break;
    }
  }
  if (reading > voltages[i] && i == ARRAY_SIZE) {
    Serial3.println("Something went wrong!");
  }
  i = 0; // Reset i
}


void verify() {
  int median = 0;
  int numReadings = 0;
  isObstacle = 0;
  while (numReadings < 10) // Take 10 readings and store in array for finding median
  {
    distanceSense();
    readings.add(distanceToObject);
    numReadings++;
  }
  median = readings.getMedian(); // Find median
  if (median < 10 && median > 4) { 
    isObstacle = 1;  // If the median distance is between 4 and 10 inches, there is an obstacle
    distanceToObject = median;
  }
  readings.clear();
  numReadings = 0;
  delay(100);
}

void avoidObstacle() {
  brake(); // Stop driving
  Turn(3); // Turn 90 degrees
  brake(); // Stop again
  delay(100);
  distanceSense(); // Take a distance reading
  if (distanceToObject >= 6 || distanceToObject == 0) { drive(5); } // Ensure nothing is in the way then drive 5 inches forward
  // Will need some sort of checks... issues bound to happen
  brake();
  Turn(-3); // Turn back to face original orientation
  brake();
  delay(100);
  distanceSense();
  if (distanceToObject > 0 && distanceToObject <= 6) { avoidObstacle(); } // Repeat steps if still obstacle
  else if (distanceToObject > 6 || distanceToObject == 0) { drive(5); } // Drive 5 inches forward if free
  brake();
  Turn(-3); // Turn to left (back towards original route)
  brake();
  delay(100);
  distanceSense();
  if (distanceToObject == 0 || distanceToObject > 6) { drive(5); } // Should now be back on original path
  brake();
  Turn(3); // Face in original orientation
  brake();
  delay(100);
  distanceSense();
  if (distanceToObject > 20 || distanceToObject < 4) {
    driveMult = (data[1] * 10) + data[2];
    distanceLeft = driveMult - 5 - k;
    drive(distanceLeft);
  }
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

  // Time
  if(time.length() == 1) { time = 000000 + time; }
  if(time.length() == 2) { time = 00000 + time; }
  if(time.length() == 3) { time = 0000 + time; }
  if(time.length() == 4) { time = 000 + time; }
  if(time.length() == 5) { time = 00 + time; }
  if(time.length() == 6) { time = 0 + time; }

  // Current
  current_mA = ina219.getCurrent_mA(); // Take current reading
  current = String(float(current_mA));
  if (current.length() == 1) { current = 0000000 + current; }
  if (current.length() == 2) { current = 000000 + current; }
  if (current.length() == 3) { current = 00000 + current; }
  if (current.length() == 4) { current = 0000 + current; }
  if (current.length() == 5) { current = 000 + current; }
  if (current.length() == 6) { current = 00 + current; }
  if (current.length() == 7) { current = 0 + current; } 

  // Distance to obstacle
  dist = String(float(distanceToObject));
  if (dist.length() == 4) { dist = 00 + dist; }
  if (dist.length() == 5) { dist = 0 + dist; }
  
  

  Serial3.print(index + " ");
  Serial3.print(encoder1 + " ");
  Serial3.print(encoder2 + " ");
  Serial3.print(encoder3 + " ");
  Serial3.print(turn + " ");
  Serial3.print(time + " ");
  Serial3.print(current + " ");
  Serial3.print(dist);
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
