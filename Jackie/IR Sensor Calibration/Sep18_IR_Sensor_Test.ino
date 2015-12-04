// Program to demonstrate use of IR sensor. 
// Program takes given input voltage and finds distance using known values.
// Created Conor Lyman 29 July 2015
// Last update: 18 September 2015

#define ARRAY_SIZE 57
float distances[ARRAY_SIZE] = {4, 5.19666666666667, 6.25666666666667, 7.22333333333333, 8.22333333333333, 9.19333333333333, 10.2133333333333, 11.2133333333333, 12.2133333333333, 13.1733333333333, 14.2133333333333, 15.2333333333333, 16.2333333333333, 17.2166666666667, 18.2033333333333, 19.2, 20.25,  21.2666666666667, 22.2066666666667, 23.2133333333333, 24.1966666666667, 25.24,  26.2066666666667, 27.2, 28.16,  29.2033333333333, 30.21,  31.2333333333333, 32.19,  33.19,  34.2, 35.2266666666667, 36.25,  37.21,  38.2266666666667, 39.165, 40.24,  41.21,  42.2033333333333, 43.19,  44.2233333333333, 45.25,  46.2366666666667, 47.2366666666667, 48.2066666666667, 49.2, 50.25,  51.2733333333333, 52.2233333333333, 53.2466666666667, 54.2033333333333, 55.2566666666667, 56.24,  57.24,  58.2066666666667, 59.2166666666667, 60.26};  
float voltages[ARRAY_SIZE] = {3.58666666666667,  2.92666666666667, 2.52, 2.27333333333333, 2.03666666666667, 1.86333333333333, 1.73666666666667, 1.61666666666667, 1.53333333333333, 1.46333333333333, 1.40333333333333, 1.32666666666667, 1.28666666666667, 1.24333333333333, 1.21, 1.17, 1.15, 1.11333333333333, 1.09, 1.06333333333333, 1.04333333333333, 1.02666666666667, 1.01, 0.983333333333333,  0.97, 0.96, 0.94, 0.93, 0.91, 0.903333333333333,  0.893333333333333,  0.883333333333333,  0.87, 0.86, 0.85, 0.84, 0.84, 0.83, 0.813333333333333,  0.816666666666667,  0.8,  0.8,  0.796666666666667,  0.79, 0.783333333333333,  0.78, 0.776666666666667,  0.77, 0.766666666666667,  0.76, 0.763333333333333,  0.756666666666667,  0.75, 0.75, 0.74, 0.74, 0.74};
float input1;
int i = 0;
int distance_size;
float y[2];
float x[2];
float point1;
float point2;
float value1;
float value2;
float m;
float distance1;
float sum = 0;
float mean = 0;
int j = 0;

void setup() {
  // put your setup code here, to run once:
  Serial3.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
  sum = 0;
  distanceSense();
  delay(100);
  for (i = 0; i <= ARRAY_SIZE; i++){
    if (input1 == 0) {
      Serial3.println("There is no object!");
      Serial3.print("i = ");
      Serial3.println(i);
      break;
    }
    if (input1 > voltages[i] && i == 0) {
      Serial3.println("Error! Input is greater than given voltage!");
      break;
    }
    else if(input1 == voltages[i]) {
      distance1 = distances[i];
      Serial3.print("The input is: ");
      Serial3.println(input1);
      Serial3.print("The distance is: ");
      Serial3.println(distance1);
      break;
    }
    else if (input1 > voltages[i] && i > 0) {
      point2 = distances[i];
      point1 = distances[i-1];
      value2 = voltages[i];
      value1 = voltages[i-1];
      x[0] = point1;
      x[1] = point2;
      y[0] = value1; 
      y[1] = value2;
      m = (y[1] - y[0]) / (x[1] - x[0]); // slope of linear interpolation line
      distance1 = (1/m)*input1 - (1/m)*y[0]+x[0];
      Serial3.print("The input is: ");
      Serial3.println(input1);
      Serial3.print("The upper bound is: ");
      Serial3.print(point1);
      Serial3.print(" ");
      Serial3.println(value1);
      Serial3.print("The lower bound is: ");
      Serial3.print(point2);
      Serial3.print(" ");
      Serial3.println(value2);
      Serial3.print("The calculated distance is: ");
      Serial3.println(distance1);
      break;
    }
    else if (input1 < voltages[i] && i == ARRAY_SIZE) {
      Serial3.println("Error! Input is smaller than any given voltages!");
      break;
    }
  }

  if (input1 > voltages[i] && i == ARRAY_SIZE) {
    Serial3.println("Something went wrong!!!");
  }
  
  delay(5000);
}

void distanceSense() {
  for (j = 1; j <= 5; j++) {
    sum = sum + analogRead(A15);
    delay(50);
  }
  mean = sum / (j - 1);
  input1 = mean * 5 / 1024;
}


