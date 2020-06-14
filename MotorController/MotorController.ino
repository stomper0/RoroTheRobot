/*
 * MotorController.ino for RoroTheRobot
 * Alfie Pearce and Franklyn Watson
 * 2019
 */
 
#include <Adafruit_TCS34725.h>

#define in1 0   // motor A fwd/rev
#define in2 1   // motor A fwd/rev
#define in3 2   // motor B fwd/rev
#define enA 3   // motor A PWM signal
#define in4 4   // motor B fwd/rev
#define enB 5   // motor B PWM signal
#define pLed 6  // "programmed" LED
#define btn0 7  // living room
#define btn1 8  // hall
#define btn2 12 // kitchen
#define btn3 13 // dining room

Adafruit_TCS34725 leftColourSensor;
Adafruit_TCS34725 rightColourSensor;

int motorSpeedA = 0;
int motorSpeedB = 0;
int currentPos;
int destinationPos;

void setMotA(int dir) {					// Set the direction for left motor
  if (dir<0) {
    // Set Motor A backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir>0) {
    // Set Motor A forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  }
}

void setMotB(int dir) {					// Set the direction for right motor
    if (dir<0) {
    // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (dir>0) {
    // Set Motor B forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    // Set Motor B stopped
    digitalWrite(in3, HIGH);
    digitalWrite(in4, HIGH);
  }
}

void motStop() {
  setMotA(0);
  setMotB(0);
}

void aboutTurn() {
  setMotA(1);     // MotA fwd
  setMotB(-1);     // opposed to MotA
  // turn for 5 sec                   // correct this number
  motStop();        // stop turning
}

void setup() {
  pinMode(enA, OUTPUT);             // Setup for the H bridge pins
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(pLed, OUTPUT);            //"Programmed" LED pin
  pinMode(btn0, INPUT);             // Setup for the button pins
  pinMode(btn1, INPUT);
  pinMode(btn2, INPUT);
  pinMode(btn3, INPUT);

  Serial.begin(9600);
  //Serial.println or Serial.print
}

void loop() {
 
  int switchval = digitalRead(btn0) + digitalRead(btn1) + digitalRead(btn2) + digitalRead(btn3);
  if (switchval>0) {
    if (digitalRead(btn0) == HIGH) {
      destinationPos = 0;
    } else if (digitalRead(btn1) == HIGH) {
      destinationPos = 1;
    } else if (digitalRead(btn2) == HIGH) {
      destinationPos = 2;
    } else if (digitalRead(btn3) == HIGH) {
      destinationPos = 3;
    }
    digitalWrite(pLed, HIGH)
  }
//
//  while (currentPos != destinationPos) {
//    
//  }
//  

  
}
