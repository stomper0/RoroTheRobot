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

#define btn0 7  // living room
#define btn1 8  // hall
#define btn2 12 // kitchen
#define btn3 13 // dining room

Adafruit_TCS34725 ColorSensor;

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

  pinMode(btn0, INPUT);             // Setup for the button pins
  pinMode(btn1, INPUT);
  pinMode(btn2, INPUT);
  pinMode(btn3, INPUT);

  Serial.begin(9600);

  if (ColorSensor.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
  }
}

void loop() {
  uint16_t r, g, b, c, colorTemp, lux;

  ColorSensor.getRawData(&r, &g, &b, &c);
  // colorTemp = ColorSensor.calculateColorTemperature(r, g, b);
  colorTemp = ColorSensor.calculateColorTemperature_dn40(r, g, b, c);
  lux = ColorSensor.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");
//  int switchval = digitalRead(btn0) + digitalRead(btn1) + digitalRead(btn2) + digitalRead(btn3);
//  if (switchval>0) {
//    if (digitalRead(btn0) == HIGH) {
//      destinationPos = 0;
//    } else if (digitalRead(btn1) == HIGH) {
//      destinationPos = 1;
//    } else if (digitalRead(btn2) == HIGH) {
//      destinationPos = 2;
//    } else if (digitalRead(btn3) == HIGH) {
//      destinationPos = 3;
//    }
//  }
//
//  while (currentPos != destinationPos) {
//    
//  }
//  
//  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
//  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
}  
  // X-axis used for left and right control
//  if (xAxis < 470) {
//    // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
//    int xMapped = map(xAxis, 470, 0, 0, 255);
//    // Move to left - decrease left motor speed, increase right motor speed
//    motorSpeedA = motorSpeedA - xMapped;
//    motorSpeedB = motorSpeedB + xMapped;
//    // Confine the range from 0 to 255
//    if (motorSpeedA < 0) {
//      motorSpeedA = 0;
//    }
//    if (motorSpeedB > 255) {
//      motorSpeedB = 255;
//    }
//  }
//  if (xAxis > 550) {
//    // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
//    int xMapped = map(xAxis, 550, 1023, 0, 255);
//    // Move right - decrease right motor speed, increase left motor speed
//    motorSpeedA = motorSpeedA + xMapped;
//    motorSpeedB = motorSpeedB - xMapped;
//    // Confine the range from 0 to 255
//    if (motorSpeedA > 255) {
//      motorSpeedA = 255;
//    }
//    if (motorSpeedB < 0) {
//      motorSpeedB = 0;
//    }
//  }

// --- commented until motors tested by range, to find the lowest possible PWM range
//  // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
//  if (motorSpeedA < 70) {
//    motorSpeedA = 0;
//  }
//  if (motorSpeedB < 70) {
//    motorSpeedB = 0;
//  }
