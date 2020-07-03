/*
 * MotorTestforPWMrange.ino
 * script for assessing the 'buzz' range on two motors
 * used to eliminate buzzing by excluding the range from a working sketch
 * Franklyn Watson
 */

#define enA 2   // motor A PWM signal
#define in1 22  // motor A fwd/rev
#define in2 24  // motor A fwd/rev
#define in3 26  // motor B fwd/rev
#define in4 28  // motor B fwd/rev
#define enB 3   // motor B PWM signal

int motorSpeedA;
int motorSpeedB;
int outSpeed = 0;
int dir = 1;

void setMotA(int dir) {          // Set the direction for left motor
  if (dir < 0) {
    // Set Motor A backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir > 0) {
    // Set Motor A forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  }
}

void setMotB(int dir) {         // Set the direction for right motor
  if (dir < 0) {
    // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (dir > 0) {
    // Set Motor B forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    // Set Motor B stopped
    digitalWrite(in3, HIGH);
    digitalWrite(in4, HIGH);
  }
}

void setup() {
  pinMode(enA, OUTPUT);             // Setup for the H bridge pins
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(9600);
  //Serial.println or Serial.print

  setMotA(1);
  setMotB(1);

  delay(5000);
}

void loop() {
  outSpeed += 5;
  if (outSpeed > 255) {
    outSpeed = 0;
    if (dir > -1) {
      dir--;
    } else {
      dir = 1;
    }
    setMotA(dir);
    setMotB(dir);
  }

  motorSpeedA = motorSpeedB = outSpeed;

  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B

  Serial.print("motA: "); Serial.print(motorSpeedA, DEC); Serial.print(" ");
  Serial.print("motB: "); Serial.print(motorSpeedB, DEC); Serial.print(" ");
  Serial.print("dir: "); Serial.print(dir, DEC); Serial.print(" ");
  Serial.print("outSpeed: "); Serial.print(outSpeed, DEC); Serial.print(" ");
  Serial.println(" ");

  delay(500);
}

// --- commented until motors tested by range, to find the lowest possible PWM range
//  // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
//  if (motorSpeedA < 70) {
//    motorSpeedA = 0;                      // atm looking like 35 and up, both directions
//  }
//  if (motorSpeedB < 70) {
//    motorSpeedB = 0;
//  }
