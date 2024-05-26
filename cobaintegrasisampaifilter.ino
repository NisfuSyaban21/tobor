#include <Arduino.h>

// Definisi pin motor
const int motor1Pin1 = 22;  // Pin motor 1
const int motor1Pin2 = 23;
const int motor1pwm = 4;

const int motor2Pin1 = 24;  // Pin motor 2
const int motor2Pin2 = 25;
const int motor2pwm = 5;

const int motor3Pin1 = 26;  // Pin motor 3
const int motor3Pin2 = 27;
const int motor3pwm = 6;

const int motor4Pin1 = 28;  // Pin motor 4
const int motor4Pin2 = 29;
const int motor4pwm = 7;

const int encA1 = 21;
const int encB1 = 42;
const int encA2 = 20;
const int encB2 = 40;
const int encA3 = 18;
const int encB3 = 34;
const int encA4 = 19;
const int encB4 = 36;

const float kel_roda = 37.68;
const float pulse_per_rotation = 135; // Motor1

int pulse1 = 0;
int pulse2 = 0;
int pulse3 = 0;
int pulse4 = 0;
float x_enc = 0;
float y_enc = 0;

float jarakroda3;
float jarakroda4;


#define ULTRASONIC_PIN 59  // Pin used for Trig and Echo

#define s0 54        // Module pins wiring
#define s1 55
#define s2 56
#define s3 57
#define out 58

const int motorroller1Pin1 = 43;  // Pin motor 1
const int motorroller1Pin2 = 45;
const int motorroller1pwm = 13;

const int motorroller2Pin1 = 39;  // Pin motor 2
const int motorroller2Pin2 = 41;
const int motorroller2pwm = 12;
void setup() {
  Serial.begin(9600);

  pinMode(ULTRASONIC_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_PIN, LOW);
  delay(2);

  pinMode(s0, OUTPUT);     // Set pin modes
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);

  pinMode(motor1pwm, OUTPUT);
  pinMode(motor2pwm, OUTPUT);
  pinMode(motor3pwm, OUTPUT);
  pinMode(motor4pwm, OUTPUT);

  pinMode(motorroller1Pin1, OUTPUT);
  pinMode(motorroller1Pin2, OUTPUT);
  pinMode(motorroller2Pin1, OUTPUT);
  pinMode(motorroller2Pin2, OUTPUT);

  pinMode(motorroller1pwm, OUTPUT);
  pinMode(motorroller2pwm, OUTPUT);



  pinMode(encA1, INPUT_PULLUP);
  pinMode(encB1, INPUT_PULLUP);
  pinMode(encA2, INPUT_PULLUP);
  pinMode(encB2, INPUT_PULLUP);
  pinMode(encA3, INPUT_PULLUP);
  pinMode(encB3, INPUT_PULLUP);
  pinMode(encA4, INPUT_PULLUP);
  pinMode(encB4, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encA1), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(encA3), readEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(encA4), readEncoder4, RISING);

  digitalWrite(s0, HIGH);  // Set frequency scaling to 100%
  digitalWrite(s1, HIGH);

  
  delay(2000);

}

void readEncoder1() {
  if (digitalRead(encB1) == 0)
    pulse1--;
  else
    pulse1++;
}

void readEncoder2() {
  if (digitalRead(encB2) == 0)
    pulse2--;
  else
    pulse2++;
}

void readEncoder3() {
  if (digitalRead(encB3) == 0)
    pulse3--;
  else
    pulse3++;
}

void readEncoder4() {
  if (digitalRead(encB4) == 0)
    pulse4--;
  else
    pulse4++;
}

void total_jarak() {
  float jarakroda1 = kel_roda * (float(pulse1) / pulse_per_rotation);
  float jarakroda2 = kel_roda * (float(pulse2) / pulse_per_rotation);
  jarakroda3 = kel_roda * (float(pulse3) / pulse_per_rotation);
  jarakroda4 = kel_roda * (float(pulse4) / pulse_per_rotation);

  x_enc = 0.5*((-jarakroda1 * 0.7071) + (-jarakroda2 * 0.7071) +(jarakroda3 * 0.7071 )+ (jarakroda4 * 0.7071 ));
  y_enc = 0.5*((jarakroda1 * 0.7071) + (-jarakroda2 * 0.7071) +(-jarakroda3 * 0.7071 )+ (jarakroda4 * 0.7071 ));
}

void moveRobot(float V_x, float V_y, float omega) {
  float V1 = -0.707 * V_x + 0.707 * V_y+ omega ;
  float V2 = -0.707 * V_x - 0.707 * V_y+ omega;
  float V3 = 0.707 * V_x - 0.707 * V_y+ omega;
  float V4 = 0.707 * V_x + 0.707 * V_y+ omega;

  controlMotor(motor1Pin1, motor1Pin2, motor1pwm, V1);
  controlMotor(motor2Pin1, motor2Pin2, motor2pwm, V2);
  controlMotor(motor3Pin1, motor3Pin2, motor3pwm, V3);
  controlMotor(motor4Pin1, motor4Pin2, motor4pwm, V4);
}

void controlMotor(int pinA, int pinB, int pinPWM, float speed) {
  int pwmValue = int(abs(speed)); // Ambil nilai absolut dari kecepatan
  pwmValue = max(100, min(pwmValue, 0)); // Batasi nilai PWM antara 0 dan 255
  analogWrite(pinPWM, pwmValue); // Atur PWM untuk kontrol kecepatan

  if (speed > 0) {
    digitalWrite(pinA, LOW); // Putar motor ke depan
    digitalWrite(pinB, HIGH);
  } else if (speed < 0) {
    digitalWrite(pinA, HIGH); // Putar motor ke belakang
    digitalWrite(pinB, LOW);
  } else {
    digitalWrite(pinA, LOW); // Berhenti
    digitalWrite(pinB, LOW);
  }
}

long duration;
long distance;

// Function to get distance from ultrasonic sensor
long getDistance() {
  // Send trigger signal
  pinMode(ULTRASONIC_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_PIN, LOW);

  // Set pin to input to receive echo
  pinMode(ULTRASONIC_PIN, INPUT);
  duration = pulseIn(ULTRASONIC_PIN, HIGH);

  // Calculate distance in centimeters
  distance = duration * 0.034 / 2;

  return distance;
}

String getColor() {
  int Red, Blue, Green;

  // Measure Red
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  Red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  delay(20);

  // Measure Blue
  digitalWrite(s3, HIGH);
  Blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  delay(20);

  // Measure Green
  digitalWrite(s2, HIGH);
  Green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  delay(20);

  // Determine color
  if (Red <= 25 && Green <= 25 && Blue <= 25) {
    return "White";
  } else if (Red < Blue && Red <= Green && Red < 84) {
    return "Red";
  } else if (Blue < Green && Blue < Red && Blue < 80) {
    return "Blue";
  } else if (Green < Red && Green - Blue <= 8) {
    return "Green";
  } else {
    return "Unknown";
  }
}

void motor1roller() {
  analogWrite(motorroller1pwm, 255);
  digitalWrite(motorroller1Pin1, LOW);
  digitalWrite(motorroller1Pin2, HIGH);
}

void motor1rollers() {
  analogWrite(motorroller1pwm, 100);
  digitalWrite(motorroller1Pin1, LOW);
  digitalWrite(motorroller1Pin2, HIGH);

}

void motor2roller() {
  analogWrite(motorroller2pwm, 200);
  digitalWrite(motorroller2Pin1, LOW);
  digitalWrite(motorroller2Pin2, HIGH);
}

void motor1rollerreverse() {
  analogWrite(motorroller1pwm, 200);
  digitalWrite(motorroller1Pin1, HIGH);
  digitalWrite(motorroller1Pin2, LOW);
}

void motorrollerstop() {
  analogWrite(motorroller1pwm, 0);
  digitalWrite(motorroller1Pin1, LOW);
  digitalWrite(motorroller1Pin2, LOW);

  analogWrite(motorroller2pwm, 0);
  digitalWrite(motorroller2Pin1, LOW);
  digitalWrite(motorroller2Pin2, LOW);
}

void objectfollowing(){
  long distance = getDistance();
  if (Serial.available() > 0) {
  String data = Serial.readStringUntil('\n');
  int commaIndex = data.indexOf(',');
  if (commaIndex != -1) {  // Pastikan ada koma dalam data yang diterima
    float Vx = data.substring(0, commaIndex).toFloat();
    float Vy = data.substring(commaIndex + 1).toFloat();
    float omega = data.substring(commaIndex + 1).toFloat();
    moveRobot(Vx, Vy,omega);
    if (distance < 10){
      moveRobot(0,0,0);
      filterwarna();
    }
  } else {
    moveRobot(0,0,0);  // Tampilkan pesan jika format data tidak sesuai
  }
}
}

void filterwarna(){
  long distance = getDistance();
  if (distance < 10) {
    
    motorrollerstop();
    delay(2000);
    String color = getColor();
    Serial.println("Color: " + color);

    if (color == "Red") {
      motorrollerstop();
      moveRobot(0,0,45);
      delay(1000);
      moveRobot(0,0,0);
    }  else {
      motor2roller();
    }
  } else {
    motor2roller();
  }
  delay(500);
}


void loop() {
  // put your main code here, to run repeatedly:
  

}

