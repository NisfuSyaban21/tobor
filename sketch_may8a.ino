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

// PID constants
float Kp = 20;
float Ki = 0.002;
float Kd = 0.05;

float error_x = 0;
float error_y = 0;
float last_error_x = 0;
float last_error_y = 0;
float integral_x = 0;
float integral_y = 0;

void setup() {
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

  Serial.begin(9600);
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

void loop() {
  float target_x = 0; // Target posisi x
  float target_y = 1000; // Target posisi y

  total_jarak();

  error_x = target_x - x_enc;
  error_y = target_y - y_enc;

  float output_x = calculatePID(Kp, Ki, Kd, error_x, last_error_x, integral_x);
  float output_y = calculatePID(Kp, Ki, Kd, error_y, last_error_y, integral_y);

  moveRobot(output_x, output_y);

  last_error_x = error_x;
  last_error_y = error_y;

  Serial.print(x_enc);
  Serial.print("||");
  Serial.println(y_enc);
  //delay(100);
}

float calculatePID(float Kp, float Ki, float Kd, float error, float last_error, float& integral) {
  float proportional = Kp * error;
  integral += error;
  float derivative = Kd * (error - last_error);
  float output = proportional + Ki * integral + derivative;
  return output;
}


void moveRobot(float V_x, float V_y) {

  float V1 = -0.707 * V_x + 0.707 * V_y ;
  float V2 = -0.707 * V_x - 0.707 * V_y;
  float V3 = 0.707 * V_x - 0.707 * V_y ;
  float V4 = 0.707 * V_x + 0.707 * V_y;

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
