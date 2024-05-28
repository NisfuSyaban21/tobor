#include <Arduino.h>

const int motor1Pin1 = 5; // Pin motor 1
const int motor1Pin2 = 4;
const int motor2Pin1 = 11; // Pin motor 2
const int motor2Pin2 = 10;
const int motor3Pin1 = 9; // Pin motor 3
const int motor3Pin2 = 8;
const int motor4Pin1 = 7; // Pin motor 4
const int motor4Pin2 = 6;

const int encA1 = 20;
const int encB1 = 53;
const int encA2 = 21;
const int encB2 = 52;

const float kel_roda = 37.68;
const float pulse_per_rotation = 135; // Motor1

int pulse1 = 0;
int pulse2 = 0;
float x_enc = 0;
float y_enc = 0;

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

float target_points[4][2] = {
  {30, 30}, // Langkah 1
  {-30, 0}, // Langkah 2
  {0, -30}, // Langkah 3
  {30, 0}   // Langkah 4
};

int current_step = 0; // Langkah saat ini

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);

  pinMode(encA1, INPUT_PULLUP);
  pinMode(encB1, INPUT_PULLUP);
  pinMode(encA2, INPUT_PULLUP);
  pinMode(encB2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encA1), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2), readEncoder2, RISING);

  Serial.begin(9600);
  delay(2000);
}

void readEncoder1() {
  if (digitalRead(encB1) == 0)
    pulse1++;
  else
    pulse1--;
}

void readEncoder2() {
  if (digitalRead(encB2) == 0)
    pulse2++;
  else
    pulse2--;
}

void total_jarak() {
  float jarakroda1 = kel_roda * (pulse1 / pulse_per_rotation);
  float jarakroda2 = kel_roda * (pulse2 / pulse_per_rotation);

  x_enc = (-jarakroda1 * 0.7071) + (-jarakroda2 * 0.7071);
  y_enc = (jarakroda1 * 0.7071) + (-jarakroda2 * 0.7071);
}

void loop() {
  float target_x = target_points[current_step][0];
  float target_y = target_points[current_step][1];

  total_jarak();

  error_x = target_x - x_enc;
  error_y = target_y - y_enc;

  float output_x = calculatePID(Kp, Ki, Kd, error_x, last_error_x, integral_x);
  float output_y = calculatePID(Kp, Ki, Kd, error_y, last_error_y, integral_y);

  moveRobot(output_x, output_y);

  last_error_x = error_x;
  last_error_y = error_y;

 /* Serial.print("X Target: ");
  Serial.print(target_x);
  Serial.print(" Y Target: ");
  Serial.print(target_y);
  Serial.print(" X Current: ");
  Serial.print(x_enc);
  Serial.print(" Y Current: ");
  Serial.print(y_enc);
  Serial.print(" X Error: ");
  Serial.print(error_x);
  Serial.print(" Y Error: ");
  Serial.println(error_y);
*/
  Serial.println(y_enc);
  //delay(100);
    // Pindah ke langkah berikutnya
  current_step++;
  if (current_step >= 4) {
    current_step = 0; // Kembali ke langkah pertama setelah mencapai langkah terakhir
  }
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

  controlMotor(motor1Pin1, motor1Pin2, V1);
  controlMotor(motor2Pin1, motor2Pin2, V2);
  controlMotor(motor3Pin1, motor3Pin2, V3);
  controlMotor(motor4Pin1, motor4Pin2, V4);
}

void controlMotor(int pin1, int pin2, float speed) {
  int pwmValue = int(speed); // Scale speed to PWM value
  pwmValue = min(200, max(pwmValue, -200)); // Limit PWM value
  if (pwmValue > 0) {
    analogWrite(pin1, 0);
    analogWrite(pin2, pwmValue);
  } else if (pwmValue < 0) {
    analogWrite(pin1, -pwmValue);
    analogWrite(pin2, 0);
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}
