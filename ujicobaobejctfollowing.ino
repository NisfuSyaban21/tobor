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

void setup() {
  // put your setup code here, to run once:
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
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {  // Pastikan ada koma dalam data yang diterima
      float x_center = data.substring(0, commaIndex).toFloat();
      float y_center = data.substring(commaIndex + 1).toFloat();
      
      if (x_center <= 100  && y_center < 370) {
        moveRobot(0, 0, 30);  // Gerak maju jika objek berada di sebelah kiri
      } else if (x_center >= 500 && y_center < 370) {
        moveRobot(0, 0, -30);  // Gerak mundur jika objek berada di sebelah kanan
      } else if (x_center > 100 && x_center <= 320 && y_center < 370) {
        moveRobot(-30, 30, 0);  // Putar ke kiri jika objek berada di depan dan ke kiri
      } else if (x_center > 320 && x_center < 500 && y_center < 370) {
        moveRobot(30, 30, 0);  // Putar ke kanan jika objek berada di depan dan ke kanan
      } else if (y_center > 370 && y_center <= 480 && x_center >= 300 && x_center <= 320) {
        moveRobot(0, 0, 0);  // Berhenti jika objek berada di atas dan di tengah
      }
    }
  }
}



void moveRobot(float V_x, float V_y, float omega) {

  float V1 = -0.707 * V_x + 0.707 * V_y + omega;
  float V2 = -0.707 * V_x - 0.707 * V_y + omega;
  float V3 = 0.707 * V_x - 0.707 * V_y + omega;
  float V4 = 0.707 * V_x + 0.707 * V_y + omega;

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
