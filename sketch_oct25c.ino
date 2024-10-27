// Pin motor
const int motor1Pin1 = 31;  
const int motor1Pin2 = 33;
const int motor1pwm = 8;

const int motor2Pin1 = 24;  
const int motor2Pin2 = 25;
const int motor2pwm = 5;

const int motor3Pin1 = 26;  
const int motor3Pin2 = 27;
const int motor3pwm = 6;

const int motor4Pin1 = 28;  
const int motor4Pin2 = 29;
const int motor4pwm = 7;

// Pin sensor ultrasonik
const int trigPinFront = 59;   // Sensor ultrasonik depan (untuk y-axis)
const int echoPinFront = 60;

const int trigPinSide = 63;   // Sensor ultrasonik samping (untuk x-axis)
const int echoPinSide = 64;

// Definisi pin encoder
const int encA2 = 21;
const int encB2 = 42;
const int encA1 = 20;
const int encB1 = 40;
const int encA3 = 18;
const int encB3 = 34;
const int encA4 = 19;
const int encB4 = 36;

// Konstanta robot
const float r = 6.05; // Jari-jari roda (cm)
const float l = 37.5; // Jarak dari pusat robot ke roda (cm)
const float K_p = 0.01; // Konstanta proporsional untuk kecepatan
const float pulse_per_rotation = 134;  // Pulse per rotasi motor

// Variabel PID untuk setiap motor
float Kp = 0.9;  // Koefisien proportional
float Ki = 0.3;  // Koefisien integral
float Kd = 0.02;  // Koefisien derivative
float previousError1 = 0, previousError2 = 0, previousError3 = 0, previousError4 = 0;
float integral1 = 0, integral2 = 0, integral3 = 0, integral4 = 0;

// Variabel pulse encoder
volatile long pulse1 = 0, pulse2 = 0, pulse3 = 0, pulse4 = 0;
unsigned long lastTime = 0;
unsigned long lastUltrasonicTime = 0;

#define PI 3.14

// Definisikan batas maksimum dan minimum RPM
float maxRPM = 15.0;  // RPM maksimum yang diizinkan
float minRPM = 8.0;  // RPM minimum yang diizinkan

void setup() {
  // Setup pin motor
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

  // Setup pin sensor ultrasonik
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinSide, OUTPUT);
  pinMode(echoPinSide, INPUT);

  // Setup pin encoder
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

  lastTime = millis();
  Serial.begin(9600);  // Untuk debugging
}

// Fungsi untuk membaca pulse encoder motor 1
void readEncoder1() {
  if (digitalRead(encB1) == 0) {
    pulse1--;  // Putar mundur
  } else {
    pulse1++;  // Putar maju
  }
}

// Fungsi untuk membaca pulse encoder motor 2
void readEncoder2() {
  if (digitalRead(encB2) == 0) {
    pulse2--;
  } else {
    pulse2++;
  }
}

// Fungsi untuk membaca pulse encoder motor 3
void readEncoder3() {
  if (digitalRead(encB3) == 0) {
    pulse3--;
  } else {
    pulse3++;
  }
}

// Fungsi untuk membaca pulse encoder motor 4
void readEncoder4() {
  if (digitalRead(encB4) == 0) {
    pulse4--;
  } else {
    pulse4++;
  }
}

// Fungsi untuk menghitung RPM dari jumlah pulse
float calculateRPM(long pulseCount) {
  return (pulseCount / pulse_per_rotation) * 60;  // Pulse per rotasi -> RPM
}

// Fungsi untuk membaca sensor ultrasonik
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);  // Durasi pulsa dalam microseconds
  float distance = duration * 0.034 / 2;  // Jarak dalam cm
  return distance;
}

// Fungsi untuk mengonversi rad/s ke RPM
float radToRPM(float omega) {
  return (omega * 60) / (2 * PI);
}

// Fungsi untuk mengontrol motor menggunakan PID dan arah motor berdasarkan omega
void moveMotorByPID(float desiredRPM, float actualRPM, int pinA, int pinB, int pinPWM, float* previousError, float* integral, float omega) {
  
  float error = abs(desiredRPM) - abs(actualRPM);  // Error hanya berdasarkan desiredRPM (kecepatan)
  *integral += error;  // Hitung bagian integral
  float derivative = error - *previousError;  // Hitung bagian derivative
  *previousError = error;  // Simpan error sebelumnya untuk perhitungan berikutnya

  // Rumus PID: Kp * error + Ki * integral + Kd * derivative
  float pidOutput = (Kp * error) + (Ki * *integral) + (Kd * derivative);

  // Konversikan nilai PID menjadi nilai PWM (batasi nilai antara 0 dan 255)
  int pwm = abs(pidOutput);  // Gunakan nilai absolut dari output PID untuk PWM
  pwm = constrain(pwm, 0, 50);  // Batasi nilai PWM antara 0 dan 255

  // Kontrol arah dan kecepatan motor berdasarkan tanda dari omega (positif atau negatif)
  controlMotor(pinA, pinB, pinPWM, pwm, omega);
}

// Fungsi untuk mengontrol arah dan kecepatan motor berdasarkan output PID dan tanda omega
void controlMotor(int pinA, int pinB, int pinPWM, float pwmValue, float omega) {

  // Jika omega positif, motor berputar searah jarum jam (CW)
  if (omega > 0) {
    digitalWrite(pinA, LOW);  // Arahkan motor untuk berputar CW
    digitalWrite(pinB, HIGH);
  } 
  // Jika omega negatif, motor berputar berlawanan arah jarum jam (CCW)
  else if (omega < 0) {
    digitalWrite(pinA, HIGH);  // Arahkan motor untuk berputar CCW
    digitalWrite(pinB, LOW);
  } 
  // Jika omega nol, motor berhenti
  else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }

  // Kirimkan nilai PWM ke motor
  analogWrite(pinPWM, pwmValue);
}

// Fungsi untuk normalisasi nilai RPM
float normalizeRPM(float rpm) {
  if (rpm > maxRPM) {
    return maxRPM;  // Jika RPM melebihi batas maksimum, kembalikan nilai maksimum
  } else if (rpm < minRPM) {
    return minRPM;  // Jika RPM kurang dari batas minimum, kembalikan nilai minimum
  }
  return rpm;  // Jika dalam rentang, kembalikan nilai RPM apa adanya
}

// Fungsi untuk mengontrol semua motor dengan menggunakan inverse kinematics dan PID control
void moveRobot(float V_x, float V_y, float W) {

  // Hitung omega untuk arah berdasarkan kinematika invers
  float omega1 = (-0.707 * V_x) + (0.707 * V_y) + W;
  float omega2 = (-0.707 * V_x) - (0.707 * V_y) + W;
  float omega3 = (0.707 * V_x) - (0.707 * V_y) + W;
  float omega4 = (0.707 * V_x) + (0.707 * V_y) + W;

  // Konversi dari rad/s ke RPM
  float rpm1 = radToRPM(abs(omega1));  // Desired RPM hanya menggunakan nilai absolut dari omega
  float rpm2 = radToRPM(abs(omega2));
  float rpm3 = radToRPM(abs(omega3));
  float rpm4 = radToRPM(abs(omega4));

  rpm1 = normalizeRPM(rpm1);
  rpm2 = normalizeRPM(rpm2);
  rpm3 = normalizeRPM(rpm3);
  rpm4 = normalizeRPM(rpm4);

  Serial.print("rpm 1:");
  Serial.println(rpm1);

  Serial.print("rpm 2:");
  Serial.println(rpm2);

  Serial.print("rpm 3:");
  Serial.println(rpm3);

  Serial.print("rpm 4:");
  Serial.println(rpm4);

  Serial.print("omega 1:");
  Serial.println(omega1);

  Serial.print("omega 2:");
  Serial.println(omega2);

  Serial.print("omega 3:");
  Serial.println(omega3);

  Serial.print("rpm 4:");
  Serial.println(omega4);



  /*
  rpm1 = normalizeRPM(rpm1);
  rpm2 = normalizeRPM(rpm2);
  rpm3 = normalizeRPM(rpm3);
  rpm4 = normalizeRPM(rpm4);
  */



  // Kontrol setiap motor menggunakan PID untuk kecepatan, dan omega untuk arah
  moveMotorByPID(rpm1, calculateRPM(pulse1), motor1Pin1, motor1Pin2, motor1pwm, &previousError1, &integral1, omega1);
  moveMotorByPID(rpm2, calculateRPM(pulse2), motor2Pin1, motor2Pin2, motor2pwm, &previousError2, &integral2, omega2);
  moveMotorByPID(rpm3, calculateRPM(pulse3), motor3Pin1, motor3Pin2, motor3pwm, &previousError3, &integral3, omega3);
  moveMotorByPID(rpm4, calculateRPM(pulse4), motor4Pin1, motor4Pin2, motor4pwm, &previousError4, &integral4, omega4);
}

void loop() {
  // Baca sensor ultrasonik setiap 100 ms
  float jarak_x = readUltrasonic(trigPinSide, echoPinSide);    // Jarak di sumbu x
  float jarak_y = readUltrasonic(trigPinFront, echoPinFront);  // Jarak di sumbu y

  // Hitung delta jarak dari setpoint (misalnya setpoint = 10 cm)
  float delta_x = (jarak_x -10)*-1; // Set ke 0 jika tidak ada gerakan sumbu X
  float delta_y = jarak_y - 10; // Misalnya setpoint di sumbu y adalah 30 cm

  // Konversi delta jarak ke kecepatan linear (m/s)
  float V_x = K_p * delta_x;
  float V_y = K_p * delta_y;

  Serial.print("x :");
  Serial.println(jarak_x);

  Serial.print("y :");
  Serial.println(jarak_y);



  // Jalankan robot dengan kecepatan yang dihitung
  moveRobot(V_x, V_y, 0);  // Tanpa rotasi (W = 0)

  delay(1000);

  // Reset pulse counter setiap 1 detik untuk menghitung RPM feedback dari encoder
  if (millis() - lastTime >= 100) {
    pulse1 = 0;
    pulse2 = 0;
    pulse3 = 0;
    pulse4 = 0;
    lastTime = millis();
  }
}
