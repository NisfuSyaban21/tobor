// Deklarasi pin motor dan encoder tetap seperti sebelumnya
const int motor1Pin1 = 31;  // Pin motor 1
const int motor1Pin2 = 33;
const int motor1pwm = 8;

const int motor2Pin1 = 24;  // Pin motor 2
const int motor2Pin2 = 25;
const int motor2pwm = 5;

const int motor3Pin1 = 26;  // Pin motor 3
const int motor3Pin2 = 27;
const int motor3pwm = 6;

const int motor4Pin1 = 28;  // Pin motor 4
const int motor4Pin2 = 29;
const int motor4pwm = 7;

// Definisi pin encoder
const int encA2 = 21;
const int encB2 = 42;

const int encA3 = 18;
const int encB3 = 34;
const int encA4 = 19;
const int encB4 = 36;

const int encA1 = 20;
const int encB1 = 40;

const float pulse_per_rotation = 134.4; // Pulse per rotasi motor
const float max_rpm = 100.0;  // Maksimum RPM motor
volatile long pulse1 = 0;
unsigned long lastTime = 0;

// Nilai RPM yang diinginkan untuk motor 1
float desiredRPM1 = 5.0; 

// Variabel PID
float Kp = 1.2;  // Koefisien proportional
float Ki = 0.7;  // Koefisien integral
float Kd = 0.3;  // Koefisien derivative

float previousError1 = 0;
float integral1 = 0;

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor1pwm, OUTPUT);

  pinMode(encA1, INPUT_PULLUP);
  pinMode(encB1, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encA1), readEncoder1, RISING);

  float integral1 = 0;

  lastTime = millis();
  Serial.begin(9600);
}

void loop() {
  // Setiap interval waktu, hitung RPM motor dari encoder
  if (millis() - lastTime >= 100) {  // Setiap 100 ms
    float actualRPM1 = calculateRPM(pulse1);

    // Tampilkan RPM yang dihitung dari encoder untuk Motor 1 di Serial Plotter
    Serial.print(desiredRPM1);  // Tampilkan nilai RPM yang diinginkan
    Serial.print(" ");          // Pisahkan nilai dengan spasi
    Serial.println(actualRPM1);  // Tampilkan nilai RPM aktual, lalu baris baru

    // Reset pulse counter setiap 100 ms
    pulse1 = 0;
    lastTime = millis();

    // Gunakan PID untuk mengontrol kecepatan motor
    moveMotorByPID(desiredRPM1, actualRPM1);
  }
}

// Fungsi untuk membaca encoder motor 1
void readEncoder1() {
  if (digitalRead(encB1) == 0) {
    pulse1--;
  } else {
    pulse1++;
  }
}

// Fungsi untuk menghitung RPM dari pulse encoder
float calculateRPM(long pulseCount) {
  // Rumus untuk menghitung RPM: (pulseCount / pulse_per_rotation) * 60 detik
  return (pulseCount / pulse_per_rotation) * 60;
}

// Fungsi untuk mengontrol motor menggunakan PID
void moveMotorByPID(float desiredRPM, float actualRPM) {
  float error1 = desiredRPM - actualRPM; // Hitung error
  integral1 += error1;  // Hitung bagian integral
  float derivative1 = error1 - previousError1;  // Hitung bagian derivative
  previousError1 = error1;  // Simpan error sebelumnya untuk perhitungan berikutnya

  // Rumus PID: Kp * error + Ki * integral + Kd * derivative
  float pidOutput1 = (Kp * error1) + (Ki * integral1) + (Kd * derivative1);

  // Konversikan nilai PID menjadi nilai PWM (batasi nilai antara 0 dan 255)
  int pwm1 = constrain(pidOutput1, 0, 50);


  // Kontrol motor dengan PWM yang dihitung dari PID
  controlMotor(motor1Pin1, motor1Pin2, motor1pwm, pwm1);
}

// Fungsi untuk mengontrol motor menggunakan PWM
void controlMotor(int pinA, int pinB, int pinPWM, int pwmValue) {
  analogWrite(pinPWM, pwmValue); // Atur PWM untuk kontrol kecepatan

  if (pwmValue > 0) {
    digitalWrite(pinA, LOW); // Putar motor ke depan
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW); // Berhenti
    digitalWrite(pinB, LOW);
  }
}
