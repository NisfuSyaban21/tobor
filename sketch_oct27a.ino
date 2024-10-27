#include <Arduino.h>
#define S0 54
#define S1 55
#define S2 56
#define S3 57
#define sensorOut 58

// Definisikan pin untuk sensor ultrasonik 1 (Depan)
const int trigPin1 = 59;
const int echoPin1 = 60;

// Definisikan pin untuk sensor ultrasonik 2 (Kiri)
const int trigPin2 = 61;
const int echoPin2 = 62;

// Definisikan pin untuk sensor ultrasonik 3 (Kanan)
const int trigPin3 = 63;
const int echoPin3 = 64;

const int trigPin4 = 65;
const int echoPin4 = 66;

const int trigPin5 = 67;
const int echoPin5 = 68;

const int trigPin6 = 47;
const int echoPin6 = 49;
// Definisi pin motor
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

//definisi pin motor roller
const int motorroller1Pin1 = 43;  // Pin motor 1
const int motorroller1Pin2 = 45;
const int motorroller1pwm = 13;

const int motorroller2Pin1 = 39;  // Pin motor 2
const int motorroller2Pin2 = 41;
const int motorroller2pwm = 12;

// Definisi pin encoder
const int encA2 = 21;
const int encB2 = 42;
const int encA1 = 20;
const int encB1 = 40;
const int encA3 = 18;
const int encB3 = 34;
const int encA4 = 19;
const int encB4 = 36;

bool ballremoved = false;
int a;
int b;
int siklus_home;
int siklus_homing;
float output_w;

bool ishoming = false;

int interval = 2000;
unsigned long time_now = 0;

const float kel_roda = 37.68;
const float pulse_per_rotation = 135; // Motor1

long pulse1 = 0;
long pulse2 = 0;
long pulse3 = 0;
long pulse4 = 0;
float x_enc = 0;
float y_enc = 0;
float w_enc = 0;


float jarakroda3;
float jarakroda4;


//PID TUNNING
// PID constants
// Variabel PID untuk setiap motor
float Kp = 1.2;  // Koefisien proportional
float Ki = 0.3;  // Koefisien integral
float Kd = 0.02;  // Koefisien derivative
float previousError1 = 0, previousError2 = 0, previousError3 = 0, previousError4 = 0;
float integral1 = 0, integral2 = 0, integral3 = 0, integral4 = 0;


long distance1, distance2, distance3,distance4;

bool isGerakkankeRunning = false;
String data1_str = "";

int done_home;

// Definisikan batas maksimum dan minimum RPM
float maxRPM = 15.0;  // RPM maksimum yang diizinkan
float minRPM = 10.0;  // RPM minimum yang diizinkan

unsigned long lastTime = 0;

const float K_p = 0.01; // Konstanta proporsional untuk kecepatan
#define PI 3.14

#define Button_pin A8//tombol reset pergerakan

int sens_red(){
  int redFrequency;

  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);

  return redFrequency;
}

int sens_green(){
  int greenFrequency;

  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);

  return greenFrequency;
}

int sens_blue(){
  int blueFrequency;

  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);

  return blueFrequency;
}



void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Set scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);



  // Inisialisasi pin sebagai output/input
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);

  pinMode(trigPin5, OUTPUT);
  pinMode(echoPin5, INPUT);

  pinMode(trigPin6, OUTPUT);
  pinMode(echoPin6, INPUT);
  
  Serial.begin(115200);

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

  //motor roller
  pinMode(motorroller1Pin1, OUTPUT);
  pinMode(motorroller1Pin2, OUTPUT);
  pinMode(motorroller2Pin1, OUTPUT);
  pinMode(motorroller2Pin2, OUTPUT);

  pinMode(motorroller1pwm, OUTPUT);
  pinMode(motorroller2pwm, OUTPUT);
  
  //encoder
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
  
  delay(2000);
  //siklus_home = 0;
  //done_home = homing();
  //stepsilo1();
  //siklus_home = 2;
      

}


// Fungsi untuk membaca jarak dari sensor ultrasonik
long readUltrasonic(int trigPin, int echoPin) {
  // Kirim sinyal trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Hitung durasi echo
  long duration = pulseIn(echoPin, HIGH);

  // Konversi durasi menjadi jarak (cm)
  long   distance = duration * 0.034 / 2;

  return distance;
}


void readEncoder1() {
  if (digitalRead(encB1) == 0) {
    pulse1--;
  }
  else {
    pulse1++;
  }
}

void readEncoder2() {
  if (digitalRead(encB2) == 0){
    pulse2--;
  }
  else{
    pulse2++;
  }
}

void readEncoder3() {
  if (digitalRead(encB3) == 0){
    pulse3--;
  }
  else{
    pulse3++;
  }

}

void readEncoder4() {
  if (digitalRead(encB4) == 0){
    pulse4--;
  }
  else{
    pulse4++;
  }
}

// Fungsi untuk menghitung RPM dari jumlah pulse
float calculateRPM(long pulseCount) {
  return (pulseCount / pulse_per_rotation) * 60;  // Pulse per rotasi -> RPM
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

  // Kontrol setiap motor menggunakan PID untuk kecepatan, dan omega untuk arah
  moveMotorByPID(rpm1, calculateRPM(pulse1), motor1Pin1, motor1Pin2, motor1pwm, &previousError1, &integral1, omega1);
  moveMotorByPID(rpm2, calculateRPM(pulse2), motor2Pin1, motor2Pin2, motor2pwm, &previousError2, &integral2, omega2);
  moveMotorByPID(rpm3, calculateRPM(pulse3), motor3Pin1, motor3Pin2, motor3pwm, &previousError3, &integral3, omega3);
  moveMotorByPID(rpm4, calculateRPM(pulse4), motor4Pin1, motor4Pin2, motor4pwm, &previousError4, &integral4, omega4);
}


void total_jarak() {
  float jarakroda1 = kel_roda * (float(pulse1) / pulse_per_rotation);
  float jarakroda2 = kel_roda * (float(pulse2) / pulse_per_rotation);
  jarakroda3 = kel_roda * (float(pulse3) / pulse_per_rotation);
  jarakroda4 = kel_roda * (float(pulse4) / pulse_per_rotation);

  x_enc = 0.5*((-jarakroda1 * 0.7071) + (-jarakroda2 * 0.7071) +(jarakroda3 * 0.7071 )+ (jarakroda4 * 0.7071 ));
  y_enc = 0.5*((jarakroda1 * 0.7071) + (-jarakroda2 * 0.7071) +(-jarakroda3 * 0.7071 )+ (jarakroda4 * 0.7071 ));
  w_enc = ((pulse1+pulse2+pulse3+pulse4)/4)*90/200;

}



float calculatePID(float Kp, float Ki, float Kd, float error, float last_error, float& integral) {
  float proportional = Kp * error;
  integral += error;
  float derivative = Kd * (error - last_error);
  float output = proportional + ( Ki * integral ) + derivative;
  return output;
}






String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}




void loop() {



  
  int rd = sens_red();
  int gr = sens_green();
  int bl = sens_blue();

  long distance1 = readUltrasonic(trigPin1, echoPin1);  // kiri
  long distance2 = readUltrasonic(trigPin2, echoPin2);  // Kanan
  long distance6 = readUltrasonic(trigPin6, echoPin6);  // samping
  //Serial.print(distance1);Serial.print("|");Serial.print(distance2);Serial.print("|");Serial.print(distance3);Serial.print("|");Serial.println(distance4);
  if (done_home == 0){
  if (!isGerakkankeRunning && a==0 && b==0) { //tambahin kondisi sensor warna dan ultra
    if(distance6 <= 10){
      if(rd < 500 && bl > 500){
        Serial.println("readyred");
      }
      else if (rd > 500 && bl < 500){
        Serial.println("readyblue");
      }
     else {
    Serial.println("unready");
    }
      
    }
    else {
    Serial.println("unready");
    }
    delay(50);
    }
  } 

  
  if (Serial.available() > 0) { // Cek apakah ada data yang tersedia di Serial
    String receivedData = Serial.readStringUntil('\n'); // Menerima data hingga newline ('\n')
    data1_str = receivedData;
    //Serial.print(data1_str);  Serial.print(" | ");  Serial.println(isGerakkankeRunning);
    
    
    if (data1_str == "1" && !isGerakkankeRunning) {
      if (b == 0) {
        isGerakkankeRunning = true;
        siklus_home = 0;
        ballremoved = false;
        stepsilo1();
      //  Serial.println("1");
        a=0; b=0;
      }
    } else if (data1_str == "2" && !isGerakkankeRunning) {
      if (b == 0) {
        isGerakkankeRunning = true;
        siklus_home = 0;
        ballremoved = false;
        stepsilo2();
        a=0; b=0;
      }
    }

    else if (data1_str == "3" && !isGerakkankeRunning) {
      if (b == 0) {
        isGerakkankeRunning = true;
        siklus_home = 0;
        ballremoved = false;
        //stepsilo3();
        a=0; b=0;
      }
    }

    else if (data1_str == "4" && !isGerakkankeRunning) {
      if (b == 0) {
        isGerakkankeRunning = true;
        siklus_home = 0;
        ballremoved = false;
        //stepsilo4();
        a=0; b=0;
      }
    }

    else if (data1_str == "5" && !isGerakkankeRunning) {
      if (b == 0) {
        isGerakkankeRunning = true;
        siklus_home = 0;
        ballremoved = false;
        //stepsilo5();
        a=0; b=0;
      }
    }

  if (!data1_str) Serial.end();
  }
  //Serial.println(siklus_home);
}


void motor1rollers() {
  analogWrite(motorroller1pwm, 170);
  digitalWrite(motorroller1Pin1, HIGH);
  digitalWrite(motorroller1Pin2, LOW);

}

void motor2rollers() {
  analogWrite(motorroller2pwm, 170);
  digitalWrite(motorroller2Pin1, LOW);
  digitalWrite(motorroller2Pin2, HIGH);
}

void motorrollerstop() {
  analogWrite(motorroller1pwm, 0);
  digitalWrite(motorroller1Pin1, LOW);
  digitalWrite(motorroller1Pin2, LOW);

  analogWrite(motorroller2pwm, 0);
  digitalWrite(motorroller2Pin1, LOW);
  digitalWrite(motorroller2Pin2, LOW);
}

int stepsilo1() {
  while (siklus_home == 0) {  // Loop terus menerus hingga kondisi berhenti terpenuhi
  

    // Reset pulse counter setiap 100 ms untuk menghitung RPM feedback dari encoder
    if (millis() - lastTime >= 100) {
      // Hitung RPM aktual dari setiap motor berdasarkan encoder
      float actualRPM1 = calculateRPM(pulse1);
      float actualRPM2 = calculateRPM(pulse2);
      float actualRPM3 = calculateRPM(pulse3);
      float actualRPM4 = calculateRPM(pulse4);
      
      Serial.print("omega 1:");
      Serial.println(actualRPM1);



      // Membaca sensor ultrasonik untuk jarak
      long distance1 = readUltrasonic(trigPin1, echoPin1);  // kiri
      long distance2 = readUltrasonic(trigPin2, echoPin2);  // kanan
      long distance3 = readUltrasonic(trigPin3, echoPin3);  // samping
      long distance4 = readUltrasonic(trigPin4, echoPin4);

      Serial.print("distance:");
      Serial.println(distance1);
      
      // Setpoint atau target
      long spy = 20;
      long spx = 10;

      // Error untuk masing-masing arah
      long erhomx = (distance3 - spx) * -1;
      long erhomy_1 = distance1 - spy;

      // Konversi error menjadi kecepatan linear yang diinginkan
      float V_x = K_p * erhomx;
      float V_y = K_p * erhomy_1;

      // Jalankan `moveRobot` dengan kecepatan yang diinginkan
      moveRobot(V_x, V_y, 0);  // W diatur ke 0 karena tidak ada rotasi

      // Reset pulse counter dan waktu untuk interval berikutnya
      pulse1 = 0;
      pulse2 = 0;
      pulse3 = 0;
      pulse4 = 0;
      lastTime = millis();

      // Kondisi untuk berhenti
      if (erhomy_1 >= -1 && erhomy_1 <= 1) {  // Ketika berada dalam jarak target
        // Hentikan robot dengan memanggil `moveRobot` dengan semua kecepatan 0
        moveRobot(0, 0, 0);
        siklus_home = 0;
        isGerakkankeRunning = false;
        break;  // Keluar dari loop setelah mencapai target
      }
    }



    //delay(100);  // Tambahkan delay untuk stabilisasi
  }

  //return 1;  // Nilai kembalian untuk menandakan langkah selesai
}

int stepsilo2() {
  while (siklus_home == 0) {  // Loop terus menerus hingga kondisi berhenti terpenuhi
  

    // Reset pulse counter setiap 100 ms untuk menghitung RPM feedback dari encoder
    if (millis() - lastTime >= 100) {
      // Hitung RPM aktual dari setiap motor berdasarkan encoder
      float actualRPM1 = calculateRPM(pulse1);
      float actualRPM2 = calculateRPM(pulse2);
      float actualRPM3 = calculateRPM(pulse3);
      float actualRPM4 = calculateRPM(pulse4);
      
      Serial.print("omega 1:");
      Serial.println(actualRPM1);



      // Membaca sensor ultrasonik untuk jarak
      long distance1 = readUltrasonic(trigPin1, echoPin1);  // kiri
      long distance2 = readUltrasonic(trigPin2, echoPin2);  // kanan
      long distance3 = readUltrasonic(trigPin3, echoPin3);  // samping
      long distance4 = readUltrasonic(trigPin4, echoPin4);

      Serial.print("distance:");
      Serial.println(distance1);
      
      // Setpoint atau target
      long spy = 20;
      long spx = 10;

      // Error untuk masing-masing arah
      long erhomx = (distance3 - spx) * -1;
      long erhomy_1 = distance1 - spy;
      long errormega = (distance1 - distance2);

      // Konversi error menjadi kecepatan linear yang diinginkan
      float V_x = K_p * erhomx;
      float V_y = K_p * erhomy_1;
      float V_w = K_p * errormega;

      // Jalankan `moveRobot` dengan kecepatan yang diinginkan
      moveRobot(0, 0, V_w);  // W diatur ke 0 karena tidak ada rotasi

      // Reset pulse counter dan waktu untuk interval berikutnya
      pulse1 = 0;
      pulse2 = 0;
      pulse3 = 0;
      pulse4 = 0;
      lastTime = millis();

      // Kondisi untuk berhenti
      if (errormega >= -1 && errormega <= 1) {  // Ketika berada dalam jarak target
        // Hentikan robot dengan memanggil `moveRobot` dengan semua kecepatan 0
        moveRobot(0, 0, 0);
        siklus_home = 0;
        isGerakkankeRunning = false;
        break;  // Keluar dari loop setelah mencapai target
      }
    }



    //delay(100);  // Tambahkan delay untuk stabilisasi
  }

  //return 1;  // Nilai kembalian untuk menandakan langkah selesai
}






