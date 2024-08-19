#include <Arduino.h>
#define S0 54
#define S1 55
#define S2 56
#define S3 57
#define sensorOut 58

#define ULTRASONIC_PIN1 59 //dalam
#define ULTRASONIC_PIN2 60 //kanan
#define ULTRASONIC_PIN3 61 // kiri
#define ULTRASONIC_PIN4 62 // samping

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
const int encA4 = 21;
const int encB4 = 42;
const int encA3 = 20;
const int encB3 = 40;
const int encA1 = 18;
const int encB1 = 34;
const int encA2 = 19;
const int encB2 = 36;

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

//PID RPM
// Parameter PID global
float Kp1 = 3.2, Ki1 = 2.0, Kd1 = 0.0; // Untuk motor 1
float Kp2 = 3.2, Ki2 = 2.0, Kd2 = 0.0; // Untuk motor 2
float Kp3 = 3.2, Ki3 = 2.0, Kd3 = 0.0; // Untuk motor 3
float Kp4 = 3.2, Ki4 = 2.0, Kd4 = 0.0; // Untuk motor 4

// Variabel PID untuk setiap motor
float last_error1 = 0, last_error2 = 0, last_error3 = 0, last_error4 = 0;
float integral1 = 0, integral2 = 0, integral3 = 0, integral4 = 0;
float rpm1;
float rpm2;
float rpm3;
float rpm4;


//PID TUNNING\
// PID constants
float Kp = 20;
float Ki = 0.0;
float Kd = 0.0;

float wKp = 20;
float wKi = 0.001;
float wKd = 0.0;

float error_x = 0;
float error_y = 0;
float error_w = 0;
float last_error_x = 0;
float last_error_y = 0;
float last_error_w = 0;
float integral_x = 0;
float integral_y = 0;
float integral_w = 0;

long distance1, distance2, distance3,distance4;

bool isGerakkankeRunning = false;
String data1_str = "";

int done_home;

unsigned long timelast = 0;
const float timesampling = 30.0;  // 30 ms atau 0.03 detik

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



  pinMode(ULTRASONIC_PIN1, OUTPUT);
  pinMode(ULTRASONIC_PIN2, OUTPUT);
  pinMode(ULTRASONIC_PIN3, OUTPUT);
  pinMode(ULTRASONIC_PIN4, OUTPUT);
  
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


long getDistance(int pin) {
  long duration, distance;

  // Set the pin to OUTPUT mode and send a LOW signal for 2 microseconds
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);

  // Send a HIGH signal for 10 microseconds
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);

  // Change the pin to INPUT mode to read the echo
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);

  // Calculate the distance
  distance = (duration / 2) / 29.1;

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

void total_jarak() {
  float jarakroda1 = kel_roda * (float(pulse1) / pulse_per_rotation);
  float jarakroda2 = kel_roda * (float(pulse2) / pulse_per_rotation);
  jarakroda3 = kel_roda * (float(pulse3) / pulse_per_rotation);
  jarakroda4 = kel_roda * (float(pulse4) / pulse_per_rotation);

  x_enc = 0.5*((-jarakroda1 * 0.7071) + (-jarakroda2 * 0.7071) +(jarakroda3 * 0.7071 )+ (jarakroda4 * 0.7071 ));
  y_enc = 0.5*((jarakroda1 * 0.7071) + (-jarakroda2 * 0.7071) +(-jarakroda3 * 0.7071 )+ (jarakroda4 * 0.7071 ));
  w_enc = ((pulse1+pulse2+pulse3+pulse4)/4)*90/200;

}

void rpm(){
  unsigned long timenow = millis();
  float deltatime = timenow - timelast;
  if(deltatime>=timesampling){
    float freqSig1 = pulse1/(deltatime/1.0e3);
    float freqSig2 = pulse2/(deltatime/1.0e3);
    float freqSig3 = pulse3/(deltatime/1.0e3);
    float freqSig4 = pulse4/(deltatime/1.0e3);
    pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
    float rpm1 = freqSig1 * 60.0 /134.4;
    float rpm2 = freqSig2 * 60.0 /134.4;
    float rpm3 = freqSig3 * 60.0 /134.4;
    float rpm4 = freqSig4 * 60.0 /134.4;

  }

}

void moveRobot(float V_x, float V_y, float W) {

  float V1 = (-0.707 * V_x) + (0.707 * V_y) + W;
  float V2 = (-0.707 * V_x) - (0.707 * V_y) + W;
  float V3 = (0.707 * V_x) - (0.707 * V_y) + W;
  float V4 = (0.707 * V_x) + (0.707 * V_y) + W;

  rpm();

  // Menghitung error untuk setiap motor
  float error1 = V1 - rpm1;
  float error2 = V2 - rpm2;
  float error3 = V3 - rpm3;
  float error4 = V4 - rpm4;


  // Menghitung output PID untuk setiap motor dengan parameter global
  float V1_control = calculatePID(Kp1, Ki1, Kd1, error1, last_error1, integral1);
  float V2_control = calculatePID(Kp2, Ki2, Kd2, error2, last_error2, integral2);
  float V3_control = calculatePID(Kp3, Ki3, Kd3, error3, last_error3, integral3);
  float V4_control = calculatePID(Kp4, Ki4, Kd4, error4, last_error4, integral4);


  // Mengontrol motor dengan output PID
  controlMotor(motor1Pin1, motor1Pin2, motor1pwm, V1_control);
  controlMotor(motor2Pin1, motor2Pin2, motor2pwm, V2_control);
  controlMotor(motor3Pin1, motor3Pin2, motor3pwm, V3_control);
  controlMotor(motor4Pin1, motor4Pin2, motor4pwm, V4_control);

  last_error1 = error1;
  last_error2 = error2; 
  last_error3 = error3;
  last_error4 = error4;
}

float calculatePID(float Kp, float Ki, float Kd, float error, float last_error, float& integral) {
  float proportional = Kp * error;
  integral += error;
  float derivative = Kd * (error - last_error);
  float output = proportional + ( Ki * integral ) + derivative;
  return output;
}

void controlMotor(int pinA, int pinB, int pinPWM, float speed) {
  int pwmValue = int(abs(speed)); // Ambil nilai absolut dari kecepatan
  //pwmValue = min(60, max(pwmValue, 0)); // Batasi nilai PWM antara 0 dan 255
  if (ishoming) {
    if (siklus_homing == 0) { pwmValue = min(60, max(pwmValue, 45)); }
    else if (siklus_homing == 1) { pwmValue = min(40, max(pwmValue, 10)); }
    else if (siklus_homing >= 2) { pwmValue = min(60, max(pwmValue, 0)); }
  } else {
    if (siklus_home == 0) { pwmValue = min(55, max(pwmValue, 45)); }
    else if (siklus_home == 1) { pwmValue = min(40, max(pwmValue, 10)); }
    else if (siklus_home >= 2) { pwmValue = min(60, max(pwmValue, 0)); }
  }
    //pwmValue = min(80, max(pwmValue, 0));

  
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



void gerakke(float target_x,float target_y, float target_w) {
  // Tetapkan target berdasarkan status

  total_jarak();

  error_x = target_x - x_enc;
  error_y = target_y - y_enc;
  error_w = target_w - w_enc;
  
  moveRobot(error_x, error_y, error_w);

  last_error_x = error_x;
  last_error_y = error_y;
  last_error_w = error_w;


  
}

void gerakwke(float target_w) {
  // Tetapkan target berdasarkan status

  total_jarak();

  error_w = target_w - w_enc;

  float output_w = calculatePID(wKp, wKi, wKd, error_w, last_error_w, integral_w);
  
  moveRobot(0, 0, output_w);
  
  last_error_w = error_w;


  
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

  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  //Serial.print(distance1);Serial.print("|");Serial.print(distance2);Serial.print("|");Serial.print(distance3);Serial.print("|");Serial.println(distance4);
  if (done_home == 0){
  if (!isGerakkankeRunning && a==0 && b==0) { //tambahin kondisi sensor warna dan ultra
    if(distance1 <= 10){
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
        stepsilo3();
        a=0; b=0;
      }
    }

    else if (data1_str == "4" && !isGerakkankeRunning) {
      if (b == 0) {
        isGerakkankeRunning = true;
        siklus_home = 0;
        ballremoved = false;
        stepsilo4();
        a=0; b=0;
      }
    }

    else if (data1_str == "5" && !isGerakkankeRunning) {
      if (b == 0) {
        isGerakkankeRunning = true;
        siklus_home = 0;
        ballremoved = false;
        stepsilo5();
        a=0; b=0;
      }
    }

  if (!data1_str) Serial.end();
  }
  
}


void motor1rollers() {
  analogWrite(motorroller1pwm, 170);
  digitalWrite(motorroller1Pin1, LOW);
  digitalWrite(motorroller1Pin2, HIGH);

}

void motor2roller() {
  analogWrite(motorroller2pwm, 200);
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

int homing(){
  ishoming = true;
  //Serial.print(erromega);  Serial.print(" | ");  Serial.print(erhomy_1);  Serial.print(" | ");  Serial.println(erhomy_2);

  while (siklus_homing==0){
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  long spy = 30;
  long spx = 30;
  long erhomx = (distance4 - spx)*-1;
  long erhomy_1 = distance2 - spy;
  long erhomy_2 = distance3 - spy;
  long erromega = (distance2 - distance3);
  
  moveRobot(erhomx,erhomy_2,0);
  
  if ((erhomx >= -1 && erhomx <= 1)&&(erhomy_2 >= -1 && erhomy_2 <=1 )){
  if (millis() >= time_now + interval){
    time_now += interval;
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  gerakke(0,0,0);
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  delay(1000);
  siklus_homing = 1;
  }
  }

  }

  while(siklus_homing==1){
    
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  long spy = 30;
  long spx = 30;
  long erhomx = (distance4 - spx)*-1;
  long erhomy_1 = distance2 - spy;
  long erhomy_2 = distance3 - spy;
  long erromega = (distance2 - distance3);
    moveRobot(0,0,erromega);
  if (erromega >= -1 && erromega <= 1){
  if (millis() >= time_now + interval){
    time_now += interval;
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  gerakke(0,0,0);
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  delay(1000);
  siklus_homing = 2;
  }
  }    
  }



 
  while(siklus_homing==2){
    gerakke(110, -190, 0);
      if ((error_x > -2.0 && error_x < 2.0) && (error_y > -2.0 && error_y < 2.0) && (error_w > -1.0 && error_w < 1.0)) {
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        siklus_homing = 3;

      }     
  }

  while(siklus_homing==3){
    gerakke(0, 0, 0);
      if ((error_x > -2.0 && error_x < 2.0) && (error_y > -2.0 && error_y < 2.0) && (error_w > -1.0 && error_w < 1.0)) {
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        siklus_homing = 4;
        return 1;
        isGerakkankeRunning = false;
      }     
  }

}


int stepsilo1(){


  long erromega;
  ahuy:
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  ishoming = false;
  //bool ishoming = false;
  //Serial.print(erromega);  Serial.print(" | ");  Serial.print(erhomy_1);  Serial.print(" | ");  Serial.println(erhomy_2);

  while (siklus_home==0){
   
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  long spy = 30;
  long spx = 30;
  long erhomx = (distance4 - spx)*-1;
  long erhomy_1 = distance2 - spy;
  long erhomy_2 = distance3 - spy;
  long erromega = (distance2 - distance3);
  
  moveRobot(erhomx,erhomy_2,0);
  if (millis() >= time_now + interval){
    time_now += interval;
  if ((erhomx >= -1 && erhomx <= 1)&&(erhomy_2 >= -1 && erhomy_2 <=1 )){
  moveRobot(0,0,0);
  
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  delay(3000);
  siklus_home = 1;
  break;
  }
  }

  }

  while(siklus_home==1){
    
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  erromega = (distance2 - distance3);
    gerakwke(erromega);
    if (error_w > -1.0 && error_w < 1.0){
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        moveRobot(0,0,0);
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        delay(1000);
        siklus_home = 2;
        break;
      }
       
  }
      



 
  while(siklus_home==2){
    
    if (!ballremoved) gerakke(0, 0, 0);
    else  gerakke(110, -190, 0);
      if ((error_x > -2.0 && error_x < 2.0) && (error_y > -2.0 && error_y < 2.0) && (error_w > -1.0 && error_w < 1.0)) {
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        siklus_home = 3;
        break;
      }     
  }

  while(siklus_home==3){
    
    gerakke(0, 0, 0);
      if ((error_x > -2.0 && error_x < 2.0) && (error_y > -2.0 && error_y < 2.0) && (error_w > -1.0 && error_w < 1.0)) {
         if (!ballremoved) {
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        delay(1000);
        motor1rollers();
        delay(4000);
        motorrollerstop();
        siklus_home = 4;
        delay(1000);
        break;
         }
         else {
          siklus_home = 5;
          break;
         }
      }     
  }

  while(siklus_home == 4){
    
    ballremoved = true;
    siklus_home = 0;
    pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
    break;
    }

    if(siklus_home != 5 && ballremoved) goto ahuy;

    while(siklus_home == 5){
    ballremoved = false;
    isGerakkankeRunning = false;
    break;
    }
  
  }

int stepsilo2(){


  long erromega;
  ahuy:
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  ishoming = false;
  //bool ishoming = false;
  //Serial.print(erromega);  Serial.print(" | ");  Serial.print(erhomy_1);  Serial.print(" | ");  Serial.println(erhomy_2);

  while (siklus_home==0){

  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  long spy = 30;
  long spx = 30;
  long erhomx = (distance4 - spx)*-1;
  long erhomy_1 = distance2 - spy;
  long erhomy_2 = distance3 - spy;
  long erromega = (distance2 - distance3);
  
  moveRobot(erhomx,erhomy_2,0);
  if (millis() >= time_now + interval){
    time_now += interval;
  if ((erhomx >= -1 && erhomx <= 1)&&(erhomy_2 >= -1 && erhomy_2 <=1 )){
  moveRobot(0,0,0);
  
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  delay(3000);
  siklus_home = 1;
  break;
  }
  }

  }

  while(siklus_home==1){
    
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  erromega = (distance2 - distance3);
    gerakwke(erromega);
    if (error_w > -1.0 && error_w < 1.0){
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        moveRobot(0,0,0);
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        delay(1000);
        siklus_home = 2;
        break;
      }
        
  }
      



 
  while(siklus_home==2){
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  long spy = 5;
  long spx = 30;
  long erhomx = (distance4 - spx)*-1;
  long erhomy_1 = distance2 - spy;
  long erhomy_2 = distance3 - spy;
    
    if (!ballremoved) gerakke(60, -10, 0);
    else  gerakke(110, -190, 0);
      if ((error_x > -2.0 && error_x < 2.0) && (error_y > -2.0 && error_y < 2.0) && (error_w > -1.0 && error_w < 1.0)) {
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        delay(1000);
        siklus_home = 3;
        break;
      }
           
  }


  while(siklus_home==3){
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  erromega = (distance2 - distance3);
    gerakwke(erromega);
    if (error_w > -1.0 && error_w < 1.0){
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        moveRobot(0,0,0);
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        delay(1000);
        siklus_home = 4;
        break;
      }
    }
  while(siklus_home ==4){
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  long spy = 8;
  long spx = 30;
  long erhomx = (distance4 - spx)*-1;
  long erhomy_1 = distance2 - spy;
  long erhomy_2 = distance3 - spy;
  long erromega = (distance2 - distance3);
  
  moveRobot(0,erhomy_2,0);
  if (millis() >= time_now + interval){
    time_now += interval;
  if (erhomy_2 >= -1 && erhomy_2 <=1 ){
  moveRobot(0,0,0);
  
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  delay(3000);
  siklus_home = 5;
  break;
  }
  }    
    
    }

  while(siklus_home==5){

    gerakke(0, 0, 0);
      if ((error_x > -2.0 && error_x < 2.0) && (error_y > -2.0 && error_y < 2.0) && (error_w > -1.0 && error_w < 1.0)) {
         if (!ballremoved) {
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        delay(1000);
        motor1rollers();
        delay(4000);
        motorrollerstop();
        siklus_home = 6;
        delay(1000);
        break;
         }
         else {
          siklus_home = 7;
          break;
         }
      }     
  }

  while(siklus_home == 6){
    
    ballremoved = true;
    siklus_home = 0;
    pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
    break;
    }

    if(siklus_home != 7 && ballremoved) goto ahuy;

    while(siklus_home == 7){
    ballremoved = false;
    isGerakkankeRunning = false;
    break;
    }
  
  }

int stepsilo3(){


  long erromega;
  ahuy:
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  ishoming = false;
  //bool ishoming = false;
  //Serial.print(erromega);  Serial.print(" | ");  Serial.print(erhomy_1);  Serial.print(" | ");  Serial.println(erhomy_2);

  while (siklus_home==0){
   
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  long spy = 30;
  long spx = 30;
  long erhomx = (distance4 - spx)*-1;
  long erhomy_1 = distance2 - spy;
  long erhomy_2 = distance3 - spy;
  long erromega = (distance2 - distance3);
  
  moveRobot(erhomx,erhomy_2,0);
  if (millis() >= time_now + interval){
    time_now += interval;
  if ((erhomx >= -1 && erhomx <= 1)&&(erhomy_2 >= -1 && erhomy_2 <=1 )){
  moveRobot(0,0,0);
  
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  delay(3000);
  siklus_home = 1;
  break;
  }
  }

  }

  while(siklus_home==1){
    
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  erromega = (distance2 - distance3);
    gerakwke(erromega);
    if (error_w > -1.0 && error_w < 1.0){
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        moveRobot(0,0,0);
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        delay(1000);
        siklus_home = 2;
        break;
      }
        
  }
      



 
  while(siklus_home==2){
    
    if (!ballremoved) gerakke(120, 5, 0);
    else  gerakke(110, -190, 0);
      if ((error_x > -2.0 && error_x < 2.0) && (error_y > -2.0 && error_y < 2.0) && (error_w > -1.0 && error_w < 1.0)) {
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        
        siklus_home = 3;
        break;
      }     
  }

  while(siklus_home==3){
    
    gerakke(0, 0, 0);
      if ((error_x > -2.0 && error_x < 2.0) && (error_y > -2.0 && error_y < 2.0) && (error_w > -1.0 && error_w < 1.0)) {
         if (!ballremoved) {
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        delay(1000);
        motor1rollers();
        delay(4000);
        motorrollerstop();
        siklus_home = 4;
        delay(1000);
        break;
         }
         else {
          siklus_home = 5;
          break;
         }
      }     
  }

  while(siklus_home == 4){
    
    ballremoved = true;
    siklus_home = 0;
    pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
    break;
    }

    if(siklus_home != 5 && ballremoved) goto ahuy;

    while(siklus_home == 5){
    ballremoved = false;
    isGerakkankeRunning = false;
    break;
    }
  
  }


int stepsilo4(){


  long erromega;
  ahuy:
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  ishoming = false;
  //bool ishoming = false;
  //Serial.print(erromega);  Serial.print(" | ");  Serial.print(erhomy_1);  Serial.print(" | ");  Serial.println(erhomy_2);

  while (siklus_home==0){
   
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  long spy = 30;
  long spx = 30;
  long erhomx = (distance4 - spx)*-1;
  long erhomy_1 = distance2 - spy;
  long erhomy_2 = distance3 - spy;
  long erromega = (distance2 - distance3);
  
  moveRobot(erhomx,erhomy_2,0);
  if (millis() >= time_now + interval){
    time_now += interval;
  if ((erhomx >= -1 && erhomx <= 1)&&(erhomy_2 >= -1 && erhomy_2 <=1 )){
  moveRobot(0,0,0);
  
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  delay(3000);
  siklus_home = 1;
  break;
  }
  }

  }

  while(siklus_home==1){
    
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  erromega = (distance2 - distance3);
    gerakwke(erromega);
    if (error_w > -1.0 && error_w < 1.0){
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        moveRobot(0,0,0);
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        delay(1000);
        siklus_home = 2;
        break;
      }
        
  }
      



 
  while(siklus_home==2){
    
    if (!ballremoved) gerakke(180, 0, 0);
    else  gerakke(110, -190, 0);
      if ((error_x > -2.0 && error_x < 2.0) && (error_y > -2.0 && error_y < 2.0) && (error_w > -1.0 && error_w < 1.0)) {
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        siklus_home = 3;
        break;
      }     
  }

  while(siklus_home==3){
    
    gerakke(0, 0, 0);
      if ((error_x > -2.0 && error_x < 2.0) && (error_y > -2.0 && error_y < 2.0) && (error_w > -1.0 && error_w < 1.0)) {
         if (!ballremoved) {
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        delay(1000);
        motor1rollers();
        delay(4000);
        motorrollerstop();
        siklus_home = 4;
        delay(1000);
        break;
         }
         else {
          siklus_home = 5;
          break;
         }
      }     
  }

  while(siklus_home == 4){
    
    ballremoved = true;
    siklus_home = 0;
    pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
    break;
    }

    if(siklus_home != 5 && ballremoved) goto ahuy;

    while(siklus_home == 5){
    ballremoved = false;
    isGerakkankeRunning = false;
    break;
    }
  
  }

int stepsilo5(){


  long erromega;
  ahuy:
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  ishoming = false;
  //bool ishoming = false;
  //Serial.print(erromega);  Serial.print(" | ");  Serial.print(erhomy_1);  Serial.print(" | ");  Serial.println(erhomy_2);

  while (siklus_home==0){
   
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  long spy = 30;
  long spx = 30;
  long erhomx = (distance4 - spx)*-1;
  long erhomy_1 = distance2 - spy;
  long erhomy_2 = distance3 - spy;
  long erromega = (distance2 - distance3);
  
  moveRobot(erhomx,erhomy_2,0);
  if (millis() >= time_now + interval){
    time_now += interval;
  if ((erhomx >= -1 && erhomx <= 1)&&(erhomy_2 >= -1 && erhomy_2 <=1 )){
  moveRobot(0,0,0);
  
  pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
  delay(3000);
  siklus_home = 1;
  break;
  }
  }

  }

  while(siklus_home==1){
    
  distance1 = getDistance(ULTRASONIC_PIN1);
  distance2 = getDistance(ULTRASONIC_PIN2);
  distance3 = getDistance(ULTRASONIC_PIN3);
  distance4 = getDistance(ULTRASONIC_PIN4);
  erromega = (distance2 - distance3);
    gerakwke(erromega);
    if (error_w > -1.0 && error_w < 1.0){
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        moveRobot(0,0,0);
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        delay(1000);
        siklus_home = 2;
        break;
      }
        
  }
      



 
  while(siklus_home==2){
    
    if (!ballremoved) gerakke(240, 0, 0);
    else  gerakke(110, -190, 0);
      if ((error_x > -2.0 && error_x < 2.0) && (error_y > -2.0 && error_y < 2.0) && (error_w > -1.0 && error_w < 1.0)) {
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        siklus_home = 3;
        break;
      }     
  }

  while(siklus_home==3){
    
    gerakke(0, 0, 0);
      if ((error_x > -2.0 && error_x < 2.0) && (error_y > -2.0 && error_y < 2.0) && (error_w > -1.0 && error_w < 1.0)) {
         if (!ballremoved) {
        pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
        delay(1000);
        motor1rollers();
        delay(4000);
        motorrollerstop();
        siklus_home = 4;
        delay(1000);
        break;
         }
         else {
          siklus_home = 5;
          break;
         }
      }     
  }

  while(siklus_home == 4){
    
    ballremoved = true;
    siklus_home = 0;
    pulse1 = 0; pulse2 = 0; pulse3 = 0; pulse4 = 0;
    break;
    }

    if(siklus_home != 5 && ballremoved) goto ahuy;

    while(siklus_home == 5){
    ballremoved = false;
    isGerakkankeRunning = false;
    break;
    }
  
  }