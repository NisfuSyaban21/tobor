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

// Definisi pin encoder
const int encA4 = 21;
const int encB4 = 42;
const int encA3 = 20;
const int encB3 = 40;
const int encA1 = 18;
const int encB1 = 34;
const int encA2 = 19;
const int encB2 = 36;

int a;
int b;

float output_w;


const float kel_roda = 37.68;
const float pulse_per_rotation = 135; // Motor1

int pulse1 = 0;
int pulse2 = 0;
int pulse3 = 0;
int pulse4 = 0;
float x_enc = 0;
float y_enc = 0;
float w_enc = 0;

float jarakroda3;
float jarakroda4;


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


#define Button_pin A8//tombol reset pergerakan

void setup() {
  Serial.begin(9600);

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
  w_enc = ((pulse1+pulse2+pulse3+pulse4)/4)*90/200;

}

void moveRobot(float V_x, float V_y, float W) {

  float V1 = (-0.707 * V_x) + (0.707 * V_y) + W;
  float V2 = (-0.707 * V_x) - (0.707 * V_y) + W;
  float V3 = (0.707 * V_x) - (0.707 * V_y) + W;
  float V4 = (0.707 * V_x) + (0.707 * V_y) + W;

  controlMotor(motor1Pin1, motor1Pin2, motor1pwm, V1);
  controlMotor(motor2Pin1, motor2Pin2, motor2pwm, V2);
  controlMotor(motor3Pin1, motor3Pin2, motor3pwm, V3);
  controlMotor(motor4Pin1, motor4Pin2, motor4pwm, V4);
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
  pwmValue = min(100, max(pwmValue, 0)); // Batasi nilai PWM antara 0 dan 255
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

  float output_w = calculatePID(wKp, wKi, wKd, error_w, last_error_w, integral_w);
  float output_x = calculatePID(Kp, Ki, Kd, error_x, last_error_x, integral_x);
  float output_y = calculatePID(Kp, Ki, Kd, error_y, last_error_y, integral_y);
  
  moveRobot(output_x, output_y, output_w);

  last_error_x = error_x;
  last_error_y = error_y;
  last_error_w = error_w;


  
}


void loop(){
  if (Serial.available() > 0) { // Cek apakah ada data yang tersedia di Serial
    String receivedData = Serial.readString(); // Menerima data hingga newline ('\n')
    data1_str = receivedData;

    if (data1_str == "1"){
       if(b==0){
        gerakkanke1();
        if(a==4)b++;
        }
 } else if (data1_str =="2"){
       if(b==0){
        gerakkanke2();
        if(a==4)b++;
        }
  }
    }
    



//Serial.println(error_w); 

Serial.print(error_x);Serial.print(" | ");Serial.print(error_y);Serial.print(" | ");Serial.println(error_w);
  

}




void gerakkanke1(){
   if(a==0){
    gerakke(0,90,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }

  else if(a==1){
    gerakke(70,0,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }

  else if(a==2){
    gerakke(0,40,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }
  

  else if (a==3){
    gerakke(0,0,0);
  }
  }


void gerakkanke2(){
   if(a==0){
    gerakke(0,90,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }

  else if(a==1){
    gerakke(70,0,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }

  else if(a==2){
    gerakke(0,40,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }
  

  else if (a==3){
    gerakke(0,0,0);
  }
  }

void gerakkanke3(){
   if(a==0){
    gerakke(0,90,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }

  else if(a==1){
    gerakke(70,0,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }

  else if(a==2){
    gerakke(0,40,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }
  

  else if (a==3){
    gerakke(0,0,0);
  }
  }


void gerakkanke4(){
   if(a==0){
    gerakke(0,90,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }

  else if(a==1){
    gerakke(70,0,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }

  else if(a==2){
    gerakke(0,40,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }
  

  else if (a==3){
    gerakke(0,0,0);
  }
  }

void gerakkanke5(){
   if(a==0){
    gerakke(0,90,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }

  else if(a==1){
    gerakke(70,0,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }

  else if(a==2){
    gerakke(0,40,0);
    if((error_x>-2.0 && error_x<2.0)&&(error_y>-2.0 && error_y<2.0)&&(error_w>-1.0 && error_w<1.0))
    {
      pulse1=0;pulse2=0;pulse3=0;pulse4=0;
      a++;
    }
  }
  

  else if (a==3){
    gerakke(0,0,0);
  }
  }



  
  
