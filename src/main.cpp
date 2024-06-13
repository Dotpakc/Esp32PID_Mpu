#include <Arduino.h>
#include <Wire.h>

#include <MPU6050_light.h>
#include <PID_v1.h>

#define MOTOR1_A 12
#define MOTOR1_B 14
#define MOTOR2_A 25
#define MOTOR2_B 26


double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//MPU6050 mpu6050(Wire, 0x68);
MPU6050 mpu(Wire);


unsigned long timer = 0;

void mpuSetup() {
  
  Serial.println(F("Calculating offsets, do not move CAR!"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("[MPU6050] Setup done");
}

void pidSetup() {
  Serial.println("[PID] Setup started");
  Setpoint = 45; // set the target angle
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255);
  Serial.println("[PID] Tuning parameters: ");
  Serial.print("Kp: ");
  Serial.println(consKp);
  Serial.print("Ki: ");
  Serial.println(consKi);
  Serial.print("Kd: ");
  Serial.println(consKd);
  Serial.println("[PID] Setup done");
}

void motorSetup() {
  Serial.println("[Motor] Setup started");
  pinMode(MOTOR1_A, OUTPUT);
  pinMode(MOTOR1_B, OUTPUT);
  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);
  Serial.println("[Motor] Setup done");
}

void motorControl(int speed) {

  if(speed > 150){
    analogWrite(MOTOR1_A, speed);
    analogWrite(MOTOR1_B, 0);
    analogWrite(MOTOR2_A, speed);
    analogWrite(MOTOR2_B, 0);
  } else if(speed < -150){
    analogWrite(MOTOR1_A, 0);
    analogWrite(MOTOR1_B, abs(speed));
    analogWrite(MOTOR2_A, 0);
    analogWrite(MOTOR2_B, abs(speed));
  }
}


void setup() {
  Serial.begin(115200);
  motorSetup();

  Serial.println("[MPU6050] Setup started");
  Wire.begin(21, 22);
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  
  mpuSetup();


  
  
  pidSetup();
}

void loop() {
   mpu.update();

  if(millis() - timer > 20){ // print data every second
    float angle = mpu.getAngleZ();
    Input = angle;
    myPID.Compute();
    Serial.print(angle);
    Serial.print(" ");
    Serial.print(Output);
    Serial.print(" ");
    Serial.println(Setpoint);
    motorControl(Output);
    timer = millis();
  }

}

