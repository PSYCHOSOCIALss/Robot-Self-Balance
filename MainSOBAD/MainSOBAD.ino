#include <Wire.h>
#include <MPU6050_tockn.h>
#include <PID_v1.h>
#include "motorcontrol.h"
#include "IRController.h"


#define IN1 13
#define IN2 12
#define ENA 6
#define ENB 5
#define IN4 9
#define IN3 8

#define LED 7

int MIN_ABS_SPEED_A = 0;
int MIN_ABS_SPEED_B = 0;

MPU6050 mpu6050(Wire);
IRController ir(4);

double originalSetpoint = -2;
double input, output, setpoint = originalSetpoint;
double Kp = 18, Ki = 180, Kd = 0.25;
double speedOffset = 0, trunOffset = 0;

double boot = 10;
double bootLR = 55;
bool isTurn = false;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

motorcontrol my_motor(ENA, IN1, IN2, ENB, IN3, IN4, 0.8, 1.0);  // SpeedFactorA, SpeedFactorB

unsigned long commandTimeout = 200;  // สำหรับเข็คว่ามี IRcommand ไหมใน 0.2 วิ

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.setGyroOffsets(2.20, 0.76, -0.85);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  my_motor.begin();

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);

  ir.begin();
  Serial.println("Ready!");
  digitalWrite(LED, HIGH);
  delay(800);
  digitalWrite(LED, LOW);
  delay(400);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);
}

void loop() {
  // อ่านข้อมูลจากเซนเซอร์
  mpu6050.update();
  input = mpu6050.getAngleX();

  // คำนวณค่า PID
  pid.Compute();
  
  unsigned long now = millis();
  
  if (ir.available()) {
      IRCommand cmd = ir.getCommand();
      ir.resume();
      ir.updateLastCommandTime();

      switch (cmd) {
          case IRCommand::Forward:
              speedOffset = boot;
              if (setpoint < originalSetpoint + 5)
                setpoint += 1.5;
              digitalWrite(LED, HIGH);
              break;

          case IRCommand::Backward:
              speedOffset = -boot;
              if (setpoint > originalSetpoint - 5)
                setpoint -= 1.5;
              digitalWrite(LED, HIGH);
              break;

          case IRCommand::Left:
              // เลี้ยวซ้าย
//              isTurn = true;
              speedOffset = bootLR;
              trunOffset = -bootLR;
              my_motor.turnLeft(255);
              digitalWrite(LED, HIGH);
              break;

          case IRCommand::Right:
              // เลี้ยวขวา
//              isTurn = true;
              speedOffset = bootLR;
              trunOffset = bootLR;
              digitalWrite(LED, HIGH);
              break;
          default:
              break;
      }
  }

  // กลับสู่สภาวะ Default ถ้าไม่มีคำสั่ง IR มานานเกินไป
  if (now - ir.getLastCommandTime() > commandTimeout) {
     MIN_ABS_SPEED_A = 40;
     MIN_ABS_SPEED_B = 40;
     digitalWrite(LED, LOW);
     trunOffset = 0;
     speedOffset = 0;
     isTurn = false;
     setpoint = originalSetpoint;
  }

//  if (!isTurn)
    my_motor.move(output, speedOffset, trunOffset, MIN_ABS_SPEED_A, MIN_ABS_SPEED_B);

//  // แสดงข้อมูลใน Serial Monitor
  Serial.print(" | Angle: ");
  Serial.println(input);
 Serial.print(" | Output: ");
 Serial.println(output);
 Serial.print(" | Setpoint: ");
 Serial.println(setpoint);


  delay(10);
}
