#include "IFX007T-Motor-Control.h"

// ----------- Change to 0 if your motor has no hallsensor -------------
#define HALLSENSOR  1

IFX007TMotorControl MyMotor = IFX007TMotorControl();
bool direction = 1;     // 0 or 1
uint16_t speed = 0;
uint16_t input_num = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Start");

  MyMotor.begin();
  MyMotor.MotorParam.SensingMode = HALLSENSOR;
  MyMotor.MotorParam.MotorPolepairs = 3;

  MyMotor.MotorParam.PI_Reg_P = 0.01;  // P value for the PI-RPM regulator
  MyMotor.MotorParam.PI_Reg_I = 0.01;  // I value for the PI-RPM regulator

  MyMotor.configureBLDCMotor(MyMotor.MotorParam);

  Serial.println("set speed to start");
}

void loop() {
#if 0
  Serial.print(digitalRead(A3));
  Serial.print(" ");
  Serial.print(digitalRead(A2));
  Serial.print(" ");
  Serial.println(digitalRead(A1));
  delay(500);
#endif


#if 1
  if (speed) {
    //MyMotor.setHallBLDCmotorDCspeed(direction, speed, 0);
    MyMotor.setHallBLDCmotorRPMspeed(direction, speed, 0);
  }
  if (Serial.available() > 0) {
    uint8_t in = Serial.read();
    if ('0' <= in && in <= '9') {
      input_num = input_num * 10 + (in-'0');
    } else if (in == '\n') {
      if (input_num == 0) {
        speed = 0;
        MyMotor.end();
        Serial.println("stop motor");
      } else {
        if (speed == 0) {
          MyMotor.begin();
        }
        speed = input_num;
        Serial.print("set speed to ");
        Serial.println(speed);
      }
      
      input_num = 0;
    }
  }
#endif
}
