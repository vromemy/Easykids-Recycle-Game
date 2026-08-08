#include <EasyKids3in1.h>

#define RX 26
#define TX 32

#define Left_sensor 33
#define Right_sensor 27

float pvYaw = 0;
uint8_t rxCnt = 0, rxBuf[8];


float Power_FL, Power_FR, Power_BL, Power_BR;
float yawOffset = 0;

float main_speed = 100;

float spd_L = main_speed;
float spd_R = main_speed;


float power, Steer, error, last_error, proportional, integral, I, derivatives, setpoint; 







bool readIMU() 
{
  while (Serial2.available()) {
    rxBuf[rxCnt] = Serial2.read();
    if (rxCnt == 0 && rxBuf[0] != 0xAA) return false;
    rxCnt++;
    if (rxCnt == 8) { // package is complete
      rxCnt = 0;
      if (rxBuf[0] == 0xAA && rxBuf[7] == 0x55) { // data package is correct
        pvYaw = (int16_t)(rxBuf[1] << 8 | rxBuf[2]) / 100.f;
        return true;
      }
    }
  }
  return false;
}

double getYaw() 
{
  if (readIMU())
  {
    return pvYaw;
  }
  
}

double PID_yaw() 
{
  float yaw = getYaw() - yawOffset;

  return yaw;
}

double getYawRad() 
{
  float yaw = getYaw() - yawOffset;

  // if (yaw > 180) yaw -= 360;
  // if (yaw < -180) yaw += 360;

  return yaw * PI / 180.0;

}


void setup() {
  EasyKids_Setup();

  JoyController_Setup();

  Serial2.begin(115200,SERIAL_8N1,RX,TX);

  pinMode(Right_sensor, INPUT);
  pinMode(Left_sensor, INPUT);

  servo(5, 60); // greater the value tighter the gripper
  servo(6, 0);

  error = 0;
  last_error = 0;
  integral = 0;

  // displayClear();
  // display.setTextSize(3);
  // display.print(Vbattery());
  // display.setCursor(0, 100);
  // display.print(batteryPercent());

  welcomeSong();
}


void loop() {

  JoyController();


  float yawRad = -getYawRad();

  float strafe = (joyLX / 150.0) * 1.0;
  float forward = (joyLY / 150.0) * 1.0;
  float turn = (joyRX / 150.0) * 1.0;

  float rotX = strafe * cos(yawRad) - forward * sin(yawRad);
  float rotY = strafe * sin(yawRad) + forward * cos(yawRad);

  float d = fmax(fabs(rotX) + fabs(rotY) + fabs(turn), 1);

  Power_FL = ((rotY + rotX + turn) / d) * spd_L;
  Power_FR = ((rotY - rotX - turn) / d) * spd_R; 
  Power_BL = ((rotY - rotX + turn) / d) * spd_L;
  Power_BR = ((rotY + rotX - turn) / d) * spd_R;

  // float maxVal = max(
  //   max(abs(Power_FL), abs(Power_FR)),
  //   max(abs(Power_BL), abs(Power_BR))
  // );

  // if (maxVal > 1) {
  //   Power_FL /= 100;
  //   Power_FR /= 100;
  //   Power_BL /= 100;
  //   Power_BR /= 100;
  // }

  motor(1,Power_FL);
  motor(3,Power_FR);
  motor(2,Power_BL);
  motor(4,Power_BR);

  if (joyCross == 1) 
  {
    yawOffset = getYaw();
  }

  if (joyR1 == 1) 
  {
    yawOffset = getYaw();
    delay(500);
    PID_gyro_time(100, 3, 0, 1, 1000, true);
    delay(2000);

    motor(1,-100);
    motor(2,-100);
    motor(3,-100);
    motor(4,-100);
    delay(1000);
;

  }

  // speed_set_aim();
  tick_aim();

  // Serial.println(analogRead(Left_sensor));
  // Serial.println(analogRead(Right_sensor));

  // Serial.println(Power_FL);

  // Serial.println(joyCircle);

 

}




void PID_gyro_time(float power, double Kp, double Ki, double Kd, int target_time_ms, bool goingforward)
{

  long timeflies = 0;
  timeflies = millis();
  setpoint = 0;

  if (goingforward) {

    while ((millis() - timeflies) < target_time_ms)
    {
      error        = setpoint - PID_yaw();

      while (error > 180.0) error -= 360.0;
      while (error < -180.0) error += 360.0;

      proportional = Kp * error;
      integral     = integral + error;
      I            = integral * Ki;
      derivative   = Kd * (error - last_error); 
      Steer        = proportional + I + derivatives;

      if (abs(error < 1))   integral = 0;
      Steering(power, Steer);
      last_error = error;

      // Serial.println(PID_yaw());
    
    }

  }
  else {

    while ((millis() - timeflies) < target_time_ms)
    {
      error        = PID_yaw() - setpoint;

      while (error > 180.0) error -= 360.0;
      while (error < -180.0) error += 360.0;

      proportional = Kp * error;
      integral     = integral + error;
      I            = integral * Ki;
      derivative   = Kd * (error - last_error); 
      Steer        = proportional + I + derivatives;

      if (abs(error < 1))   integral = 0;
      Steering(power, Steer);
      last_error = error;

      // Serial.println(PID_yaw());
    
    }

  }

}

void Steering(float PW, float Str)
{
  if (Str == 0)
  {
    motor(1, PW);
    motor(2, PW);
    motor(3, PW);
    motor(4, PW);
  }
  else {
    //On( PW - Str , PW + Str );
    int M_L = constrain(PW - Str, -100, 100);
    int M_R = constrain(PW + Str, -100, 100);
    motor(1, M_L);
    motor(3, M_R);
    motor(2, M_L);
    motor(4, M_R);
  }
}

void  speed_set_aim() 
{

  if (joyL1 == 1) {
    
    if (analogRead(Right_sensor) < 100 && analogRead(Left_sensor) < 100) {
      spd_L = 0;
      spd_R = 0;
    }
    else if (analogRead(Left_sensor) < 100) {
      spd_L = 0;
    }
    else if (analogRead(Right_sensor) < 100) {
      spd_R = 0;
    }
    else {
      spd_L = 2;
      spd_R = 2;
    }

  }
  else {

    spd_L = main_speed;
    spd_R = main_speed;

  }

}



bool buttonoccupied_C = false;
bool buttonoccupied_S = false;

void tick_aim()
{
    
  if (joySquare == 1) {
    if (!buttonoccupied_S) {
      motor(1, -20);
      motor(2, -20);
      motor(3, 20);
      motor(4, 20);
      delay(50);
      motor(1, 0);
      motor(2, 0);
      motor(3, 0);
      motor(4, 0);
      buttonoccupied_S = true;
    }
  } 
  else {
    buttonoccupied_S = false;
  }

  if (joyCircle == 1) {
    if (!buttonoccupied_C) {
      motor(1, 20);
      motor(2, 20);
      motor(3, -20);
      motor(4, -20);
      delay(50);
      motor(1, 0);
      motor(2, 0);
      motor(3, 0);
      motor(4, 0);
      buttonoccupied_C = true;
    }
  } 
  else {
    buttonoccupied_C = false;
  }

}







