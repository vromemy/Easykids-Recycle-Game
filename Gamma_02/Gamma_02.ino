#include <EasyKids3in1.h>
#include "Ayanami.h"

#define RX 26
#define TX 32

#define Left_sensor 33
#define Right_sensor 27
#define Start_button 0

float pvYaw = 0;
uint8_t rxCnt = 0, rxBuf[8];


float Power_FL, Power_FR, Power_BL, Power_BR;
float yawOffset = 0;  
float PID_yawOffset = 0;

float main_speed = 100;

float spd_L = main_speed;
float spd_R = main_speed;

bool buttonoccupied_R = false;
bool buttonoccupied_L = false;

bool isGrab = false;
bool isDown = false;

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
  float yaw = getYaw() - PID_yawOffset;

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

  Ayanami_Setup();

  Serial2.begin(115200,SERIAL_8N1,RX,TX);

  pinMode(Right_sensor, INPUT);
  pinMode(Left_sensor, INPUT);
  pinMode(Start_button, INPUT);

  servo(5, 0); // greater the value tighter the gripper
  servo(6, 95);

  error = 0;
  last_error = 0;
  integral = 0;

  // displayClear();
  // display.setTextSize(3);
  // display.print(Vbattery());
  // display.setCursor(0, 100);
  // display.print(batteryPercent());

  welcomeSong();

  ledSetBrightness(100);

}


//-------------------------------------------------------------------------------------------------------------------------------------------------//


void loop() {
  Ayanami_Gamepad();


  float yawRad = -getYawRad();

  float strafe = (gamepadLX[0] / 150.0) * 1.0;
  float forward = (gamepadLY[0] / 150.0) * 1.0;
  float turn = (gamepadRX[0] / 150.0) * 1.0;

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

  if (gamepadCircle[0] == 1) 
  {
    yawOffset = getYaw();
  }

  if (gamepadTriangle[0] == 1) 
  {
    PID_yawOffset = getYaw();
    delay(2000);
    servo(6, 0);
    isDown = true;
    PID_gyro_time(100, 4   , 0, 1, 1250);
    Brake();

    delay (200);

    servo(5, 60);
    isGrab = false;

    delay(200);  

    motor(1,-100);
    motor(2,-100);
    motor(3,-100);
    motor(4,-100);
    delay(1350);

    servo(6, 95);
    isDown = false;  

  }


  tick_aim();
  grab();
  Lift_arm();

  speed_set_aim();

  colours();

  // Serial.println(analogRead(Left_sensor));
  // Serial.println(analogRead(Right_sensor));

  // Serial.println(Power_FL);

  // Serial.println(gamepad[0][0]Circle);

}




void PID_gyro_time(float power, double Kp, double Ki, double Kd, int target_time_ms)
{

  long timeflies = 0;
  timeflies = millis();
  setpoint = 0;

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

  if (gamepadL1[0] == 1) {
    
    spd_L = 4;
    spd_R = 4;
    
  }
  else {

    spd_L = main_speed;
    spd_R = main_speed;

  }

}

void tick_aim()
{
    
  if (gamepadLeft[0] == 1) {
    if (!buttonoccupied_L) {
      motor(1, -20);
      motor(2, -20);
      motor(3, 20);
      motor(4, 20);
      delay(50);
      motor(1, 0);
      motor(2, 0);
      motor(3, 0);
      motor(4, 0);
      buttonoccupied_L = true;
    }
  } 
  else {
    buttonoccupied_L = false;
  }

  if (gamepadRight[0] == 1) {
    if (!buttonoccupied_R) {
      motor(1, 20);
      motor(2, 20);
      motor(3, -20);
      motor(4, -20);
      delay(50);
      motor(1, 0);
      motor(2, 0);
      motor(3, 0);
      motor(4, 0);
      buttonoccupied_R = true;
    }
  } 
  else {
    buttonoccupied_R = false;
  }

}


bool GrabOccupied = false;

void grab() {

  if (gamepadR1[0] == 1) {

    if (!GrabOccupied) {

      if (isGrab) {
        servo(5, 60);
        isGrab = false;
      }
      else {
        servo(5, 100);
        isGrab = true;
      }
      
      GrabOccupied = true;

    }
    
  } 
  else {

    GrabOccupied = false;

  }
     
}

bool ArmOccupied = false;

void Lift_arm() {

  if (gamepadCross[0] == 1) {

    if (!ArmOccupied) {

      if (isDown) {
        servo(5, 100);
        isGrab = true;

        delay(100);

        servo(6, 95);
        isDown = false;  
      }
      else {
        servo(5, 60);
        isGrab = false;

        delay(100);
        
        servo(6, 6);
        isDown = true;
      }

      ArmOccupied = true;

    }

  }
  else {

    ArmOccupied = false;

  }

}


void Brake() {
  motor(1,0);
  motor(2,0);
  motor(3,0);
  motor(4,0);
}



bool colourOccupied = false;
bool isBlue = false;

void colours() {

  if (digitalRead(0) == 1) { 

    if (!colourOccupied) {

      if (isBlue) {
        ledFillColor(RED);
        isBlue = false;
      }
      else {
        ledFillColor(BLUE);
        isBlue = true;
      }
      
      colourOccupied = true;

    }
    
  } 
  else {

    colourOccupied = false;

  }

}




