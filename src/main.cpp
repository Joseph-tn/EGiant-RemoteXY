//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can Motor_1_Motor_2_ENle debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__WIFI_POINT
#include <WiFi.h>
#include <cmath>
#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "EG20"
#define REMOTEXY_WIFI_PASSWORD "aaaaaaaaa"
#define REMOTEXY_SERVER_PORT 6377
#define REMOTEXY_ACCESS_PASSWORD "aaaaaaaaa"


// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 61 bytes
  { 255,2,0,4,0,54,0,19,0,0,0,0,31,1,200,84,1,1,2,0,
  5,119,8,71,71,32,35,24,31,71,16,6,85,85,56,0,175,24,135,0,
  0,0,0,0,0,200,66,0,0,160,65,0,0,32,65,0,0,0,64,24,
  0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t joystick_01_x; // from -100 to 100
  int8_t joystick_01_y; // from -100 to 100

    // output variables
  float instrument_01; // from 0 to 100

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   

//////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define Motor_1_EN  14  
#define Motor_1_in1 27  
#define Motor_1_in2 26  

#define Motor_2_EN  25  
#define Motor_2_in1 33  
#define Motor_2_in2 32  

// const uint32_t FREQ = 490;
// const uint8_t PWM_CHANNEL = 0;
// const uint8_t RESOLUTION = 8;
// int8_t duty_cycle= 200;

void init_pinout();

/* defined two arrays with a list of pins for each motor */
unsigned char RightMotor[3] = 
  {Motor_1_in1, Motor_1_in2, Motor_1_EN};
unsigned char LeftMotor[3] = 
  {Motor_2_in2 ,Motor_2_in1, Motor_2_EN};

/*
   speed control of the motor
   motor - pointer to an array of pins
   v - motor speed can be set from -100 to 100
*/

void Wheel (unsigned char * motor, int v)
{
  RemoteXY.instrument_01 = static_cast<float> (v);
  if (v > 100) v = 100;
  if (v < -100) v = -100;
  if (v > 0) {
    digitalWrite(motor[0], HIGH);
    digitalWrite(motor[1], LOW);
    analogWrite(motor[2], v*2.55);
  }
  else if (v < 0) {
    digitalWrite(motor[0], LOW);
    digitalWrite(motor[1], HIGH);
    analogWrite(motor[2], (-v)*2.55);
  }
  else {
    digitalWrite(motor[0], LOW);
    digitalWrite(motor[1], LOW);
    analogWrite(motor[2], 0);
  }
}

void setup() {
  RemoteXY_Init(); 

  init_pinout(); 
  Serial.begin(115200);
}

void loop() { 
   /* event handler module RemoteXY */
  RemoteXY_Handler ();
  int rightSpeed = RemoteXY.joystick_01_y - RemoteXY.joystick_01_x;
  int leftSpeed = RemoteXY.joystick_01_y + RemoteXY.joystick_01_x;
  
  /* find the maximum absolute value between both motors */
  int maxSpeed = max(abs(rightSpeed), abs(leftSpeed));
  
  /* normalize if any motor speed exceeds the joystick range */
  const int MAX_JOYSTICK = 100;
  
  if (maxSpeed > MAX_JOYSTICK) {
    /* scale both motors proportionally to maintain the arc */
    float scale = (float)MAX_JOYSTICK / maxSpeed;
    rightSpeed = rightSpeed * scale;
    leftSpeed = leftSpeed * scale;
  }
  
  /* manage the motors with normalized speeds */
  Wheel(RightMotor, leftSpeed);
  Wheel(LeftMotor, rightSpeed);
}

void init_pinout() {
  // motor 1
  pinMode(Motor_1_EN,  OUTPUT);
  pinMode(Motor_1_in1, OUTPUT);
  pinMode(Motor_1_in2, OUTPUT);

  // motor 2
  pinMode(Motor_2_EN,  OUTPUT);
  pinMode(Motor_2_in1, OUTPUT);
  pinMode(Motor_2_in2, OUTPUT);
}

