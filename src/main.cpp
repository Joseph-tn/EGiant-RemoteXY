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
#define REMOTEXY_WIFI_SSID "ESP32_AP"
#define REMOTEXY_WIFI_PASSWORD "123456789"
#define REMOTEXY_SERVER_PORT 6377
#define REMOTEXY_ACCESS_PASSWORD "123456789"

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__SOFTSERIAL

// #include <SoftwareSerial.h>

// // RemoteXY connection settings 
// #define REMOTEXY_SERIAL_RX 2
// #define REMOTEXY_SERIAL_TX 3
// #define REMOTEXY_SERIAL_SPEED 9600


// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 172 bytes
  { 255,5,0,8,0,165,0,19,0,0,0,0,31,2,106,200,200,80,2,1,
  2,3,0,5,14,20,150,150,131,12,59,59,0,2,26,31,71,238,75,75,
  75,20,19,54,54,56,0,2,24,135,0,0,0,0,0,0,200,66,0,0,
  160,65,0,0,32,65,0,0,0,64,24,0,131,41,3,21,35,80,1,30,
  12,2,17,2,31,80,97,103,101,0,6,4,0,4,18,50,7,90,8,2,
  25,75,32,2,26,71,18,60,75,75,72,20,58,58,56,0,2,24,135,0,
  0,0,0,0,0,200,66,0,0,160,65,0,0,32,65,0,0,0,64,24,
  0,4,10,48,7,90,133,33,74,25,160,2,26,131,41,3,21,35,88,2,
  25,9,2,17,2,31,80,97,103,101,0,9 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t slider; // =1 if page is visible, else =0, from 0 to 1
  int8_t joystick_01_x; // from -100 to 100
  int8_t joystick_01_y; // from -100 to 100
  int8_t slider_01; // from -100 to 100
  int8_t slider_02; // from -100 to 100

    // output variables
  float instrument_01; // from 0 to 100
  float instrument_02; // from 0 to 100
    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)

//////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define Motor_1_EN  14  
#define Motor_1_in1 27  
#define Motor_1_in2 26  


#define Motor_2_EN  25  
#define Motor_2_in1 33  
#define Motor_2_in2 32  

void init_pinout();
void Handler(int val2, int val1);
void forward_motor1(int speed);
void backward_motor1(int speed);
void forward_motor2(int speed);
void backward_motor2(int speed);
void stopMotors();

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
  if (v>100) v=100;
  if (v<-100) v=-100;
  if (v>0) {
    digitalWrite(motor[0], HIGH);
    digitalWrite(motor[1], LOW);
    analogWrite(motor[2], v*2.55);
  }
  else if (v<0) {
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
  Serial.begin(9600);
}

void loop() { 
   /* event handler module RemoteXY */
  RemoteXY_Handler ();

  Handler(RemoteXY.slider_01, RemoteXY.slider_02);
  /* manage the right motor */
  Wheel (RightMotor, RemoteXY.joystick_01_y - RemoteXY.joystick_01_x);
  /* manage the left motor */
  Wheel (LeftMotor, RemoteXY.joystick_01_y + RemoteXY.joystick_01_x);
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


// /////////////////////////////////////////////
// //             The Driver For Sliders            //
// /////////////////////////////////////////////

void Handler(int val2, int val1) {
  int speed1 = map(abs(val1), 0, 100, 0, 255); // FWD SPEED, Map val1 to PWM range (0-255)
  int speed2 = map(abs(val2), 0, 100, 0, 255); // Turn SPEED, Map val2 to PWM range (0-255)
  float final_speed = sqrt(speed1 * speed1 + (speed2 * speed2)) / 3.6;
  RemoteXY.instrument_01 = final_speed;

  // Priority to val2 for turning...why? (I want to make sure if i tell to turn it turns)
  // going forward is normal but turning is urgent.

  if (val2 < 0) {  // Turn right
    forward_motor1(speed2);    // Motor 1 moves forward
    backward_motor2(speed2);   // Motor 2 moves backward
    Serial.println("Turn right");
  } 
  else if (val2 > 0) {  // Turn left
    backward_motor1(speed2);   // Motor 1 moves backward
    forward_motor2(speed2);    // Motor 2 moves forward
    Serial.println("Turn Left");
  } 
  
  //********** Fwd/Bwd Slider **********//
  else {
    // If not turning, check val1 for forward/backward motion
    if (val1 > 0) {  // Move both motors forward
      forward_motor1(speed1);
      forward_motor2(speed1);
      Serial.println("Forward");

    } else if (val1 < 0) {  // Move both motors backward
      backward_motor1(speed1);
      backward_motor2(speed1);
      Serial.println("Backward");
    } else {
      // Stop motors if both val1 and val2 are 0
      stopMotors();
    }
  }
}


/////////////////////////////////////////////
//        Motor controller Funtions For Sliders  //
/////////////////////////////////////////////
void forward_motor1(int speed) {
  analogWrite(Motor_1_EN, speed);    // Set PWM speed for motor 1
  digitalWrite(Motor_1_in1, LOW);     // Forward direction for motor 1
  digitalWrite(Motor_1_in2, HIGH);
}

void backward_motor1(int speed) {
  analogWrite(Motor_1_EN, speed);    // Set PWM speed for motor 1
  digitalWrite(Motor_1_in1, HIGH);    // Backward direction for motor 1
  digitalWrite(Motor_1_in2, LOW);
}

void forward_motor2(int speed) {
  analogWrite(Motor_2_EN, speed);    // Set PWM speed for motor 2
  digitalWrite(Motor_2_in1, LOW);    // Forward direction for motor 2
  digitalWrite(Motor_2_in2, HIGH);
}

void backward_motor2(int speed) {
  analogWrite(Motor_2_EN, speed);    // Set PWM speed for motor 2
  digitalWrite(Motor_2_in1, HIGH);   // Backward direction for motor 2
  digitalWrite(Motor_2_in2, LOW);
}

void stopMotors() {
  // Stop both motors
  digitalWrite(Motor_1_in1, LOW);
  digitalWrite(Motor_1_in2, LOW);
  analogWrite(Motor_1_EN, 0);

  digitalWrite(Motor_2_in1, LOW);
  digitalWrite(Motor_2_in2, LOW);
  analogWrite(Motor_2_EN, 0);
}