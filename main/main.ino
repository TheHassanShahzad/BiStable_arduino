#include <micro_ros_arduino.h>
#include <EEPROM.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using I2C
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;


#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}


// MPU stuff
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful

uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float yaw, pitch, roll;
float rad2deg = 180 / M_PI;
// end of MPU stuff


float eq;
float kp;
float ki;
float kd;
float wheel_radius = 0.115/2;
float target_inclination = 0.0;
float steer_vel = 0.0;
float incl_err;
float prev_incl_err = 0.0;
float integral;
unsigned long prevMillis;
unsigned long prev_loop_time = 0.0;


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Reading all 6 floats from twist message
  float target_inclination = msg->linear.x;
  float steer_velocity = msg->linear.y;
  float eq_change = msg->linear.z;
  float kp_change  = msg->angular.x;
  float ki_change = msg->angular.y;
  float kd_change = msg->angular.z;

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  //display.setCursor(0, 10);
  display.print("BiStable");

  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Eq = ");
  display.print(eq);
  display.print(" pitch=");
  display.println(pitch, 1);

  display.setCursor(0, 30);
  display.print("Kp = ");
  display.println(kp);
  
  display.setCursor(0, 40);
  display.print("Ki = ");
  display.println(ki);

  display.setCursor(0, 50);
  display.print("Kd = ");
  display.println(kd);
  
  display.display();
  //delay(2000);
  display.clearDisplay(); 
  
  if (eq_change != 0 or kp_change != 0 or ki_change != 0 or kd_change != 0){
    eq += eq_change;
    kp += kp_change;
    ki += ki_change;
    kd += kd_change;
    
    digitalWrite(LED_PIN, HIGH);
    
    EEPROM.put(0, eq);
    EEPROM.put(sizeof(float), kp);
    EEPROM.put(2 * sizeof(float), ki);
    EEPROM.put(3 * sizeof(float), kd);
    EEPROM.commit();
  }
  else{
    digitalWrite(LED_PIN, LOW);
  }

//  // Example usage: if linear velocity in x direction is 0, turn off LED; if 1, turn on LED
//  digitalWrite(LED_PIN, (linear_x == 0) ? LOW : HIGH);
}



void setup() {
  // MPU stuff
  Wire.begin();
  Serial.begin(115200);

  mpu.initialize();
  dmpReady = mpu.dmpInitialize() == 0;

  if (dmpReady) {
    mpu.setXGyroOffset(92);
    mpu.setYGyroOffset(-23);
    mpu.setZGyroOffset(-3);
    mpu.setXAccelOffset(-2700);
    mpu.setYAccelOffset(-555);
    mpu.setZAccelOffset(3850);
    
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    }
  // end of MPU stuff

  
  float read_eq, read_kp, read_ki, read_kd;
  
  EEPROM.begin(1024); 
  EEPROM.get(0, read_eq);
  EEPROM.get(sizeof(float), read_kp);
  EEPROM.get(2 * sizeof(float), read_ki);
  EEPROM.get(3 * sizeof(float), read_kd);

  eq = read_eq;
  kp = read_kp;
  ki = read_ki;
  kd = read_kd;
  
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "combined_data"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  // initialize the OLED object
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  // Clear the buffer.
  display.clearDisplay();

}

float calcAngularVel(float linearVel){ 
//  angular vel is in rad/s
//linear vel is in metres/s
  return linearVel / wheel_radius;
}

//void driveLeftStepper(float angularVel){
////  drive left stepper at some angular vel in rad/s
//}
//
//void driveRightStepper(float angularVel){
////  drive right stepper at some angular vel in rad/s
//}

float balance_pid(float err, float prev_err, float *integral_ptr, float interval){
  float control_signal;
  if (interval != 0.0) {
    *integral_ptr += err*interval; // CHANGE integral term calculation to trapezoidal rule if steady state error persists             
    float derivative = (err - prev_err)/interval;
    control_signal = kp*err + ki*integral + kd*derivative; 
  }
  else {
//    control_signal = kp*err + ki*integral; // REMOVE integral term if using trapezoidal rule
    control_signal = 0;
  }
  return control_signal;
}

void loop() {
  unsigned long currentMillis = millis();
  
  // MPU stuff
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    pitch = ypr[0] * rad2deg;
    Serial.println(pitch);
  }
  // end of MPU stuff

  // PID
  incl_err = target_inclination - eq - pitch; 
  float balance_vel = balance_pid(incl_err, prev_incl_err, &integral, prev_loop_time);

  // Stepper motor drive
//  right_wheel_vel_cmd = balance_vel + steer_vel;
//  left_wheel_vel_cmd = balance_vel - steer_vel;
//  driveRightStepper(right_wheel_vel_cmd);
//  driveLeftStepper(left_wheel_vel_cmd);
  
  prev_incl_err = incl_err;
  prevMillis = currentMillis;
  prev_loop_time = currentMillis - prevMillis;
  
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
