#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <EEPROM.h>
#include <Arduino.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

#include <AccelStepper.h>

// Define the pins connected to the A4988 driverS
const int L_STEP_PIN = 33; 
const int L_DIR_PIN = 25; 
const int R_STEP_PIN = 12; 
const int R_DIR_PIN = 13;
const int L_MS1 = 32;
const int L_MS2 = 19;
const int L_MS3 = 18;
const int R_MS1 = 26;
const int R_MS2 = 27;
const int R_MS3 = 14;
const int LED_PIN = 2;

AccelStepper L_stepper(1, L_STEP_PIN, L_DIR_PIN); // Define left stepper motor object
AccelStepper R_stepper(1, R_STEP_PIN, R_DIR_PIN); // Define right stepper motor object

TaskHandle_t Core1Task; // Task handle for Core 1

#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire, 0.001, 0.999);

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


float pitch = 0.0;
int steps_per_cycle = 1600;
float target_angle = 1.0;
int r_speed = 0.0;
int l_speed = 0.0;
int max_vel = 10000;
int accel = 3000;
//float target_inclination, steer_velocity, eq, kp, ki, kd;
float target_inclination;
float steer_velocity;
float eq = 0.0;
float kp = 0.0;
float ki = 0.0;
float kd = 0.0;
float prev_error = 0.0;
float integral = 0.0;
float steer_multiplier = 10000;
float pitch_threshold = 20;
//float inclination multiplier = 

void driveStepper(void *param){
  pinMode(L_MS1, OUTPUT);
  pinMode(L_MS2, OUTPUT);
  pinMode(L_MS3, OUTPUT);
  pinMode(R_MS1, OUTPUT);
  pinMode(R_MS2, OUTPUT);
  pinMode(R_MS3, OUTPUT);

  // Set microstep pins to HIGH for 16 microsteps
  digitalWrite(L_MS1, HIGH);
  digitalWrite(L_MS2, HIGH);
  digitalWrite(L_MS3, HIGH);
  digitalWrite(R_MS1, HIGH);
  digitalWrite(R_MS2, HIGH);
  digitalWrite(R_MS3, HIGH);

  L_stepper.setMaxSpeed(max_vel);
  L_stepper.setAcceleration(accel);
  R_stepper.setMaxSpeed(max_vel);
  R_stepper.setAcceleration(accel);

  while (true){    
    R_stepper.setSpeed(r_speed);
    L_stepper.setSpeed(l_speed);
    R_stepper.runSpeed(); 
    L_stepper.runSpeed();

  }
}
//twist message cb
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Reading all 6 floats from twist message
  target_inclination = msg->linear.x;
  steer_velocity = msg->linear.y;
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
//  display.setCursor(0, 30);
//  display.print("R speed = ");
//  display.println(r_speed);
  
  display.setCursor(0, 40);
  display.print("Ki = ");
  display.println(ki);
//  display.setCursor(0, 40);
//  display.print("L speed = ");
//  display.println(l_speed);

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
}

void setup() {
  delay(1000);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); 
  
  xTaskCreatePinnedToCore(
    driveStepper,          // Function to implement the task
    "Core 1 Task",      // Name of the task
    10000,              // Stack size in words
    NULL,               // Task input parameter
    0,                  // Priority of the task
    &Core1Task,         // Task handle
    0                   // Run task on Core 1
  );
  
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
    "/combined_data"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

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

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  // Clear the buffer.
  display.clearDisplay();

  
}

void loop() {
//  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  mpu6050.update();
  pitch = mpu6050.getAngleZ();
  float error = -target_inclination - eq - pitch; // sign of eq doesn't really matter as it's variable
  // target_inclination has a -ve sign so that the robot actually moves forward when joystick is forward
  float derivative = error - prev_error;
  integral += error;
  float control_signal = (kp*100) * error + (ki*100) * integral + (kd*100) * derivative;
  prev_error = error;

  if (abs(pitch) >= pitch_threshold){
    control_signal = 0;
  }

//  steer_velocity *= 00;
//  int move_speed = control_signal + (steer_velocity*steer_multiplier);
  r_speed = control_signal - (steer_velocity*steer_multiplier);
  l_speed = -(control_signal + (steer_velocity*steer_multiplier));
}
