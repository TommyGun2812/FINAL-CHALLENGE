// Include Libraries to be used
#include <micro_ros_arduino.h>    //micro-ros-arduino library
#include <rcl/rcl.h>              //Core ROS 2 Client Library (RCL) for node management.
#include <rcl/error_handling.h>   //Error handling utilities for Micro-ROS.
#include <rclc/rclc.h>            //Micro-ROS Client library for embedded devices.
#include <rclc/executor.h>        //Micro-ROS Executor to manage callbacks
#include <std_msgs/msg/float32.h>   //Predefined ROS 2 message type
#include <stdio.h>                //Standard I/O library for debugging.

//Declare nodes to be used
rcl_node_t node;            //Represents a ROS 2 Node running on the microcontroller.

//Instantiate executor and its support classes
rclc_executor_t executor;   //Manages task execution (timers, callbacks, etc.).
rclc_support_t support;     //Data structure that holds the execution context of Micro-ROS, including its communication state, memory management, and initialization data.
rcl_allocator_t allocator;  //Manages memory allocation.
rcl_timer_t timer;  

//Declare Subscribers to be used
rcl_publisher_t encoder_publisher;
rcl_publisher_t ctrl_publisher;    // Publishes button state
rcl_subscription_t ref_subscriber;

//Declare Messages to be used
std_msgs__msg__Float32 msgS;  //Defines a message of type float32.
std_msgs__msg__Float32 msgP1;  //Defines a message of type float32.
std_msgs__msg__Float32 msgP2; 

float referencia = 0.0;
float ki         = 219.1700911;
float kp         = 17.4267264;
float kd         = 0.0;
float K1,K2,K3;            
float T          = 0.05;
float error[3]   = {0,0,0};
float u[2]       = {0,0};
double pos       = 0.0;
double pos_ant   = 0.0;
double vel,retro;

//Define Macros to be used
//Executes fn and goes to error_loop() function if fn fails.
#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// Executes fn, but ignores failures.
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
      
//Specifies GPIO pin 13 for controlling an LED
#define LED_PIN 23 //Define LED_PIN
#define PWM_PIN 4 //DEFINE PWM_PIN
#define CW_PIN 13 //DEFINE 
#define CCW_PIN 12
#define PWM_FRQ 980 //Define PWM Frequency
#define PWM_RES 8  //Define PWM Resolution
#define PWM_CHNL 0    //Define Channel
#define MAX_VEL 15.0
#define EN_A 19
#define EN_B 18
#define pi 3.1416

//Variables to be used
float pwm_set_point = 0.0;

//Define Error Functions
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED state
    delay(100); // Wait 100 milliseconds
  }
}

void IRAM_ATTR int_callback(){ 
  if(digitalRead(EN_B)==0){pos = pos + 1;}
  else{pos = pos - 1;}
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  retro = ((60.0 * 20.0 * (pos - pos_ant)) / (12.0 * 34.0)); 
  retro = retro * (pi/30);
  //retro = retro*pi/30.0;
  //msg.data = retro;
  //rcl_publish(&encoder_publisher, &msg, NULL); 
  pos_ant = pos;
  error[0] = referencia - retro;
  u[0] = (K1 * error[0]) + (K2 * error[1]) + (K3 * error[2]) + u[1];
  //msg.data = u[0];
  //rcl_publish(&encoder_publisher, &msg, NULL); 
  if(u[0]>255)       u[0]=255;
  else if(u[0]<-255) u[0]=-255;
  
  msgP1.data = retro;
  rcl_publish(&encoder_publisher, &msgP1, NULL);
  msgP2.data = u[0];
  rcl_publish(&ctrl_publisher, &msgP2, NULL);
  

  if(u[0]>0){
    //analogWrite(cw,u[0]);
    //analogWrite(ccw,0);
    digitalWrite(CW_PIN, HIGH);
    digitalWrite(CCW_PIN, LOW);
    ledcWrite(PWM_CHNL,  u[0]); 
  }
  else if (u[0] < 0){
    //analogWrite(cw,0);
    //analogWrite(ccw,-u[0]);
    digitalWrite(CW_PIN, LOW);
    digitalWrite(CCW_PIN, HIGH);
    ledcWrite(PWM_CHNL,  abs(u[0]));  
  } else{
    digitalWrite(CW_PIN, LOW);
    digitalWrite(CCW_PIN, LOW);
  }
    
  u[1]     = u[0];
  error[2] = error[1];
  error[1] = error[0];
}

//Define callbacks
void subscription_callback(const void * msgin){  
  //Get the message received and store it on the message msg
  const std_msgs__msg__Float32 * msgS = (const std_msgs__msg__Float32 *)msgin;
  pwm_set_point = constrain(msgS->data, -15, 15);
  referencia = pwm_set_point;
  
}



//Setup
void setup() {
  K1         = kp + (T * ki) + (kd/T);
  K2         = -kp - (2.0 * kd/T);
  K3         = kd/T;
  set_microros_transports(); // Initializes communication between ESP32 and the ROS 2 agent (Serial).
  //Setup Microcontroller Pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(CW_PIN, OUTPUT);
  pinMode(CCW_PIN, OUTPUT);

  pinMode(EN_A, INPUT);
  pinMode(EN_B, INPUT);
  attachInterrupt(EN_A, int_callback, RISING);

  digitalWrite(LED_PIN, HIGH); 
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);  //Setup the PWM
  ledcAttachPin(PWM_PIN, PWM_CHNL);       //Setup Attach the Pin to the Channel   
  pos       = 0.0;
  pos_ant   = 0.0;
  //Connection delay
  delay(2000);
  //Initializes memory allocation for Micro-ROS operations.
  allocator = rcl_get_default_allocator();

  //Creates a ROS 2 support structure to manage the execution context.
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "esp32_controlator", "", &support));

  // create subscriber
  RCCHECK(rclc_publisher_init_best_effort(
      &encoder_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "encoder"));

  RCCHECK(rclc_publisher_init_best_effort(
      &ctrl_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "ctrl"));

  RCCHECK(rclc_subscription_init_best_effort(
      &ref_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "ref"));

  const unsigned int timer_timeout = 50;  
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  // Register suscription with executor
  RCCHECK(rclc_executor_add_subscription(&executor, &ref_subscriber, &msgS, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  //Executor Spin
  delay(25);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(25)));  //Executor Spin
}