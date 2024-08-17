#ifndef ROS_H
#define ROS_H

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include "Pitches.h"

volatile float remote_traction, remote_steering;
volatile unsigned long last_traction_time;
volatile unsigned long last_steering_time;

void motorCb(const void* msg_in) {
  
  const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*) msg_in;
  remote_traction = msg->data;
  last_traction_time = millis();
}

void steeringCb(const void* msg_in) {
  
  const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*) msg_in;
  remote_steering = msg->data;
  last_steering_time = millis();
}


class ROS {
  #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return temp_rc;}}
  #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
  #define LED_PIN  13

public:

  void init() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  

    while (init_micro_ros() != RCL_RET_OK) {
      // Flash LED to indicate retrying
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        // playMelody(error_melody);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }
    }

    digitalWrite(LED_PIN, HIGH);
    playMelody(startup_melody);
  }

  void spinOnce() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1))); // Reduced to 10ms to match timer period
  }

private:
  rcl_publisher_t velocity_publisher;
  rcl_publisher_t steering_publisher;

  rcl_subscription_t velocity_subscriber;
  rcl_subscription_t steering_subscriber;

  std_msgs__msg__Float32 velocity_cmd_msg, velocity_ref_msg, steering_cmd_msg, steering_ref_msg;
  rclc_executor_t executor;
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;
  rcl_timer_t timer;

  rcl_ret_t init_micro_ros() {

    rcl_ret_t status;

    set_microros_transports();
    allocator = rcl_get_default_allocator();

    //create init_options
    status = rclc_support_init(&support, 0, NULL, &allocator);
    if (status != RCL_RET_OK) return status;

    // create node
    status = rclc_node_init_default(&node, "racecar_base", "", &support);
    if (status != RCL_RET_OK) return status;

    // create subscriber
    status = rclc_subscription_init_best_effort(
      &velocity_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "throttle/velocity/command");
    if (status != RCL_RET_OK) return status;
    
    status = rclc_publisher_init_best_effort(
      &velocity_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "throttle/velocity/reference");
    if (status != RCL_RET_OK) return status;
  
      // create subscriber
    status = rclc_subscription_init_best_effort(
      &steering_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "steering/position/command");
    if (status != RCL_RET_OK) return status;
    
    status = rclc_publisher_init_best_effort(
      &steering_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "steering/position/reference");
    if (status != RCL_RET_OK) return status;

    // create executor
    status = rclc_executor_init(&executor, &support.context, 2, &allocator);
    if (status != RCL_RET_OK) return status;
    status = rclc_executor_add_subscription(&executor, &velocity_subscriber, &velocity_cmd_msg, motorCb, ON_NEW_DATA);
    if (status != RCL_RET_OK) return status;
    status = rclc_executor_add_subscription(&executor, &steering_subscriber, &steering_cmd_msg, steeringCb, ON_NEW_DATA);
    if (status != RCL_RET_OK) return status;

    return RCL_RET_OK;
  }


  void error_loop(){
    while(1){
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }
  }
};

#endif