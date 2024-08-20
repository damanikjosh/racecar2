#ifndef ROS_H
#define ROS_H

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>


#include <IMU.h>

#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/imu.h>
#include "Pitches.h"

volatile float remote_traction, remote_steering;
unsigned long last_traction_time;
unsigned long last_steering_time;

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);


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

  void publishThrottle(float velocity) {
    velocity_ref_msg.data = velocity;
    RCSOFTCHECK(rcl_publish(&velocity_publisher, &velocity_ref_msg, NULL));
  }

  void publishSteering(float steering) {
    steering_ref_msg.data = steering;
    RCSOFTCHECK(rcl_publish(&steering_publisher, &steering_ref_msg, NULL));
  }

  void publishImu(cIMU *imu) {
    std_msgs__msg__Header header;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    imu_msg.header.stamp.sec = ts.tv_sec;
    imu_msg.header.stamp.nanosec = ts.tv_nsec;
    imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");

    imu_msg.angular_velocity.x = imu->gyroData[0];
    imu_msg.angular_velocity.y = imu->gyroData[1];
    imu_msg.angular_velocity.z = imu->gyroData[2];
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_msg.linear_acceleration.x = imu->accData[0];
    imu_msg.linear_acceleration.y = imu->accData[1];
    imu_msg.linear_acceleration.z = imu->accData[2];
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.orientation.w = imu->quat[0];
    imu_msg.orientation.x = imu->quat[1];
    imu_msg.orientation.y = imu->quat[2];
    imu_msg.orientation.z = imu->quat[3];

    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0.0025;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }

  void spinOnce() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1))); // Reduced to 10ms to match timer period
  }

private:

  rcl_publisher_t velocity_publisher;
  rcl_publisher_t steering_publisher;
  rcl_publisher_t imu_publisher;

  rcl_subscription_t velocity_subscriber;
  rcl_subscription_t steering_subscriber;

  std_msgs__msg__Float32 velocity_cmd_msg, velocity_ref_msg, steering_cmd_msg, steering_ref_msg;
  sensor_msgs__msg__Imu imu_msg;
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

    status = rclc_publisher_init_best_effort(
      &imu_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data");
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