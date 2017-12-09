
#include <Arduino.h>
#include <Wire.h>
#include <IRTrackingCamera.h>
#include <Encoder.h>
#include <L3G.h>
#include <LSM303.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>

#include "MC33926MotorDriver.h"
#include "XV11.h"

// Constants
// tan(fov / 2) / (width_pix / 2)
const int kHalfFlameCamera = 1023 / 2;
const double kFlameH = tan(0.576 / 2) / kHalfFlameCamera;

// Hardware
// Motors
MC33926MotorDriver md(9, 8, 6, 10, 7, 5);
XV11 xv11(13);
L3G gyro;
LSM303 compass;

// Sensors
IRTrackingCamera flameCamera;
Encoder leftEncoder(18, 19);
Encoder rightEncoder(2, 3);

// ROS
ros::NodeHandle nh;

// Publishers
std_msgs::Int16 encoderLeftMessage;
ros::Publisher encoderLeftPublisher("lwheel", &encoderLeftMessage);

std_msgs::Int16 encoderRightMessage;
ros::Publisher encoderRightPublisher("rwheel", &encoderRightMessage);

std_msgs::Float32 flameHAngleMessage;
ros::Publisher flameHAnglePublisher("flameh_angle", &flameHAngleMessage);

sensor_msgs::Imu imuMessage;
ros::Publisher imuPublisher("imu/data", &imuMessage);

// Subscribers
void motorLeftCb(const std_msgs::Float32& message){
    md.SetM1Speed(message.data);
}
ros::Subscriber<std_msgs::Float32> motorLeftSubscriber("lmotor_cmd", &motorLeftCb);

void motorRightCb(const std_msgs::Float32& message){
    md.SetM2Speed(message.data);
}
ros::Subscriber<std_msgs::Float32> motorRightSubscriber("rmotor_cmd", &motorRightCb);

void lidarSpeedCb(const std_msgs::UInt16& message){
    xv11.Update(message.data);
}
ros::Subscriber<std_msgs::UInt16> lidarSpeedSubscriber("rpms", &lidarSpeedCb);

void lidarEnableCb(const std_msgs::Bool& message){
    xv11.Enable(message.data);
}
ros::Subscriber<std_msgs::Bool> lidarEnableSubscriber("lidar_enable", &lidarEnableCb);

void fanEnableCb(const std_msgs::Bool& message){
    // Enable the fan
}
ros::Subscriber<std_msgs::Bool> fanEnableSubscriber("fan_enable", &fanEnableCb);

void buildGyroData() {
    gyro.read();

    imuMessage.header.stamp = nh.now();
    imuMessage.header.frame_id = "/base_imu";
    imuMessage.orientation_covariance[0] = -1;
    imuMessage.angular_velocity.x = gyro.g.x;
    imuMessage.angular_velocity.y = gyro.g.y;
    imuMessage.angular_velocity.z = gyro.g.z;
    imuMessage.linear_acceleration.x = compass.a.x;
    imuMessage.linear_acceleration.y = compass.a.y;
    imuMessage.linear_acceleration.z = compass.a.z;
}


void setup() {
    // put your setup code here, to run once:
    Wire.begin();
    md.Init();
    flameCamera.initialize();
    gyro.init();
    compass.init();
    gyro.enableDefault();
    compass.enableDefault();

    nh.getHardware()->setBaud(115200);
    nh.initNode();

    nh.advertise(encoderLeftPublisher);
    nh.advertise(encoderRightPublisher);
    nh.advertise(flameHAnglePublisher);
    nh.advertise(imuPublisher);

    nh.subscribe(motorLeftSubscriber);
    nh.subscribe(motorRightSubscriber);
    nh.subscribe(lidarSpeedSubscriber);
    nh.subscribe(lidarEnableSubscriber);
    nh.subscribe(fanEnableSubscriber);
    
    xv11.Update(0);
}

void loop() {
    encoderLeftMessage.data = leftEncoder.read();
    encoderRightMessage.data = rightEncoder.read();

    flameCamera.update();
    if (flameCamera.Points[0].x == 1023) {
        flameHAngleMessage.data = -1.0; // Magic number to signify no flame found
    } else {
        flameHAngleMessage.data = atan((flameCamera.Points[0].x - kHalfFlameCamera) * kFlameH);
    }

    buildGyroData();

    encoderLeftPublisher.publish(&encoderLeftMessage);
    encoderRightPublisher.publish(&encoderRightMessage);
    flameHAnglePublisher.publish(&flameHAngleMessage);
    imuPublisher.publish(&imuMessage);

    nh.spinOnce();

    delay(10);
}
