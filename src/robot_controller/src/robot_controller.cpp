
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "explore/explore.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient* ac;
bool robotEnabled = false;
bool stateChanged = true;
uint8_t state = 0;
float flameHAngle = 0.0;
float flameVAngle = 0.0;
float distFwd = 0.0;


void enableCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data == true) {
    state = 1;
    stateChanged = true;
  } else {
    ac->cancelAllGoals();
  }
  robotEnabled = msg->data;
}

void flameHAngleCallback(const std_msgs::Float32::ConstPtr& msg) {
  if (msg->data != -1.0) {
    if (state >= 1 && state < 3) {
      state = 3;
      stateChanged = true;
      ac->cancelAllGoals(); // Cancel old goals
      // Turn to face candle
      move_base_msgs::MoveBaseGoal goal;

      goal.target_pose.header.frame_id = "base_link";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(flameHAngle);

      ac->sendGoal(goal);
    }
  }

  flameHAngle = msg->data;
}

void flameVAngleCallback(const std_msgs::Float32::ConstPtr& msg) {
  flameVAngle = msg->data;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  distFwd = msg->ranges[0];
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_controller");
  ros::NodeHandle n;

  ac = new MoveBaseClient("move_base", true);

  while(!ac->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Subscriber enable_sub = n.subscribe("robot_enable", 1000, &enableCallback);
  ros::Subscriber flameh_angle_sub = n.subscribe("flameh_angle", 1000, &flameHAngleCallback);
  ros::Subscriber flamev_angle_sub = n.subscribe("flamev_angle", 1000, &flameVAngleCallback);
  ros::Subscriber laser_sub = n.subscribe("scan", 1000, &scanCallback);
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher flame_pub = n.advertise<geometry_msgs::PointStamped>("flame", 1000);
  ros::Publisher fanAngle_pub = n.advertise<std_msgs::Float32>("fan_angle", 1000);

  ros::Rate loop_rate(10);
  uint32_t timer = 0;

  ROS_INFO("Creating Explore Object");
  explore::Explore explore;
  ROS_INFO("Explore created!");

  while(ros::ok()) {
    if (robotEnabled) {
      switch (state) {
        case 0: {
          // Nothing
          break;
        }
        case 1: {
          ROS_INFO("1: Making explore plan");
          if (stateChanged) {
            stateChanged = false;
            explore.makePlan();
          }

          if (ac->getState().isDone()) {
            state++;
            ROS_INFO("Got to frontier");
            stateChanged = true;
          }
          break;
        }
        case 2: {
          if (stateChanged) {
            ROS_INFO("2: Spin");
            timer = ros::Time::now().sec;
            stateChanged = false;
            ac->cancelAllGoals();
          }
          geometry_msgs::Twist twist;
          twist.angular.z = 2.0;
          twist_pub.publish(twist);

          // Lol... who would use time?
          if ((ros::Time::now().sec - timer) > 7) {
            ROS_INFO("2: Spin complete");
            twist.angular.z = 0.0;
            twist_pub.publish(twist);
            state = 1;
            stateChanged = true;
          }
          break;
        }
        case 3: {
          ROS_INFO("3: Turn to face candle");
          ac->waitForResult();
          state = 4;
          break;
        }
        case 4: {
          ROS_INFO("4: Calculate position");
          // Calculate candle position
          geometry_msgs::PointStamped flameLoc;
          flameLoc.header.frame_id = "neato_laser";
          flameLoc.header.stamp = ros::Time::now();
          flameLoc.point.x = distFwd;
          flameLoc.point.z = 0.21 + (distFwd * tan(flameVAngle));

          tf::TransformListener listener(ros::Duration(10));
          listener.transformPoint("map", *flameLoc, flameLoc);
          flame_pub.publish(flameLoc);
          state = 5;
          break;
        }
        case 5: {
          ROS_INFO("5: Blow");
          // Blow out candle
          // Enable the blower
          // Exit: Wait for candle to be out and then start timer for some seconds
          // Proceed to next state & turn off blower
          if (stateChanged) {
            std_msgs::Float32 angle;
            angle.data = flameVAngle;
            fanAngle_pub.publish(angle);
            timer = 0;
            stateChanged = false;
          }

          if (timer == 0 && flameVAngle == -1.0) {
            ROS_INFO("5: Setting timer");
            timer = ros::Time::now().sec;
          }

          if (timer != 0 && ros::Time::now().sec - timer > 4) {
            ROS_INFO("5: Complete");
            std_msgs::Float32 angle;
            angle.data = -1.0;
            fanAngle_pub.publish(angle);
            state = 6;
            stateChanged = true;
          }
          break;
        }
        case 6: {
          ROS_INFO("6: Home");
          // Return home
          // Send move goal as 0, 0, 0 with a fixed frame of map
          // Exit: Move goal success
          // Proceed: case 0
          move_base_msgs::MoveBaseGoal goal;

          goal.target_pose.header.frame_id = "map";
          goal.target_pose.header.stamp = ros::Time::now();

          goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

          ac->sendGoal(goal);
          ac->waitForResult();
          ROS_INFO("6: We are Home");
          state = 0;
          break;
        }
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
