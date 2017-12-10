
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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient* ac;
bool robotEnabled = false;
bool stateChanged = true;
uint8_t state = 0;
double flameH = 0.0;
double flameV = 0.0;
double distFwd = 0.0;


void enableCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data == true) {
    state = 2;
    stateChanged = true;
  } else {
    ac->cancelAllGoals();
  }
  robotEnabled = msg->data;
}

void flameHAngleCallback(const std_msgs::Float32::ConstPtr& msg) {
  if (msg->data != -1.0) {
    if (state != 3) {
      state = 3;
      stateChanged = true;
    }
    flameH = msg->data;
  }
}

void flameVAngleCallback(const std_msgs::Float32::ConstPtr& msg) {
  if (msg->data != -1.0) {
    flameV = msg->data;
  }
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
  ros::Subscriber laser_sub = n.subscribe("scan", 1000, &scanCallback);
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher flame_pub = n.advertise<geometry_msgs::PointStamped>("flame", 1000);

  ros::Rate loop_rate(10);
  double timer = 0.0;

  while(ros::ok()) {
    if (robotEnabled) {
      switch (state) {
        case 1: {
          break;
        }
        case 2: {
          if (stateChanged) {
            timer = ros::Time::now().sec;
            stateChanged = false;
            ac->cancelAllGoals();
          }
          geometry_msgs::Twist twist;
          twist.angular.z = 0.4;
          twist_pub.publish(twist);

          // Lol... who would use time?
          if ((ros::Time::now().sec - timer) * 0.4 > 7) {
            twist.angular.z = 0.0;
            twist_pub.publish(twist);
            state = 1;
            stateChanged = true;
          }
          break;
        }
        case 3: {
          move_base_msgs::MoveBaseGoal goal;

          goal.target_pose.header.frame_id = "base_link";
          goal.target_pose.header.stamp = ros::Time::now();

          goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(flameH);

          ac->sendGoal(goal);
          ac->waitForResult();
          state = 4;
          break;
        }
        case 4: {
          geometry_msgs::PointStamped flameLoc;
          flameLoc.header.frame_id = "neato_laser";
          flameLoc.header.stamp = ros::Time::now();
          flameLoc.point.x = distFwd;
          flameLoc.point.z = 0.21 + (distFwd * tan(flameV));
          flame_pub.publish(flameLoc);
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
