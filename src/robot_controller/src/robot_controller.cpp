
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

uint8_t state = 0;

MoveBaseClient* ac;

/*************
 * Callbacks *
 *************/

void flameHAngleCallback(const std_msgs::Float32::ConstPtr& msg) {
  if (msg->data != -1.0 && state != 3) {
    state = 3;

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(msg->data);

    ac->sendGoal(goal);
  } else if (state == 3) {
    ac->cancelAllGoals();
  }
}

void stateCallback(const std_msgs::UInt8::ConstPtr& msg) {
  state = msg->data;
}

void turnLeft() {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(2.09);
  ac->sendGoal(goal);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_controller");
  ros::NodeHandle n;

  ac = new MoveBaseClient("move_base", true);

  while(!ac->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Publisher state_pub = n.advertise<std_msgs::UInt8>("robot_state", 1000);
  ros::Subscriber state_sub = n.subscribe("robot_state", 1000, stateCallback);
  ros::Subscriber flameh_angle_sub = n.subscribe("flameh_angle", 1000, flameHAngleCallback);

  ros::Rate loop_rate(10);

  std_msgs::UInt8 stateMessage;
  bool stateChanged = true;

  while(ros::ok()) {
    switch (state) {
      case 0: {
        // do nothing
        break;
      }
      case 1: {
        // explore node
        break;
      }
      case 2: {
        // check for flame (turn in place) 1/3
        if (stateChanged) {
          turnLeft();
        } else if (ac->getState().isDone()) {
          state++;
        }
        break;
      }
      case 3: {
        // check for flame (turn in place) 2/3
        if (stateChanged) {
          turnLeft();
        } else if (ac->getState().isDone()) {
          state++;
        }
        break;
      }
      case 4: {
        // check for flame (turn in place) 3/3
        if (stateChanged) {
          turnLeft();
        } else if (ac->getState().isDone()) {
          state = 1;
        }
        break;
      }
      case 5: {
        // calculate flame location
        break;
      }
      case 6: {
        // blow out flame
        break;
      }
      case 7: {
        // return home
        if (stateChanged) {
          move_base_msgs::MoveBaseGoal goal;

          goal.target_pose.header.frame_id = "map";
          goal.target_pose.header.stamp = ros::Time::now();

          ac->sendGoal(goal);
        } else if (ac->getState().isDone()) {
          state = 0;
        }
        break;
      }
    }

    if (stateMessage.data != state) {
      state_pub.publish(stateMessage);
      stateChanged = true;
    } else {
      stateChanged = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
