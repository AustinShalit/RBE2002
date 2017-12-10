
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

uint8_t state = 0;

MoveBaseClient* ac;

void emptyCallback() {

}

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

void turnInPlaceDoneCallback(const actionlib::SimpleClientGoalState& state, const ResultConstPtr& result) {
  state = 2;
}

void stateCallback(const std_msgs::UInt8::ConstPtr& msg) {
  state = msg->data;

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
        // check for flame (turn in place)
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        for (int i = 1; i <= 3; i++) {
          goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(i * 0.785);
          ac->sendGoal(goal);
        }
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(4 * 0.785);
        ac->sendGoal(goal, &doneCb, &emptyCallback, &emptyCallback);
        break;
      }
      case 3: {
        // calculate flame location
        if (ac->getState().isDone()) {
          state = 0;
        }
        break;
      }
      case 4: {
        // blow out flame
        break;
      }
      case 5: {
        // return home
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 0.0;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

        ac->sendGoal(goal);

        if (ac->getState().isDone()) {
          state = 0;
        }
        break;
      }
    }
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

  while(ros::ok()) {

    if (stateMessage.data != state) {
      state_pub.publish(stateMessage);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
