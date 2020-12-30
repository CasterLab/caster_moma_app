#include <ros/ros.h>
#include <string>
#include <std_msgs/Float64.h>

#include <actionlib/client/simple_action_client.h>

#include <kinova_msgs/ArmJointAnglesActionGoal.h>
#include <kinova_msgs/JointAngles.h>
kinova_msgs::ArmJointAnglesActionGoal arm_joint_goal_msg;
bool joint_angles_status = false;

void armInitCallBack(const kinova_msgs::JointAngles::ConstPtr &joint_angles_msg) {
  ROS_INFO("%f",arm_joint_goal_msg.goal.angles.joint1);

  arm_joint_goal_msg.goal.angles.joint1 = joint_angles_msg->joint1;
  arm_joint_goal_msg.goal.angles.joint2 = joint_angles_msg->joint2;
  arm_joint_goal_msg.goal.angles.joint3 = joint_angles_msg->joint3;
  arm_joint_goal_msg.goal.angles.joint4 = joint_angles_msg->joint4;
  arm_joint_goal_msg.goal.angles.joint5 = joint_angles_msg->joint5;
  arm_joint_goal_msg.goal.angles.joint6 = joint_angles_msg->joint6;
  arm_joint_goal_msg.goal.angles.joint7 = joint_angles_msg->joint7;
  joint_angles_status = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "caster_moma_app_node");
  ros::NodeHandle nh, private_nh("~");

  // // set up spinner
  ros::AsyncSpinner spinner(3);
  spinner.start();

  ros::Subscriber joint_angles_sub = nh.subscribe("j2s6s200_driver/out/joint_angles", 1000, armInitCallBack);
  ros::Publisher arm_joint_pub = nh.advertise<kinova_msgs::ArmJointAnglesActionGoal>("j2s6s200_driver/joints_action/joint_angles/goal", 1000);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(2.0).sleep();

  ros::Rate loop_rate(1);
  while(ros::ok()) {
    if(joint_angles_status) {
      ROS_INFO("%f",arm_joint_goal_msg.goal.angles.joint1);
      ROS_INFO("%f",arm_joint_goal_msg.goal.angles.joint2);
      ROS_INFO("%f",arm_joint_goal_msg.goal.angles.joint3);
      ROS_INFO("%f",arm_joint_goal_msg.goal.angles.joint4);
      ROS_INFO("%f",arm_joint_goal_msg.goal.angles.joint5);
      ROS_INFO("%f",arm_joint_goal_msg.goal.angles.joint6);
      ROS_INFO("%f",arm_joint_goal_msg.goal.angles.joint7);

      arm_joint_pub.publish(arm_joint_goal_msg);

      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("All finish");

  return 0;
}
