#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <aruco_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

bool receive_data = false;
geometry_msgs::TransformStamped marker_transform, object_transform;

void MarkerPoseCallback(const aruco_msgs::MarkerArray::ConstPtr& msg) {
  static tf2_ros::TransformBroadcaster br;
  
  marker_transform.header.stamp = ros::Time::now();
  marker_transform.header.frame_id = "kinect2_ir_optical_frame";
  marker_transform.child_frame_id = "marker";
  marker_transform.transform.translation.x = msg->markers[0].pose.pose.position.x;
  marker_transform.transform.translation.y = msg->markers[0].pose.pose.position.y;
  marker_transform.transform.translation.z = msg->markers[0].pose.pose.position.z;
  // tf2::Quaternion q;
  // q.setRPY(0, 0, msg->theta);
  marker_transform.transform.rotation.x = msg->markers[0].pose.pose.orientation.x;
  marker_transform.transform.rotation.y = msg->markers[0].pose.pose.orientation.y;
  marker_transform.transform.rotation.z = msg->markers[0].pose.pose.orientation.z;
  marker_transform.transform.rotation.w = msg->markers[0].pose.pose.orientation.w;
  // br.sendTransform(marker_transform);

  object_transform.header.stamp = ros::Time::now();
  object_transform.header.frame_id = "marker";
  object_transform.child_frame_id = "object";
  object_transform.transform.translation.x = 0.00;
  object_transform.transform.translation.y = -0.036;
  object_transform.transform.translation.z = 0.00;

  tf2::Quaternion q;
  q.setRPY(-M_PI/2.0, 0.0, -M_PI/2.0);
  object_transform.transform.rotation.x = q.x();
  object_transform.transform.rotation.y = q.y();
  object_transform.transform.rotation.z = q.z();
  object_transform.transform.rotation.w = q.w();

  receive_data = true;

  // ROS_INFO("update Marker pose");
}

void publish_object(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface) {
  moveit_msgs::CollisionObject object;

  object.id = "object";
  object.header.frame_id = "object";

  object.primitives.resize(1);
  object.primitives[0].type = object.primitives[0].BOX;
  object.primitives[0].dimensions.resize(3);
  object.primitives[0].dimensions[0] = 0.07;
  object.primitives[0].dimensions[1] = 0.07;
  object.primitives[0].dimensions[2] = 0.1;

  object.primitive_poses.resize(1);
  object.primitive_poses[0].position.x = 0.00;
  object.primitive_poses[0].position.y = 0.00;
  object.primitive_poses[0].position.z = 0.00;
  object.primitive_poses[0].orientation.w = 1.0;

  object.operation = object.ADD;
  planning_scene_interface.applyCollisionObject(object);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "markder_detect_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::WallDuration(1.0).sleep();

  ros::Subscriber marker_pose_sub = nh.subscribe("aruco_marker_publisher/markers", 1000, MarkerPoseCallback);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ROS_INFO("Wait for first marker...");
  while(receive_data == false) {
    ros::WallDuration(1.0).sleep();
  }

  ROS_INFO("Start object tf publish...");
  tf2_ros::TransformBroadcaster br;
  while(ros::ok()) {
    br.sendTransform(marker_transform);
    br.sendTransform(object_transform);
    publish_object(planning_scene_interface);
  }

  ROS_INFO("Stop object tf publish");
  ros::waitForShutdown();
  return 0;
}
