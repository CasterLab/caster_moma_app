#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <aruco_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>

#include <moveit/planning_scene_interface/planning_scene_interface.h>


bool table_receive_data = false;
bool object_receive_data = false;
bool object_updated = false;
bool table_updated = false;

geometry_msgs::TransformStamped marker_transform, object_transform, table_transform, table_transform_down;
std::vector<geometry_msgs::TransformStamped> object_marker_transform, object_marker_goal_transform;
void ObjectMarkerPoseCallback(const aruco_msgs::MarkerArray::ConstPtr& msg) {
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped markers_transform;
  geometry_msgs::TransformStamped markers_goal_transform;
  for(uint16_t i=0; i<msg->markers.size(); i++) {
    // ROS_INFO("%i", msg->markers.size());
    // if (msg->markers[i].id == 88) {

    markers_transform.header.stamp = ros::Time::now();
    markers_transform.header.frame_id = msg->markers[i].header.frame_id;
    markers_transform.child_frame_id = "marker" + std::to_string(msg->markers[i].id);

    markers_transform.transform.translation.x = msg->markers[i].pose.pose.position.x;
    markers_transform.transform.translation.y = msg->markers[i].pose.pose.position.y;
    markers_transform.transform.translation.z = msg->markers[i].pose.pose.position.z;
    markers_transform.transform.rotation.x = msg->markers[i].pose.pose.orientation.x;
    markers_transform.transform.rotation.y = msg->markers[i].pose.pose.orientation.y;
    markers_transform.transform.rotation.z = msg->markers[i].pose.pose.orientation.z;
    markers_transform.transform.rotation.w = msg->markers[i].pose.pose.orientation.w;

    markers_goal_transform.header.stamp = ros::Time::now();
    markers_goal_transform.header.frame_id = "marker" + std::to_string(msg->markers[i].id);
    markers_goal_transform.child_frame_id = "object" + std::to_string(msg->markers[i].id);

    markers_goal_transform.transform.translation.x = 0.02;
    markers_goal_transform.transform.translation.y = -0.036;
    markers_goal_transform.transform.translation.z = 0.00;

    tf2::Quaternion q;
    // q.setRPY(-M_PI/2.0, -M_PI/2.0, 0.0);
    // q.setRPY(M_PI/2.0, 0.0, -M_PI/2.0);
    q.setRPY(M_PI/2.0, M_PI/2.0, 0.0);
    markers_goal_transform.transform.rotation.x = q.x();
    markers_goal_transform.transform.rotation.y = q.y();
    markers_goal_transform.transform.rotation.z = q.z();
    markers_goal_transform.transform.rotation.w = q.w();
    // object_marker_goal_transform.push_back(markers_goal_transform);
    // object_marker_transform.push_back(markers_transform);
    br.sendTransform(markers_transform);
    br.sendTransform(markers_goal_transform);
    // }
  }
  object_updated = true;
  object_receive_data = true;

  // ROS_INFO("update Marker pose");
}

void TablePoseCallback(const visualization_msgs::MarkerConstPtr &msg) {
  // table_pose_ = msg->pose;

  // Eigen::Quaterniond qua;
  // qua.x() = msg->pose.orientation.x;
  // qua.y() = msg->pose.orientation.y;
  // qua.z() = msg->pose.orientation.z;
  // qua.w() = msg->pose.orientation.w;

  // Eigen::Vector3d euler = qua.toRotationMatrix().eulerAngles(2, 1, 0);

  // ROS_INFO("%f", euler[2]);
  // ROS_INFO("%f", euler[1]);
  // ROS_INFO("%f", euler[0]);

  marker_transform.header.stamp = ros::Time::now();
  marker_transform.header.frame_id = msg->header.frame_id;
  marker_transform.child_frame_id = "marker_table";

  marker_transform.transform.translation.x = msg->pose.position.x;
  marker_transform.transform.translation.y = msg->pose.position.y;
  marker_transform.transform.translation.z = msg->pose.position.z;
  marker_transform.transform.rotation.x = msg->pose.orientation.x;
  marker_transform.transform.rotation.y = msg->pose.orientation.y;
  marker_transform.transform.rotation.z = msg->pose.orientation.z;
  marker_transform.transform.rotation.w = msg->pose.orientation.w;

  table_transform.header.stamp = ros::Time::now();
  table_transform.header.frame_id = "marker_table";
  // table_transform.header.frame_id = msg->header.frame_id;
  table_transform.child_frame_id = "table";
  // table_transform.transform.translation.x = msg->pose.position.x;
  // table_transform.transform.translation.y = msg->pose.position.y;
  // table_transform.transform.translation.z = msg->pose.position.z;
  // table_transform.transform.rotation.w = 1.0;
  // table_transform.transform.translation.x = 0.02;
  // table_transform.transform.translation.y = -0.036;
  // table_transform.transform.translation.z = 0.00;

  tf2::Quaternion q;
  q.setRPY(-M_PI/2.0, M_PI, -M_PI/2.0);
  // q.setRPY(M_PI/2.0, M_PI/2.0, 0.0);
  // q.setRPY(euler[0], 0.0, euler[2]);
  table_transform.transform.rotation.x = q.x();
  table_transform.transform.rotation.y = q.y();
  table_transform.transform.rotation.z = q.z();
  table_transform.transform.rotation.w = q.w();

  table_transform_down.header.stamp = ros::Time::now();
  table_transform_down.header.frame_id = "marker_table";
  table_transform_down.child_frame_id = "table2";
  tf2::Quaternion q2;
  q2.setRPY(0.0, M_PI/2.0, 0.0);
  table_transform_down.transform.rotation.x = q2.x();
  table_transform_down.transform.rotation.y = q2.y();
  table_transform_down.transform.rotation.z = q2.z();
  table_transform_down.transform.rotation.w = q2.w();
  // table_updated = true;
  // table_receive_data = true;
  tf2_ros::TransformBroadcaster br;
  br.sendTransform(marker_transform);
  br.sendTransform(table_transform);
  br.sendTransform(table_transform_down);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "markder_detect_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::WallDuration(1.0).sleep();

  ros::Subscriber object_markers_sub = nh.subscribe("object_marker_publisher/markers", 1000, ObjectMarkerPoseCallback);
  ros::Subscriber table_pose_sub = nh.subscribe<visualization_msgs::Marker>("plane_marker_publisher/marker", 1000, TablePoseCallback);

  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ROS_INFO("Wait for first marker...");
  while(object_receive_data == false and table_receive_data == true) {
    ros::WallDuration(1.0).sleep();
  }

  ROS_INFO("Start object tf publish...");
  tf2_ros::TransformBroadcaster br;

  ros::waitForShutdown();
  return 0;
}
