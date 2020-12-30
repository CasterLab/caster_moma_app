// ROS
#include <ros/ros.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>


// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#define random(x) rand()%(x)
// using namespace std;

void openGripper(trajectory_msgs::JointTrajectory& posture) {
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "j2n6s200_joint_finger_1";
  posture.joint_names[1] = "j2n6s200_joint_finger_2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.03;
  posture.points[0].positions[1] = 0.03;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture) {
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "j2n6s200_joint_finger_1";
  posture.joint_names[1] = "j2n6s200_joint_finger_2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.7;
  posture.points[0].positions[1] = 0.7;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group) {
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
  // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
  // transform from `"panda_link8"` to the palm of the end effector.
  grasps[0].grasp_pose.header.frame_id = "object";
  tf2::Quaternion orientation;
  orientation.setRPY(M_PI / 2, 0, M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.0;
  grasps[0].grasp_pose.pose.position.y = 0.0;
  grasps[0].grasp_pose.pose.position.z = 0.0;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "object";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.1;
  grasps[0].pre_grasp_approach.min_distance = 0.01;
  grasps[0].pre_grasp_approach.desired_distance = 0.02;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "object";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.x = -1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.01;
  grasps[0].post_grasp_retreat.desired_distance = 0.05;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  // move_group.setSupportSurfaceName("table");
  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group) {
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in verbose mode." This is a known issue. |br|
  // |br|
  // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
  // a single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "base_footprint";
  tf2::Quaternion orientation;
  orientation.setRPY(0,0,0);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* For place location, we set the value to the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0.8;
  place_location[0].place_pose.pose.position.y = -0.35;
  place_location[0].place_pose.pose.position.z = 0.8;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "base_footprint";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.01;
  place_location[0].pre_place_approach.desired_distance = 0.02;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "base_footprint";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.x = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.01;
  place_location[0].post_place_retreat.desired_distance = 0.05;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL
}

void addCollisionTables(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 2 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table";
  collision_objects[0].header.frame_id = "base_footprint";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.7;
  collision_objects[0].primitives[0].dimensions[1] = 1.7;
  collision_objects[0].primitives[0].dimensions[2] = 0.1;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 1.0;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.7;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  // collision_objects[1].header.frame_id = "base_footprint";
  // collision_objects[1].id = "table2";

  // /* Define the primitive and its dimensions. */
  // collision_objects[1].primitives.resize(1);
  // collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  // collision_objects[1].primitives[0].dimensions.resize(3);
  // collision_objects[1].primitives[0].dimensions[0] = 0.7;
  // collision_objects[1].primitives[0].dimensions[1] = 0.7;
  // collision_objects[1].primitives[0].dimensions[2] = 0.1;

  // /* Define the pose of the object. */
  // collision_objects[1].primitive_poses.resize(1);
  // collision_objects[1].primitive_poses[0].position.x = 0.3;
  // collision_objects[1].primitive_poses[0].position.y = -0.8;
  // collision_objects[1].primitive_poses[0].position.z = 0.7;
  // collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  // // END_SUB_TUTORIAL

  // collision_objects[1].operation = collision_objects[1].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 2 collision objects.
  std::srand((int)time(0));
  float scale_x = std::random(100)*0.01;
  float scale_y = std::random(100)*0.01;
  float scale_z = std::random(100)*0.01;
  ros::WallDuration(1.0).sleep();

  ROS_INFO("%f %f %f", scale_x, scale_y, scale_z);
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[0].header.frame_id = "base_footprint";
  collision_objects[0].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.06;
  collision_objects[0].primitives[0].dimensions[1] = 0.06;
  collision_objects[0].primitives[0].dimensions[2] = 0.1;

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.7;
  collision_objects[0].primitive_poses[0].position.y = 0.36*scale_y-0.18;
  collision_objects[0].primitive_poses[0].position.z = 0.8;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "fake_pick_place_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("arm");
  group.setPlanningTime(45.0);
  // moveit_msgs::AttachedCollisionObject attached_object;
  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.object.id = "object";
  // detach_object.link_name = "panda_link8";
  detach_object.object.operation = detach_object.object.REMOVE;
  moveit_msgs::PlanningScene planning_scene;


  // shapes::Mesh *mesh=shapes::createMeshFromResource("package:://myrobot_description/meshes/test.stl" );
  // shapes::Mesh *mesh=shapes::createMeshFromResource("package://caster_description/mesh/body_v3/z_movable_structure_up_v3-urdf.stl");
  // if(mesh==NULL) {
  //   ROS_INFO("search stl failed");
  // }
  // shapes::ShapeMsg shape_msg;
  // shapes::constructMsgFromShape(mesh, shape_msg);
  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.resize(1);

  // moveit_msgs::CollisionObject obj;
  // collision_objects[0].header.frame_id = "base_footprint";
  // collision_objects[0].id="object";
  // //定义物体形状尺寸

  // //定义物体方位
  // geometry_msgs::Pose pose;
  // pose.orientation.x =1.0;
  // pose.orientation.y =1.0;
  // pose.orientation.z =1.0;
  // pose.orientation.w =1.0;
  // //将形状添加到obj
  // collision_objects[0].meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
  // collision_objects[0].mesh_poses.push_back(pose);

  // //定义操作为添加
  // collision_objects[0].operation = collision_objects[0].ADD;
  // //定义一个PlanningScene消息
  // // moveit_msgs::PlanningScene planning_scene;
  // // planning_scene.world.collision_objects.push_back(collision_objects[0]);
  // // planning_scene.is_diff = true;
  // // //发布该消息
  // // planning_scene_diff_publisher.publish(planning_scene);
  // ROS_INFO("finished");
  // planning_scene_interface.applyCollisionObjects(collision_objects);

////////////

  // moveit_msgs::JointConstraint jcm;  
  // jcm.joint_name = "j2n6s200_joint_1";  
  // jcm.position = 0.0;
  // jcm.tolerance_above = 1.7;
  // jcm.tolerance_below = 1.7;
  // jcm.weight = 1.0;

  // 把它置为路径约束
  // moveit_msgs::Constraints test_constraints;
  // test_constraints.joint_constraints.push_back(jcm);  
  // group.setPathConstraints(test_constraints);

//物体形状与大小，中心为（0，0，0）
//   shape_msgs::SolidPrimitive shape;
//   shape.type = 1;
//   shape.dimensions.push_back(0.5);
//   shape.dimensions.push_back(1.0);
//   shape.dimensions.push_back(1.5);
// //物体中心点
//   geometry_msgs::Pose pose_constra;
//   pose_constra.position.x = 0.25;
//   pose_constra.position.y = 0.0;
//   pose_constra.position.z = 0.75;

//   moveit_msgs::PositionConstraint jcm;  
//   jcm.header.frame_id = "base_footprint";  
//   jcm.link_name = "j2n6s200_end_effector";  
//   jcm.constraint_region.primitives.push_back(shape);
//   jcm.constraint_region.primitive_poses.push_back(pose_constra);
//   jcm.weight = 1.0;

//   ROS_INFO("%i", jcm.constraint_region.primitives[0].type);
//   //把它置为路径约束
//   moveit_msgs::Constraints test_constraints;
//   test_constraints.position_constraints.push_back(jcm);  
//   group.setPathConstraints(test_constraints);


  // moveit_msgs::OrientationConstraint ocm;  
  // ocm.link_name = "j2n6s200_end_effector";  
  // ocm.header.frame_id = "j2n6s200_link_base";
  // ocm.orientation.x = 0.5;
  // ocm.orientation.y = 0.5;
  // ocm.orientation.z = 0.5;
  // ocm.orientation.w = 0.5;
  // ocm.absolute_x_axis_tolerance = 1.0;
  // ocm.absolute_y_axis_tolerance = 1.0;
  // ocm.absolute_z_axis_tolerance = 1.0;
  // ocm.weight = 1.0;

  // //把它置为路径约束
  // moveit_msgs::Constraints test_constraints;
  // test_constraints.orientation_constraints.push_back(ocm);  
  // group.setPathConstraints(test_constraints);


  // std::vector<std::string> s = group.getKnownConstraints ();
  // ROS_INFO("%i", s.size());
  // ROS_INFO("%s", s);
  addCollisionTables(planning_scene_interface);
  while(ros::ok()) {
    addCollisionObjects(planning_scene_interface);

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    // group.setNamedTarget("ready");
    // group.move();

    // ros::WallDuration(1.0).sleep();
    // ROS_INFO("pick");
    // pick(group);

    // ros::WallDuration(1.0).sleep();

    group.setNamedTarget("ready");
    // group.move();
    group.asyncMove();
    group.stop(); 
    ros::WallDuration(1.0).sleep();

    // ROS_INFO("place");
    // place(group);

    // ros::WallDuration(1.0).sleep();

    // group.setNamedTarget("ready");
    // group.move();

    ROS_INFO("Detaching the object from the robot and returning it to the world.");
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
    planning_scene.robot_state.is_diff = true;
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(detach_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
 
  }
  ros::waitForShutdown();
  return 0;
}