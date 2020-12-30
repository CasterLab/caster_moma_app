#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <map>
#include <string>
#include <thread>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <caster_moma_app/PickGift.h>
#include <caster_moma_app/PickItemsAction.h>
#include <kinova_msgs/Start.h>
#include <kinova_msgs/Stop.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <aruco_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pan_tilt_msgs/PanTiltCmd.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

typedef actionlib::ActionServer<caster_moma_app::PickItemsAction> Server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> FingerClient;
typedef actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> JointClient;
typedef actionlib::ServerGoalHandle<caster_moma_app::PickItemsAction> GoalHandle;

bool move_goal_flag;

void MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  ROS_INFO_STREAM("Move base feedback callback");
}

void MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  move_goal_flag = true;
  ROS_INFO_STREAM("Move base done callback");
}

class PickTaskAction
{
public:

  ros::NodeHandle nh_;
  Server as_;
  FingerClient fc_;
  JointClient jc_;
  GoalHandle goal_handle_;
  MoveBaseClient move_base_client_;
  
  std::string action_name_;

  caster_moma_app::PickItemsFeedback feedback_;
  caster_moma_app::PickItemsResult result_;

	double body_height, height_goal_, gripper_position_, object_distance_;
	int get_new_target, obstacle_id_, object_id_, step, box_count_;
	bool find_table_ = true, find_object_, task_active_ = false, get_goal, away_cabine_, marker_nav, get_object_distanse_ = false;
	std::map<std::string, std::vector<double>> mm;

	ros::ServiceClient stop_client, start_client, clear_costmaps_client_;
	ros::Publisher body_pub, head_pub, planning_scene_diff_publisher, base_cmd_pub;
	ros::Subscriber joint_states_sub, marker_pose_sub, table_pose_sub;
	
	// pan_tilt_msgs::PanTiltCmd pan_tilt_msgs;
	aruco_msgs::MarkerArray marker_array_msg_;

  std::vector<double> arm_standby_pose_, arm_box_position_, arm_box_size_;
  std::vector<std::vector<double>> arm_box_pose_, pose_vector_, orientation_vector_;

  geometry_msgs::Pose standby_position_, pick_position_, place_position_, table_pose_, object_pose_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm_group;
  moveit::planning_interface::MoveGroupInterface gripper_group;

  PickTaskAction(std::string name) : 
    as_(nh_, name, boost::bind(&PickTaskAction::ExecuteCb, this, _1), boost::bind(&PickTaskAction::preemptCB, this, _1), false),
		move_base_client_("move_base", true),
		fc_("j2s6s200_driver/fingers_action/finger_positions", true),
    jc_("j2s6s200_driver/joints_action/joint_angles", true),
    arm_group("arm"),
    gripper_group("gripper")
  {
	  marker_pose_sub = nh_.subscribe<aruco_msgs::MarkerArray>("object_marker_publisher/markers", 1000, &PickTaskAction::MarkerPoseCallback, this);
	  table_pose_sub = nh_.subscribe<visualization_msgs::Marker>("plane_marker_publisher/marker", 1000, &PickTaskAction::TablePoseCallback, this);
	  // ros::Subscriber marker_pose_sub = nh.subscribe<aruco_msgs::MarkerArray>("aruco_marker_publisher/markers", 1000, boost::bind(MarkerPoseCallback, _1, boost::ref(planning_scene_interface)));
	  planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	  base_cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	  clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");

	  joint_states_sub = nh_.subscribe("joint_states", 1000, &PickTaskAction::JointStatesCB, this);
	  body_pub = nh_.advertise<std_msgs::Float64>("caster/body_controller/command", 1000);
	  head_pub = nh_.advertise<pan_tilt_msgs::PanTiltCmd>("pan_tilt_driver_node/pan_tilt_cmd", 1000);
		stop_client = nh_.serviceClient<kinova_msgs::Stop>("/j2s6s200_driver/in/stop");
		start_client = nh_.serviceClient<kinova_msgs::Start>("/jns6s200_driver/in/start");

	  arm_group.setPlanningTime(45.0);

		GetParam();
    as_.start();
  }

  ~PickTaskAction(void)
  {
  }

  // void MarkerPoseCallback(const aruco_msgs::MarkerArrayConstPtr &msg) {
  // 	// marker_array_msg_ = *msg;
  // 	for (int i = 0; i < msg->markers.size(); ++i)
  // 	{
  // 		if (msg->markers[i].id == 9) {
  // 			table_pose_ = msg->markers[i].pose.pose;
  // 			find_table_ = true;
  // 		}
  		
  // 		if (msg->markers[i].id == object_id_) {
  // 			object_pose_ = msg->markers[i].pose.pose;
  // 			find_object_ = true;
  // 		}
  // 	}
  // }

  void MarkerPoseCallback(const aruco_msgs::MarkerArrayConstPtr &msg) {
  	// marker_array_msg_ = *msg;
  	// for (int i = 0; i < msg->markers.size(); ++i)
  	// {
  	// 	if (msg->markers[i].id == 9) {
  	// 		table_pose_ = msg->markers[i].pose.pose;
  	// 		find_table_ = true;
  	// 	}
  		
  	// 	if (msg->markers[i].id == object_id_) {
  	// 		object_pose_ = msg->markers[i].pose.pose;
  	// 		find_object_ = true;
  	// 	}
  	// }
		// ROS_INFO("%f", msg->markers[0].pose.pose.position.z);
  	if (!get_object_distanse_)
  	{
			object_distance_ = msg->markers[0].pose.pose.position.z;
			ROS_INFO("%f", object_distance_);
			get_object_distanse_ = true;
  	}
  }
// 0.307

  void TablePoseCallback(const visualization_msgs::MarkerConstPtr &msg) {
  	// marker_array_msg_ = *msg;

		// geometry_msgs::TransformStamped marker_transform, object_transform;
  	if (!find_table_)
  	{
			table_pose_ = msg->pose;
			find_table_ = true;
  	}


		// if ((msg->pose.position.z) >= 1.1)
		// {
		// 	geometry_msgs::Twist cmd_msg;
		// 	cmd_msg.linear.x = 0.1;
		// 	base_cmd_pub.publish(cmd_msg);
		// }
		if ((msg->pose.position.z) <= 1.05 and !get_goal)
		{
			get_goal = true;
		}

		if ((msg->pose.position.z) >= 1.4 and !get_goal)
		{
			away_cabine_ = true;
		}

	  // marker_transform.header.stamp = ros::Time::now();
	  // marker_transform.header.frame_id = msg->header.frame_id;
	  // marker_transform.child_frame_id = "marker_table";

	  // marker_transform.transform.translation.x = msg->pose.position.x;
	  // marker_transform.transform.translation.y = msg->pose.position.y;
	  // marker_transform.transform.translation.z = msg->pose.position.z;
	  // marker_transform.transform.rotation.x = msg->pose.orientation.x;
	  // marker_transform.transform.rotation.y = msg->pose.orientation.y;
	  // marker_transform.transform.rotation.z = msg->pose.orientation.z;
	  // marker_transform.transform.rotation.w = msg->pose.orientation.w;

	  // object_transform.header.stamp = ros::Time::now();
	  // object_transform.header.frame_id = "marker_table";
	  // object_transform.child_frame_id = "table";

	  // object_transform.transform.translation.x = 0.02;
	  // object_transform.transform.translation.y = -0.036;
	  // object_transform.transform.translation.z = 0.00;

	  // tf2::Quaternion q;
	  // q.setRPY(-M_PI/2.0, M_PI, -M_PI/2.0);
	  // object_transform.transform.rotation.x = q.x();
	  // object_transform.transform.rotation.y = q.y();
	  // object_transform.transform.rotation.z = q.z();
	  // object_transform.transform.rotation.w = q.w();
	  // tf2_ros::TransformBroadcaster br;
   //  br.sendTransform(marker_transform);
   //  br.sendTransform(object_transform);
    // ROS_INFO("table pose publishing");
  }

  void ExcuteThread(GoalHandle gh) {
		PickAndPlace(gh);
  }

	void ExecuteCb(GoalHandle gh)
	{
		ROS_INFO("````````````````````````````````````````");
		if (task_active_)
		{
			gh.setRejected();
			return;
		}
		gh.setAccepted();
		task_active_ = true;
		ROS_INFO("%c", gh.getGoal()->items);
	  ROS_INFO("call pick task server");
		// UpdateCabinet("table");
		// GetParam();
		// object_id_ = goal->items;
		// UpdateTable();
		// UpdateBox(88);

		std::thread thread(&PickTaskAction::ExcuteThread, this, gh);
		thread.detach();
		task_active_ = false;
	}

  void preemptCB(GoalHandle gh)
  {
    
    // stop kinova API driver
    kinova_msgs::Stop stop_msg;
    kinova_msgs::Start start_msg;
    stop_client.call(stop_msg);
    start_client.call(start_msg);
    
		// cancel move_base goal
    get_goal = true;		
    move_base_client_.cancelAllGoals();

    // set the action state to preempted
    gh.setCanceled();
    task_active_ = false;
  }

  void GetParam() {

	  // get arm standby pose
	  arm_standby_pose_.resize(6);
	  nh_.getParam("arm_pose/standby", arm_standby_pose_);

	  // ROS_INFO("%f", arm_box_pose_[0][0]);

	  // get navigation position 
  }

  geometry_msgs::Pose GetGoalPose(std::string goal_name) {

    std::vector<float> pose_vector(3), orientation_vector(4);
    nh_.getParam("target_goal/"+goal_name+"/pose", pose_vector);
    nh_.getParam("target_goal/"+goal_name+"/orientation", orientation_vector);

    geometry_msgs::Pose pose;
    pose.position.x = pose_vector[0];
    pose.position.y = pose_vector[1];
    pose.position.z = pose_vector[2];
    pose.orientation.x = orientation_vector[0];
    pose.orientation.y = orientation_vector[1];
    pose.orientation.z = orientation_vector[2];
    pose.orientation.w = orientation_vector[3];

    return pose;
  }

  void PickAndPlace(GoalHandle gh){
  	ROS_INFO("pick and place");
  	geometry_msgs::Pose standy_position = GetGoalPose("standby");
  	geometry_msgs::Pose pick_position = GetGoalPose("pick");
  	geometry_msgs::Pose place_position = GetGoalPose("place_1");

	  while(ros::ok()) {
      if (!MoveToGoal(pick_position, gh))
      {
        return;
      }
      ros::Rate loop_rate(10);
      for (int i = 0; i < 50; ++i)
      {
       geometry_msgs::Twist cmd_msg_back;
       cmd_msg_back.angular.z = -0.4;
       base_cmd_pub.publish(cmd_msg_back);
       loop_rate.sleep();
      }
      if (!MoveToGoal(place_position, gh))
      {
        return;
      }
      for (int i = 0; i < 80; ++i)
      {
       geometry_msgs::Twist cmd_msg_back;
       cmd_msg_back.angular.z = -0.4;
       base_cmd_pub.publish(cmd_msg_back);
       loop_rate.sleep();
      }
      if (!MoveToGoal(standy_position, gh))
      {
        return;
      }
	  	// ros::spinOnce();
	  }
  }

  //Set joint angle target,topic of "/joint_states"
	void SetArmJointPose(std::vector<double>& joint_value) {
	  arm_group.setJointValueTarget(joint_value);
	}

  //Set pose target,send pose msg to Moveit
	void SetArmTargetPose(geometry_msgs::PoseStamped target) {
	  arm_group.setPoseTarget(target);

	}

	//Set body height
	bool SetHeight(double height) {
	  if(height < 0.0) {
	    height = 0;
	  } else if (height > 0.4) {
	    height = 0.4;
	  }

	  ROS_INFO("%lf, %lf", height, body_height);

	  std_msgs::Float64 msg;
	  msg.data = height;
	  body_pub.publish(msg);

	  while(abs(body_height-height) > 0.01) {
	    ros::WallDuration(0.2).sleep();
	  }
	}

	// Using Moveit control grippers 
	void SetGripper(double box_size) {
	  std::vector<double> gripper_pose(2);

	  gripper_pose[0] = box_size*0.2;
	  gripper_pose[1] = box_size*0.2;
	  // gripper_pose[2] = value;

	  gripper_group.setJointValueTarget(gripper_pose);
	  // move_group.move();
	}
 
	// Using Moveit move arm and check execution
	bool ArmMoveAndCheck(moveit::planning_interface::MoveGroupInterface& move_group, actionlib::SimpleActionClient< moveit_msgs::MoveGroupAction > &arm_group_action, GoalHandle gh) {
	  arm_group.move();
	  if (arm_group_action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
	      ROS_INFO("SUCCEEDED");
  		  return true;
  	}
	  else {
				gh.publishFeedback(feedback_);
        if (gh.getGoalStatus().status == 1)
        {
				  gh.setAborted();
        }
		    task_active_ = false;
        return false;
	      // ROS_INFO("Cancel Goal!");
	      // arm_group_action.cancelAllGoals();
	  }
	}

	// void SetArmNamePose(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double>& joint_value) {
	//   move_group.setJointValueTarget(joint_value);
	//   move_group.move();
	// }

	// Using move_base control caster for navigation
	bool MoveToGoal(geometry_msgs::Pose pose, GoalHandle gh) {

		std_srvs::Empty clear_task;
		clear_costmaps_client_.call(clear_task);

	  move_base_msgs::MoveBaseGoal mb_goal;
	  mb_goal.target_pose.header.stamp = ros::Time::now();
	  mb_goal.target_pose.header.frame_id = "map";
	  mb_goal.target_pose.pose = pose;
	  move_goal_flag = false;
	  move_base_client_.sendGoal(mb_goal,
	            boost::bind(&MovebaseDoneCallback, _1, _2),
	            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
	            boost::bind(&MovebaseFeedbackCallback, _1));

	  while(move_goal_flag==false) {
	    ros::WallDuration(1.0).sleep();
	    ROS_INFO_STREAM("moving...");
	  }

    if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			  ROS_INFO_STREAM("get goal");
        // ROS_INFO("SUCCEEDED");
  		  return true;
  	}
    else {
        ROS_INFO("Cancel Goal!");
        move_base_client_.cancelAllGoals();
        if (gh.getGoalStatus().status == 1)
        {
				  gh.setAborted();
        }
		    task_active_ = false;
    		return false;
    }
	}

	// Add cabinet collision
	void UpdateCabinet(std::string place_name) {

	 //  std::string collision_path = "package://caster_moma_app/collision_meshes/cabinet-urdf.stl";
	 //  shapes::Mesh *mesh=shapes::createMeshFromResource(collision_path);
	 //  if(mesh==NULL) {
	 //    ROS_INFO("search stl failed");
	 //  }
	 //  shapes::ShapeMsg shape_msg;
	 //  shapes::constructMsgFromShape(mesh, shape_msg);
	 //  std::vector<moveit_msgs::CollisionObject> collision_objects;
	 //  collision_objects.resize(1);

	 //  moveit_msgs::CollisionObject obj;

	 //  collision_objects[0].header.frame_id = "table";
	 //  collision_objects[0].id="cabinet_collision";

	 //  //定义物体方位
	 //  geometry_msgs::Pose pose;
	 //  // pose.position.z = -1.14;
	 //  pose.position.z = -1.23;
	 //  pose.position.x = 0.21;

		// // tf2::Quaternion q;
  // //   q.setRPY(M_PI/2.0, 0.0, M_PI/2.0);
  // //   pose.orientation.x = q.x();
  // //   pose.orientation.y = q.y();
  // //   pose.orientation.z = q.z();
  // //   pose.orientation.w = q.w();

	 //  // pose.position.x = 0.0;
	 //  // pose.position.y = table_pose_.position.y;
	 //  // pose.position.z = table_pose_.position.z;
	 //  // pose.orientation.x = table_pose_.orientation.x;
	 //  // pose.orientation.y = table_pose_.orientation.y;
	 //  // pose.orientation.z = table_pose_.orientation.z;
	 //  // pose.orientation.w = table_pose_.orientation.w;

	 //  //将形状添加到obj
	 //  collision_objects[0].meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
	 //  collision_objects[0].mesh_poses.push_back(pose);

	 //  //定义操作为添加
	 //  collision_objects[0].operation = collision_objects[0].ADD;
	 //  //定义一个PlanningScene消息
	 //  // moveit_msgs::PlanningScene planning_scene;
	 //  // planning_scene.world.collision_objects.push_back(collision_objects[0]);
	 //  // planning_scene.is_diff = true;
	 //  // //发布该消息
	 //  // planning_scene_diff_publisher.publish(planning_scene);
	 //  planning_scene_interface.applyCollisionObjects(collision_objects);
	 //  ROS_INFO("add collision finished");


	  std::vector<moveit_msgs::CollisionObject> collision_objects;
	  collision_objects.resize(2);

	  collision_objects[0].header.frame_id = "base_footprint";
	  collision_objects[0].id = "cabinet_collision";

	  // define the primitive and its dimensions. 
	  collision_objects[0].primitives.resize(1);
	  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
	  collision_objects[0].primitives[0].dimensions.resize(3);
	  collision_objects[0].primitives[0].dimensions[0] = 0.06;
	  collision_objects[0].primitives[0].dimensions[1] = 1.2;
	  collision_objects[0].primitives[0].dimensions[2] = 1.8;

	  // define the pose of the object
	  collision_objects[0].primitive_poses.resize(1);
	  // tf2::Quaternion q;

	  collision_objects[0].primitive_poses[0].position.x = table_pose_.position.z - 0.029 - 0.2;
	  collision_objects[0].primitive_poses[0].position.y = table_pose_.position.y - 0.042;
	  collision_objects[0].primitive_poses[0].position.z = table_pose_.position.x + 1.132 - 0.2;
	  // collision_objects[0].primitive_poses[0].orientation.x = q.x();
	  // collision_objects[0].primitive_poses[0].orientation.y = q.y();
	  // collision_objects[0].primitive_poses[0].orientation.z = q.z();
	  // collision_objects[0].primitive_poses[0].orientation.w = q.w();
	  // add object to planning scene
	  collision_objects[1].header.frame_id = "base_footprint";
	  collision_objects[1].id = "cabinet_detail_collision";

	  collision_objects[1].primitives.resize(3);
	  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
	  collision_objects[1].primitives[0].dimensions.resize(3);
	  collision_objects[1].primitives[0].dimensions[1] = 1.1;
	  collision_objects[1].primitives[0].dimensions[0] = 0.52;
	  collision_objects[1].primitives[0].dimensions[2] = 0.06;
	  collision_objects[1].primitives[2].type = collision_objects[1].primitives[2].BOX;
	  collision_objects[1].primitives[2].dimensions.resize(3);
	  collision_objects[1].primitives[2].dimensions[1] = 1.1;
	  collision_objects[1].primitives[2].dimensions[0] = 0.52;
	  collision_objects[1].primitives[2].dimensions[2] = 0.06;
	  collision_objects[1].primitives[1].type = collision_objects[1].primitives[1].BOX;
	  collision_objects[1].primitives[1].dimensions.resize(3);
	  collision_objects[1].primitives[1].dimensions[1] = 1.1;
	  collision_objects[1].primitives[1].dimensions[0] = 0.52;
	  collision_objects[1].primitives[1].dimensions[2] = 0.06;

	  // define the pose of the object
	  collision_objects[1].primitive_poses.resize(3);
	  collision_objects[1].primitive_poses[0].position.x = table_pose_.position.z- 0.029 + 0.05;
	  collision_objects[1].primitive_poses[0].position.y = table_pose_.position.y - 0.042;
	  // collision_objects[1].primitive_poses[0].position.z = table_pose_.position.z + 0.05;
	  collision_objects[1].primitive_poses[0].position.z = -table_pose_.position.x + 0.115 + 1.133;
	  // collision_objects[1].primitive_poses[0].orientation.z = table_pose_.position.x + 0.115 + 1.132 - 0.05;

	  collision_objects[1].primitive_poses[2].position.x = table_pose_.position.z- 0.029 + 0.05;
	  collision_objects[1].primitive_poses[2].position.y = table_pose_.position.y - 0.042;
	  collision_objects[1].primitive_poses[2].position.z = -table_pose_.position.x  -0.267 + 1.133;
	  // collision_objects[1].primitive_poses[2].orientation.z = table_pose_.position.x  -0.267 + 1.132  - 0.05;
	  // collision_objects[1].primitive_poses[2].position.z = table_pose_.position.z + 0.05;
	  // collision_objects[1].primitive_poses[2].position.z = table_pose_.position.z -0.267;

	  collision_objects[1].primitive_poses[1].position.x = table_pose_.position.z- 0.029 + 0.05;
	  collision_objects[1].primitive_poses[1].position.y = table_pose_.position.y - 0.042;
	  collision_objects[1].primitive_poses[1].position.z = -table_pose_.position.x - 0.652 + 1.133; 
	  // collision_objects[1].primitive_poses[1].orientation.z = table_pose_.position.x - 0.672 + 1.132 - 0.05; 
	  // collision_objects[1].primitive_poses[1].position.z = table_pose_.position.z + 0.05;

	  collision_objects[0].operation = collision_objects[0].ADD;
	  collision_objects[1].operation = collision_objects[1].ADD;
	  planning_scene_interface.applyCollisionObjects(collision_objects);

	}

	// Add box collision on object link
	void UpdateBox(int id) {
	  std::vector<moveit_msgs::CollisionObject> collision_objects;
	  collision_objects.resize(2);

	  collision_objects[0].header.frame_id = "object" + std::to_string(id);
	  collision_objects[0].id = "box_collision";

	  // define the primitive and its dimensions. 
	  collision_objects[0].primitives.resize(3);
	  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
	  collision_objects[0].primitives[0].dimensions.resize(3);
	  collision_objects[0].primitives[0].dimensions[1] = 0.01;
	  collision_objects[0].primitives[0].dimensions[0] = 1.1;
	  collision_objects[0].primitives[0].dimensions[2] = 0.32;
	  collision_objects[0].primitives[2].type = collision_objects[0].primitives[0].BOX;
	  collision_objects[0].primitives[2].dimensions.resize(3);
	  collision_objects[0].primitives[2].dimensions[1] = 0.01;
	  collision_objects[0].primitives[2].dimensions[0] = 1.1;
	  collision_objects[0].primitives[2].dimensions[2] = 0.32;
	  collision_objects[0].primitives[1].type = collision_objects[0].primitives[1].BOX;
	  collision_objects[0].primitives[1].dimensions.resize(3);
	  collision_objects[0].primitives[1].dimensions[1] = 0.82;
	  collision_objects[0].primitives[1].dimensions[0] = 0.7;
	  collision_objects[0].primitives[1].dimensions[2] = 0.01;

	  // define the pose of the object
	  collision_objects[0].primitive_poses.resize(3);
	  collision_objects[0].primitive_poses[0].position.x = -0.35;
	  collision_objects[0].primitive_poses[0].position.y = -0.07;
	  collision_objects[0].primitive_poses[0].position.z = 0.01;

	  collision_objects[0].primitive_poses[2].position.x = -0.35;
	  collision_objects[0].primitive_poses[2].position.y = 0.33;
	  collision_objects[0].primitive_poses[2].position.z = 0.01;

	  collision_objects[0].primitive_poses[1].position.x = -0.53;
	  collision_objects[0].primitive_poses[1].position.y = -0.07;
	  collision_objects[0].primitive_poses[1].position.z = -0.16;

	  // add object to planning scene
	  collision_objects[0].operation = collision_objects[0].ADD;
	  planning_scene_interface.applyCollisionObjects(collision_objects);
	}

	// Add table collision
	void UpdateTable() {
	  std::vector<moveit_msgs::CollisionObject> collision_objects;
	  collision_objects.resize(1);

	  collision_objects[0].header.frame_id = "table2";
	  collision_objects[0].id = "table_collision";

	  // define the primitive and its dimensions. 
	  collision_objects[0].primitives.resize(1);
	  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
	  collision_objects[0].primitives[0].dimensions.resize(3);
	  collision_objects[0].primitives[0].dimensions[0] = 1.1;
	  collision_objects[0].primitives[0].dimensions[1] = 2.0;
	  collision_objects[0].primitives[0].dimensions[2] = 0.1;

	  // define the pose of the object
	  collision_objects[0].primitive_poses.resize(1);
	  tf2::Quaternion q;
    q.setRPY(M_PI/2.0, M_PI/2.0, 0.0);

	  collision_objects[0].primitive_poses[0].position.x = -0.06;
	  collision_objects[0].primitive_poses[0].position.y = -0.05;
	  collision_objects[0].primitive_poses[0].position.z = 0.4;
	  collision_objects[0].primitive_poses[0].orientation.x = q.x();
	  collision_objects[0].primitive_poses[0].orientation.y = q.y();
	  collision_objects[0].primitive_poses[0].orientation.z = q.z();
	  collision_objects[0].primitive_poses[0].orientation.w = q.w();
	  // add object to planning scene
	  collision_objects[0].operation = collision_objects[0].ADD;
	  planning_scene_interface.applyCollisionObjects(collision_objects);
	}

	//Add object collision
	void UpdateObject(double box_size, int id) {
	  std::vector<moveit_msgs::CollisionObject> collision_objects;
	  collision_objects.resize(1);

	  collision_objects[0].header.frame_id = "object" + std::to_string(id);
	  collision_objects[0].id = "object_collision_" + std::to_string(id);

	  // define the primitive and its dimensions. 
	  collision_objects[0].primitives.resize(1);
	  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
	  collision_objects[0].primitives[0].dimensions.resize(3);
	  collision_objects[0].primitives[0].dimensions[0] = 0.05;
	  collision_objects[0].primitives[0].dimensions[1] = 0.06;
	  collision_objects[0].primitives[0].dimensions[2] = 0.05;

	  // define the pose of the object
	  collision_objects[0].primitive_poses.resize(1);

	  // add object to planning scene
	  collision_objects[0].operation = collision_objects[0].ADD;
	  planning_scene_interface.applyCollisionObjects(collision_objects);
	}

	// Delelt collision
	void DeleltCollsion(std::vector<std::string> object_ids) {
		planning_scene_interface.removeCollisionObjects(object_ids);
	}

	void JointStatesCB(const sensor_msgs::JointState::ConstPtr& msg) {
	  for(uint8_t i=0; i<msg->name.size(); i++) {
	    if(msg->name[i] == "caster_body_connected_joint") {
	      body_height = msg->position[i];
	      // ROS_INFO("Get height %lf", body_height);
	      break;
	    }
	  }
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_task_server");

  PickTaskAction averaging("pick_task");
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  // return 0;
}