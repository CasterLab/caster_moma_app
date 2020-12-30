#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <thread>
#include <map>  
#include <xmlrpcpp/XmlRpcValue.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <caster_moma_app/IntroduceAction.h>
#include <sound_play/SoundRequestAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef actionlib::ActionServer<caster_moma_app::IntroduceAction> Server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::ServerGoalHandle<caster_moma_app::IntroduceAction> GoalHandle;
typedef actionlib::SimpleActionClient<sound_play::SoundRequestAction> SoundPlayClient;;

bool move_goal_flag, sound_play_flag;

void MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  // ROS_INFO_STREAM("Move base feedback callback");
}

void MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  move_goal_flag = true;
  // ROS_INFO_STREAM("Move base done callback");
}

void SoundplayDoneCallback(const actionlib::SimpleClientGoalState& state, const sound_play::SoundRequestResultConstPtr& result) {
  sound_play_flag = true;
  // ROS_INFO_STREAM("Sound play done callback");
}

void SoundplayFeedbackCallback(const sound_play::SoundRequestFeedbackConstPtr& feedback) {
  // ROS_INFO_STREAM("Sound play feedback callback");
}

class PickTaskAction
{
public:
  ros::NodeHandle nh_;
  Server as_;
  GoalHandle goal_handle_;
  MoveBaseClient move_base_client_;
  SoundPlayClient sound_play_client_;
  caster_moma_app::IntroduceFeedback feedback_;
  caster_moma_app::IntroduceResult result_;
  ros::Publisher base_cmd_pub;
	int step;
	bool task_active_;

  geometry_msgs::Pose standby_position_, pick_position_, place_position_;

  PickTaskAction(std::string name) : 
	  task_active_(false),
    as_(nh_, name, boost::bind(&PickTaskAction::ExecuteCb, this, _1), boost::bind(&PickTaskAction::preemptCB, this, _1), false),
		move_base_client_("move_base", true),
    sound_play_client_("sound_play", true)
  {
		// GetParam();
    base_cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    as_.start();
  }

  ~PickTaskAction(void)
  {
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
		  ROS_INFO("new goal rejected");
			return;
		}

		gh.setAccepted();
		task_active_ = true;
		ROS_INFO("%c", gh.getGoal()->places);
	  ROS_INFO("call pick task server");
	  // std::vector<float> pose_vector(3), orientation_vector(4);
	  // std::vector<float> pose_vector;
	  // nh_.getParam("guide_goal", pose_vector);
	  // ROS_INFO("%i", pose_vector.size());
	  // if (pose_vector.size() == 0)
	  // {
		 //  ROS_INFO("pose vector is NULL");
	  // }
		// GetParam();
		std::thread thread(&PickTaskAction::ExcuteThread, this, gh);
		thread.detach();
	}

  void preemptCB(GoalHandle gh)
  {
	  ROS_INFO("introduce server cancel!");
    
		// cancel move_base goal
    move_base_client_.cancelAllGoals();
    sound_play_client_.cancelAllGoals();

    // set the action state to preempted
    task_active_ = false;
    sound_play_flag = true;
    gh.setCanceled();
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

	bool SoundPlay(GoalHandle gh, std::string path) {

		sound_play::SoundRequestGoal sound_msg;
		sound_msg.sound_request.sound = -2;
		sound_msg.sound_request.volume = 1.0;
		// sound_msg.sound_request.volume = 0.5;
		sound_msg.sound_request.command = 1;
		// sound_msg.sound_request.arg = "/home/caster/sample_sounds/a2002011001-e02-ulaw" + std::to_string(index) + ".wav";
		sound_msg.sound_request.arg = path;

		sound_play_client_.sendGoal(sound_msg,
	    boost::bind(&SoundplayDoneCallback, _1, _2),
      actionlib::SimpleActionClient<sound_play::SoundRequestAction>::SimpleActiveCallback(),
      boost::bind(&SoundplayFeedbackCallback, _1)
			);
	  sound_play_flag = false;
	  while(sound_play_flag == false) {
	    ros::WallDuration(1.0).sleep();
			feedback_.step_description = "playing";
			feedback_.step_index = 3;
			gh.publishFeedback(feedback_);
	    // ROS_INFO_STREAM("playing...");
	    // ros::spinOnce();
	  }
		// sound_play_client_.waitForResult(ros::Duration(55.0)); 
    if (sound_play_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			  ROS_INFO_STREAM("get goal");
        // ROS_INFO("SUCCEEDED");
  		  return true;
  	}
    else {
        ROS_INFO("Cancel Goal!");
        sound_play_client_.cancelAllGoals();
        if (gh.getGoalStatus().status == 1)
        {
				  gh.setAborted();
        }
        task_active_ = false;
    		return false;
    }
	}

  void PickAndPlace(GoalHandle gh){

	  XmlRpc::XmlRpcValue pose_vector;
	  nh_.getParam("guide_goal", pose_vector);
	  ROS_INFO("%i", pose_vector.size());

	 //  std::vector<geometry_msgs::Pose> pose_vector_goal;
	 //  std::vector<std::string> vector_file_path;
		// for (int i = 0; i < pose_vector.size(); ++i)
		// {
		// 	geometry_msgs::Pose pose;
		// 	std::string file_path;
		//   pose.position.x = pose_vector[i]["pose"][0];
		//   pose.position.y = pose_vector[i]["pose"][1];
		//   pose.position.z = pose_vector[i]["pose"][2];
		//   pose.orientation.x = pose_vector[i]["orientation"]["pose"][0];
		//   pose.orientation.y = pose_vector[i]["orientation"]["pose"][1];
		//   pose.orientation.z = pose_vector[i]["orientation"]["pose"][2];
		//   pose.orientation.w = pose_vector[i]["orientation"]["pose"][3];
		//   file_path = pose_vector[i]["file_path"];
		//   pose_vector_goal.push_back(pose);
		//   vector_file_path.push_back(file_path);
		// }

	  geometry_msgs::Pose position_standy = GetGoalPose("standby");
  	for (int i = 0; i < pose_vector.size(); ++i)
  	{

  		std::string goal_current = static_cast<std::string>(pose_vector[i]["name"]);

			feedback_.step_description = "pending";
			feedback_.goal_current = goal_current;
			feedback_.step_index = 0;
			gh.publishFeedback(feedback_);

			geometry_msgs::Pose pose;
			// std::string file_path;
		  pose.position.x = pose_vector[i]["pose"][0];
		  pose.position.y = pose_vector[i]["pose"][1];
		  pose.position.z = pose_vector[i]["pose"][2];
		  pose.orientation.x = pose_vector[i]["orientation"][0];
		  pose.orientation.y = pose_vector[i]["orientation"][1];
		  pose.orientation.z = pose_vector[i]["orientation"][2];
		  pose.orientation.w = pose_vector[i]["orientation"][3];
		  std::string file_path = static_cast<std::string>(pose_vector[i]["file_path"]);
		  ROS_INFO("%s", file_path.c_str());
  		// std::string description = "Move to position " + places_array[i];
			int step = 1;
			bool loop = true;
		  while(loop) {

				if (step == 1) {
					/* code */
					// feedback_.goal_state = "approching";
					feedback_.step_description = "approching";
					feedback_.step_index = 1;
					gh.publishFeedback(feedback_);
				  ros::WallDuration(1.0).sleep();
				  ROS_INFO("Move to standby goal");
				  // ros::WallDuration(5.0).sleep();
				  if (i != 0)
				  {
			      if (!MoveToGoal(pose, gh))
			      {
				     	return;
			      }
				  }
					feedback_.step_description = "approched";
					feedback_.step_index = 2;
				}

				else if (step == 2)
				{
					gh.publishFeedback(feedback_);
				  ros::WallDuration(1.0).sleep();
			    // ROS_INFO("Introduce");
					feedback_.step_description = "playing";
					feedback_.step_index = 3;
					gh.publishFeedback(feedback_);
				  if (!SoundPlay(gh, file_path))
				  {
				  	return;
				  }
					feedback_.step_description = "played";
					feedback_.step_index = 4;
					if (i == pose_vector.size() - 1)
					{
            ros::Rate loop_rate(10);
            for (int i = 0; i < 30; ++i)
            {
              geometry_msgs::Twist cmd_msg_back;
              cmd_msg_back.angular.z = 0.4;
              base_cmd_pub.publish(cmd_msg_back);
              loop_rate.sleep();
            }
    
			      if (!MoveToGoal(position_standy, gh))
			      {
				     	return;
			      }
					}
					gh.publishFeedback(feedback_);
				}

				if (gh.getGoalStatus().status == 2) {
					ROS_INFO("return");
					task_active_ = false;
					return;
				}
				else if(step == 2) {
					loop = false;
				}
				step += 1;
		  }
	  	ROS_INFO("%i", i);
  	}

    if (gh.getGoalStatus().status == 1)
    {
			// result_.task_id = gh.getGoal()->task_id;
		  // gh.setSucceeded(result_, "All finished");
		  gh.setSucceeded();
			ROS_INFO("set succeeded");
    }
	  task_active_ = false;
  }

	bool MoveToGoal(geometry_msgs::Pose pose, GoalHandle gh) {
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
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sound_play_server");

  PickTaskAction averaging("introduce");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  // ros::spin();

  return 0;
}