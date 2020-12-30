#include <caster_moma_app/PickItemsAction.h>
//#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>

//这样定义下会用起来简洁许多
typedef actionlib::SimpleActionClient<caster_moma_app::PickItemsAction> Client;

class PickItemsActionClient {
private:
    // Called once when the goal completes
    void DoneCb(const actionlib::SimpleClientGoalState& state,
            const caster_moma_app::PickItemsResultConstPtr& result) {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("Toal dish cleaned: %i", result->finished_step);
        ros::shutdown();
    }

    // 当目标激活的时候，会调用一次
    void ActiveCb() {
        ROS_INFO("Goal just went active");
    }

    // 接收服务器的反馈信息
    void FeedbackCb(
            const caster_moma_app::PickItemsFeedbackConstPtr& feedback) {
        // ROS_INFO("Got Feedback Complete Rate: %f", feedback->step);
    }
public:
    PickItemsActionClient(const std::string client_name, bool flag = true) :
            client(client_name, flag) {
    }

    //客户端开始
    void Start() {
        //等待服务器初始化完成
        client.waitForServer();
        //定义要做的目标
        caster_moma_app::PickItemsGoal goal;
        goal.items = 1;
        //发送目标至服务器
        client.sendGoal(goal,
                boost::bind(&PickItemsActionClient::DoneCb, this, _1, _2),
                boost::bind(&PickItemsActionClient::ActiveCb, this),
                boost::bind(&PickItemsActionClient::FeedbackCb, this, _1));
        //等待结果，时间间隔5秒
        client.waitForResult(ros::Duration(2.0));

        //根据返回结果，做相应的处理
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            // printf("Yay! The dishes are now clean");
            ROS_INFO("SUCCEEDED");
        else {
            ROS_INFO("Cancel Goal!");
            client.cancelAllGoals();
        }

        ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
    }
private:
    Client client;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_task_client");
    PickItemsActionClient actionclient("pick_task", true);
    //启动客户端
    actionclient.Start();
    ros::spin();
    return 0;
}