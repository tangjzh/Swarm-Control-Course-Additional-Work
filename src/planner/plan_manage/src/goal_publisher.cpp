#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <map>
#include <XmlRpcValue.h>

struct Goal {
    double x, y, z;
};

class GoalPublisher {
public:
    GoalPublisher(ros::NodeHandle& nh, int drone_count) 
    : nh_(nh), drone_count_(drone_count), reached_count_(0), goal_index_(0) {
        getGoalsFromParam("goal_publisher/goals", goal_list_);
        reached = std::vector<bool>(drone_count);

        for (int i = 0; i < drone_count_; ++i) {
            reached[i] = false;
            // 初始化发布者，不使用latched模式
            ros::Publisher pub = nh_.advertise<geometry_msgs::PoseStamped>("/drone_" + std::to_string(i) + "/goal", 10);
            goal_pubs_[i] = pub;

            // 动态订阅每个无人机的`planning/finish`话题
            std::string topic_name = "/drone_" + std::to_string(i) + "_planning/finish";
            ros::Subscriber sub = nh_.subscribe<std_msgs::Bool>(topic_name, 10, boost::bind(&GoalPublisher::reachedCallback, this, _1, i));
            reached_subs_.push_back(sub);
        }
    }

    void publishGoals() {
        ros::Rate rate(0.5); // 1 Hz
        while (ros::ok()) {
            if (reached_count_ >= drone_count_ >> 1) {
                for (int i = 0; i < drone_count_; i++) reached[i] = false;
                ROS_INFO("Next formation!");
                ros::Duration(1.0).sleep(); // 等待一会再发布下一个目标位置
                // publishNextGoals();
                goal_index_++;
                reached_count_ = 0; // 重置计数器
            }
            publish_();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void getGoalsFromParam(const std::string& param_name, std::map<int, std::vector<Goal>>& goals) {
        XmlRpc::XmlRpcValue goals_param;
        if (nh_.getParam(param_name, goals_param)) {
            ROS_ASSERT(goals_param.getType() == XmlRpc::XmlRpcValue::TypeStruct);
            for (XmlRpc::XmlRpcValue::iterator it = goals_param.begin(); it != goals_param.end(); ++it) {
                int drone_id = -1;
                try {
                    std::string drone_id_str = it->first.substr(6); // 提取 'drone_' 之后的部分
                    drone_id = std::stoi(drone_id_str);
                } catch (const std::invalid_argument& e) {
                    ROS_ERROR("Invalid argument for drone_id: %s", it->first.c_str());
                    continue;
                } catch (const std::out_of_range& e) {
                    ROS_ERROR("Out of range error for drone_id: %s", it->first.c_str());
                    continue;
                }
                
                ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeArray);
                for (int i = 0; i < it->second.size(); ++i) {
                    ROS_ASSERT(it->second[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
                    Goal goal;
                    goal.x = static_cast<double>(it->second[i]["x"]);
                    goal.y = static_cast<double>(it->second[i]["y"]);
                    goal.z = static_cast<double>(it->second[i]["z"]);
                    goals[drone_id].push_back(goal);
                }
            }
        } else {
            ROS_ERROR("Failed to get param '%s'", param_name.c_str());
        }
    }

    void publish_() {
        for (int i = 0; i < drone_count_; ++i) {
            if (goal_index_ >= goal_list_[i].size()) {
                ROS_INFO("All goals have been published for drone %d.", i);
                continue;
            }

            geometry_msgs::PoseStamped goal_msg;
            goal_msg.header.stamp = ros::Time::now();
            goal_msg.header.frame_id = "world";
            goal_msg.pose.position.x = goal_list_[i][goal_index_].x;
            goal_msg.pose.position.y = goal_list_[i][goal_index_].y;
            goal_msg.pose.position.z = goal_list_[i][goal_index_].z;

            goal_pubs_[i].publish(goal_msg);
            // ROS_INFO("Published goal for drone %d: [%f, %f, %f]", i, goal_list_[i][goal_index_].x, goal_list_[i][goal_index_].y, goal_list_[i][goal_index_].z);
            ros::Duration(0.2).sleep();
        }
        // goal_index_++;
    }

    void reachedCallback(const std_msgs::Bool::ConstPtr& msg, int drone_id) {
        if (msg->data) {
            if (!reached[drone_id]) {
                reached_count_++;
                reached[drone_id] = true;
                ROS_INFO("Drone %d reached the goal. Total reached: %d", drone_id, reached_count_);
            }
        }
    }

    ros::NodeHandle& nh_;
    std::map<int, ros::Publisher> goal_pubs_;
    std::vector<ros::Subscriber> reached_subs_;
    std::map<int, std::vector<Goal>> goal_list_;
    int drone_count_;
    int reached_count_;
    std::vector<bool> reached;
    size_t goal_index_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle nh("~");

    int drone_count;
    nh.getParam("drone_count", drone_count);
    ROS_INFO("drone_count: %d", drone_count);

    GoalPublisher goal_publisher(nh, drone_count);
    goal_publisher.publishGoals();

    return 0;
}
