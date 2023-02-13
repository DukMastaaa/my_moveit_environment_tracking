// ROS
#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"

// This package
#include "my_moveit_environment_tracking/default_params.h"
#include "my_moveit_environment_tracking/VisibleModels.h"

class GazeboReader {
private:
    ros::NodeHandle ros_node;
    ros::Subscriber gz_state_sub;
    ros::Publisher visible_models_pub;

public:
    GazeboReader(ros::NodeHandle& ros_node_,
            const std::string& gz_state_topic_,
            const std::string& visible_models_topic_);
    
    void callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
};

GazeboReader::GazeboReader(ros::NodeHandle& ros_node_,
        const std::string& gz_state_topic_,
        const std::string& visible_models_topic_)
    : ros_node{ros_node_}, gz_state_sub{}, visible_models_pub{visible_models_topic_, 1}
{
    gz_state_sub = ros_node.subscribe(gz_state_topic_, 1, &GazeboReader::callback, this);
}

void GazeboReader::callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // just publish all names
    visible_models_pub.publish(msg.name);
}

int main(int argc, char** argv) {
    const std::string gz_state_topic = "/gazebo/model_states"
    const std::string visible_models_topic_param_name = "/visible_models_topic";
    const std::string default_visible_models_topic = "/visible_models";

    ros::init(argc, argv, "visibility_from_logical_camera");
    ros::NodeHandle ros_node;

    std::string visible_models_topic;
    getParam(ros_node, visible_models_topic_param_name, &visible_models_topic, default_visible_models_topic);

    GazeboReader reader{ros_node, gz_state_topic, visible_models_topic};
    ros::spin();

    return 0;
}
