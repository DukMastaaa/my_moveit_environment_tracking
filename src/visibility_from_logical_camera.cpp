// ROS
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

// Gazebo
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/msgs/logical_camera_image.pb.h"
#include "gazebo/gazebo_client.hh"

// STL
#include <functional>
#include <iostream>

// This package
#include "my_moveit_environment_tracking/default_params.h"
#include "my_moveit_environment_tracking/VisibleModels.h"


tf::Transform gzPoseToTransform(gazebo::msgs::Pose gz_pose) {
    auto gz_q = gz_pose.orientation();
    auto gz_p = gz_pose.position();
    return tf::Transform{
            tf::Quaternion{gz_q.x(), gz_q.y(), gz_q.z(), gz_q.w()},
            tf::Vector3{gz_p.x(), gz_p.y(), gz_p.z()}};
}


class LogicalCameraReader {
private:
    // Gazebo node.
    gazebo::transport::NodePtr gazebo_node;

    // Ros node handle.
    ros::NodeHandle ros_node;

    // Gazebo subscriber to the logical camera.
    gazebo::transport::SubscriberPtr camera_sub;

    // ROS publisher.
    ros::Publisher visible_models_pub;

private:
    // Callback when Gazebo transport called.
    void gazeboCallback(ConstLogicalCameraImagePtr& _msg);

public:
    LogicalCameraReader(
            gazebo::transport::NodePtr gazebo_node_,
            ros::NodeHandle ros_node_,
            const std::string& gz_logical_camera_topic_,
            const std::string& ros_visible_models_topic_);
};

LogicalCameraReader::LogicalCameraReader(
        gazebo::transport::NodePtr gazebo_node_,
        ros::NodeHandle ros_node_,
        const std::string& gz_logical_camera_topic_,
        const std::string& ros_visible_models_topic_)
    : gazebo_node{gazebo_node_}, ros_node{ros_node_},
    camera_sub{}, visible_models_pub{ros_visible_models_topic_, 1}
{
    camera_sub = gazebo_node->Subscribe(gz_logical_camera_topic_,
            &LogicalCameraReader::gazeboCallback, this);
}

void LogicalCameraReader::gazeboCallback(ConstLogicalCameraImagePtr& _msg) {
    std::vector<std::string> model_names;
    for (int i = 0; i < _msg->model_size(); i++) {
        model_names.push_back(_msg->model(i).name())
    }
    visible_models_pub.publish(model_names);
}

int main(int argc, char** argv) {
    const std::string gz_topic_param_name = "/logical_cam_gz_topic";
    const std::string default_gz_topic = "~/post/link_for_camera/logical_camera/models";
    const std::string visible_models_topic_param_name = "/visible_models_topic";
    const std::string default_visible_models_topic  = "/visible_models";

    gazebo::client::setup(argc, argv);
    gazebo::transport::NodePtr gazebo_node(new gazebo::transport::Node());
    gazebo_node->Init();
    ROS_INFO_STREAM("gazebo node initialised\n");

    ros::init(argc, argv, "visibility_from_logical_camera");
    ros::NodeHandle ros_node;
    ROS_INFO_STREAM("ros node initialised\n");

    std::string camera_topic;
    getParam(ros_node, gz_topic_param_name, &camera_topic, default_gz_topic);
    std::string visible_models_topic;
    getParam(ros_node, visible_models_topic_param_name, &visible_models_topic, default_visible_models_topic);

    LogicalCameraReader converter{gazebo_node, ros_node, camera_topic, visible_models_topic};
    while (true) {
        gazebo::common::Time::MSleep(10);
        ros::spinOnce();
    }

    gazebo::shutdown();
    ROS_INFO_STREAM("shutting down\n");
    return 0;
}
