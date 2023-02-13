#include "my_moveit_environment_tracking/default_params.h"

#include "ros/ros.h"

#include <iostream>

void getParam(ros::NodeHandle& ros_node, const std::string& param,
        std::string* value, const std::string& default_value) {
    if (!ros_node.getParam(param, *value)) {
        ROS_INFO_STREAM("No value for " << param
                << " in parameter server; using default value: \""
                << default_value << "\"");
        *value = default_value;
    }
}