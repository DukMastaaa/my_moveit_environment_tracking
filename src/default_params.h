#pragma once

#include "ros/ros.h"

#include <iostream>

void getParam(ros::NodeHandle& ros_node, const std::string& param,
        std::string* value, const std::string& default_value);
