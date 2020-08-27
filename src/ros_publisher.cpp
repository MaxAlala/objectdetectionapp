/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ros_publisher.cpp
 * Author: northlight
 * 
 * Created on July 13, 2020, 5:35 PM
 */

#include "ros_publisher.h"
#include <ros/ros.h>

ros_publisher::ros_publisher(int argc, char** argv) {
    ros::init(argc, argv, "test_node_outside_catkin_ws");
    ros::NodeHandle n;
    ROS_INFO("It worked!");
}


ros_publisher::~ros_publisher() {
}




