/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   InteractiveMarkerRos.h
 * Author: northlight
 *
 * Created on July 16, 2020, 3:10 PM
 */

#ifndef INTERACTIVEMARKERROS_H
#define INTERACTIVEMARKERROS_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include <Eigen/Geometry>

using namespace visualization_msgs;
using namespace interactive_markers;

namespace interactiveMarkerRos
{
 static bool shouldISendDetectedObject = 3;   
}
static bool shouldISendDetectedObject2 = 0;   
void graspObject( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
void dontGraspObject( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );


class InteractiveMarkerRos {
public:
    InteractiveMarkerRos(Eigen::Vector3d& position_of_the_first_match, Eigen::Quaterniond& r_quat);
    void initMenu();
    void makeMenuMarker(std::string name);
    InteractiveMarker makeEmptyMarker(bool dummyBox = true);
    InteractiveMarkerControl& makeBoxControl(InteractiveMarker &msg);
    Marker makeBox(InteractiveMarker &msg);
     virtual ~InteractiveMarkerRos();

    boost::shared_ptr<InteractiveMarkerServer> server;
    float marker_pos = 0;

    MenuHandler menu_handler;
    MenuHandler::EntryHandle h_first_entry;
    MenuHandler::EntryHandle h_mode_last;
//    void (*pointerToYes)();
//    void (* pointerToNo)();
    Eigen::Vector3d position_of_the_first_match;
    Eigen::Quaterniond r_quat;

private:

};

#endif /* INTERACTIVEMARKERROS_H */

