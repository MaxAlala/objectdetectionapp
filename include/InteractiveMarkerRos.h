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



/*!
 * \brief it starts the client and sends tool coordinates and orientation
 **/
int createSocketClientAndSendToolCoordinatesOrientation();


namespace interactiveMarkerRos {
    static bool shouldISendDetectedObject = 3;
}
static bool shouldISendDetectedObject2 = 0;

/*!
 * \brief this function represents a reaction to "Yes" selection == u chose to send orientation and position to a robot
 **/
void graspObject(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

/*!
 * \brief this function represents a reaction to "No" selection == u chose to not send orientation and position to a robot
 **/
void dontGraspObject(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

/*!
 * \brief this class just creates an interactive marker to be able to select an option that creates socket client to send coordinates to the robot
 * also it creates the socket client using the corresponding function
 **/
class InteractiveMarkerRos {
public:
    InteractiveMarkerRos(Eigen::Vector3d& position_of_the_first_match, Eigen::Quaterniond& r_quat);
    void initMenu();
    void makeMenuMarker(std::string name);
    InteractiveMarker makeEmptyMarker(bool dummyBox = true);
    InteractiveMarkerControl& makeBoxControl(InteractiveMarker &msg);

    /*!
     * \brief it creates a  marker
     **/
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

