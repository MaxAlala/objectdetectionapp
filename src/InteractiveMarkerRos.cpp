/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   InteractiveMarkerRos.cpp
 * Author: northlight
 * 
 * Created on July 16, 2020, 3:10 PM
 */

#include "InteractiveMarkerRos.h"
// for tcp server
#include<stdio.h>
#include<string.h> //strlen
#include<sys/socket.h>
#include<arpa/inet.h> //inet_addr
#include<unistd.h> //write
#include <string>

//

#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
extern Eigen::Vector3d g_position_of_the_first_match;
extern Eigen::Quaterniond g_r_quat;
extern Eigen::Quaterniond g_r_quat_tool;


/*!
 * \brief it starts the client and sends tool coordinates and orientation
 **/
int createSocketClientAndSendToolCoordinatesOrientation() {
    /*
        C socket client example
     */
    using namespace std;

    //	Create a socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        return 1;
    }

    //	Create a hint structure for the server we're connecting with
    int port = 54600;
    string ipAddress = "192.168.212.99";
    int multiplier = 10;
    int C = g_r_quat_tool.toRotationMatrix().eulerAngles(2, 1, 0)(2)*180 / 3.1415;
    int B = g_r_quat_tool.toRotationMatrix().eulerAngles(2, 1, 0)(1)*180 / 3.1415;

    int A = g_r_quat_tool.toRotationMatrix().eulerAngles(2, 1, 0)(0)*180 / 3.1415;
    int X = (g_position_of_the_first_match[0] + 2) * multiplier;
    int Y = (g_position_of_the_first_match[1] + 7) * multiplier;
    int Z = 2 * multiplier;
    if (abs(C) < 1) C = 1;
    if (abs(B) < 1) B = 1;
    if (abs(A) < 1) A = 1;
    if (abs(X) < 1) X = 1;
    if (abs(Y) < 1) Y = 1;
    if (abs(Z) < 1) Z = 1;

    std::string coordinates = "<Server><Pos2><X>" + to_string(X) + "</X><Y>" + to_string(Y) + "</Y><Z>" + to_string(Z) + "</Z><A>" + to_string(A) + "</A><B>" + to_string(B) + "</B><C>" + to_string(C) + "</C></Pos2></Server>";

    sockaddr_in hint;
    hint.sin_family = AF_INET;
    hint.sin_port = htons(port);
    inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);

    //	Connect to the server on the socket
    int connectRes = connect(sock, (sockaddr*) & hint, sizeof (hint));
    if (connectRes == -1) {
        return 1;
    }

    //	While loop:
    char buf[4096];
    string userInput;


    do {

        //		Wait for response
        memset(buf, 0, 4096);
        int bytesReceived = recv(sock, buf, 4096, 0);
        if (bytesReceived == -1) {
            cout << "There was an error getting response from server\r\n";
        } else {
            //		Display response
            cout << "SERVER> " << string(buf, bytesReceived) << "\r\n";
        }
        //        //		Enter lines of text
        //        cout << "> ";
        //        getline(cin, userInput);

        //		Send to server
        cout << "sending this coordinates: " << coordinates << endl;
        int sendRes = send(sock, coordinates.c_str(), coordinates.size() + 1, 0);
        if (sendRes == -1) {
            cout << "Could not send to server! Whoops!\r\n";
            break;
        }
        char RunOrExitTheProgram = ' ';
        while (RunOrExitTheProgram != 'Y' && RunOrExitTheProgram != 'N') {
            std::cout << "Error invalid input please try again " << std::endl;
            std::cout << "Please enter [Y/N]. Run or exit the program: ";
            std::cin >> RunOrExitTheProgram;
            RunOrExitTheProgram = toupper(RunOrExitTheProgram);
        }
        std::cout << "u have pressed: " << RunOrExitTheProgram;
        ////// save XYZ AND orientation of the first match

        if (RunOrExitTheProgram == 'N') {
            exit(0);
        }

    } while (true);

    //	Close the socket
    close(sock);

    return 0;

    //    std::string coordinates = "<Server><Pos2><X>" + to_string(g_position_of_the_first_match[0]) + "</X><Y>" + to_string(g_position_of_the_first_match[1]) + "</Y><Z>" + to_string(g_position_of_the_first_match[2]) + "</Z><A>" + to_string(g_r_quat.z()) + "</A><B>" + to_string(g_r_quat.y()) + "</B><C>" + to_string(g_r_quat.x()) + "</C></Pos2></Server>";
    //    int socket_desc, client_sock, c, read_size;
    //    struct sockaddr_in server, client;
    //    char client_message[2000];
    //
    //    //Create socket
    //    socket_desc = socket(AF_INET, SOCK_STREAM, 0);
    //    if (socket_desc == -1) {
    //        printf("Could not create socket");
    //    }
    //    puts("Socket created");
    //
    //    //Prepare the sockaddr_in structure
    //    server.sin_family = AF_INET;
    //    server.sin_addr.s_addr = INADDR_ANY;
    //    server.sin_port = htons(59152);
    //
    //    //Bind
    //    if (bind(socket_desc, (struct sockaddr *) &server, sizeof (server)) < 0) {
    //        //print the error message
    //        perror("bind failed. Error");
    //        return 1;
    //    }
    //    puts("bind done");
    //
    //    //Listen
    //    listen(socket_desc, 3);
    //
    //    //Accept and incoming connection
    //    puts("Waiting for incoming connections...");
    //    c = sizeof (struct sockaddr_in);
    //
    //    //accept connection from an incoming client
    //    client_sock = accept(socket_desc, (struct sockaddr *) &client, (socklen_t*) & c);
    //    if (client_sock < 0) {
    //        perror("accept failed");
    //        return 1;
    //    }
    //    puts("Connection accepted");
    //
    //    //Receive a message from client
    //    while ((read_size = recv(client_sock, client_message, 2000, 0)) > 0) {
    //        //Send the message back to client
    //
    //    }
    //    
    //    cout << client_message << " received this coordinate from client. \n";
    //    
    //    
    //    cout << coordinates << " send this coordinate. \n";
    //    write(client_sock, coordinates.c_str(), strlen(coordinates.c_str()));
    //    
    //    
    //    cout << client_message << " received this coordinate from client. \n";
    //    if (read_size == 0) {
    //        puts("Client disconnected");
    //        fflush(stdout);
    //    } else if (read_size == -1) {
    //        perror("recv failed");
    //    }
    //    return 0;

}

/*!
 * \brief this function represents a reaction to "Yes" selection == u chose to send orientation and position to a robot
 **/
void graspObject(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    interactiveMarkerRos::shouldISendDetectedObject = 1;
    shouldISendDetectedObject2 = 34;
    createSocketClientAndSendToolCoordinatesOrientation();

    ROS_INFO("You chose 'yes'.");

}

/*!
 * \brief this function represents a reaction to "No" selection == u chose to not send orientation and position to a robot
 **/
void dontGraspObject(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    interactiveMarkerRos::shouldISendDetectedObject = 0;
    ROS_INFO("You chose 'no'.");
}

Marker InteractiveMarkerRos::makeBox(InteractiveMarker &msg) {
    Marker marker;
    float scale = 0.10;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://mesh/toGrasp.stl";
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.pose.position.x = position_of_the_first_match[0];
    marker.pose.position.y = position_of_the_first_match[1];
    marker.pose.position.z = position_of_the_first_match[2];
    marker.pose.orientation.x = r_quat.x();
    marker.pose.orientation.y = r_quat.y();
    marker.pose.orientation.z = r_quat.z();
    marker.pose.orientation.w = r_quat.w();
    marker.color.r = 153.0f;
    marker.color.g = 0.0f;
    marker.color.b = 153.0f;
    marker.color.a = 0.7;

    return marker;
}

InteractiveMarkerControl& InteractiveMarkerRos::makeBoxControl(InteractiveMarker &msg) {
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
}

InteractiveMarker InteractiveMarkerRos::makeEmptyMarker(bool dummyBox) {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.pose.position.y = -3.0 * marker_pos++;
    ;
    int_marker.scale = 1;

    return int_marker;
}

void InteractiveMarkerRos::makeMenuMarker(std::string name) {
    InteractiveMarker int_marker = makeEmptyMarker();
    int_marker.name = name;

    InteractiveMarkerControl control;

    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    control.markers.push_back(makeBox(int_marker));
    int_marker.controls.push_back(control);

    server->insert(int_marker);
}

void InteractiveMarkerRos::initMenu() {
    h_first_entry = menu_handler.insert("should i pick detected object up?");
    MenuHandler::EntryHandle entry = menu_handler.insert(h_first_entry, "yes", &graspObject);
    entry = menu_handler.insert(h_first_entry, "no", &dontGraspObject);
}

InteractiveMarkerRos::InteractiveMarkerRos(Eigen::Vector3d& position_of_the_first_match, Eigen::Quaterniond& r_quat) {
    for (int i = 0; i < 3; i++)
        this->position_of_the_first_match[i] = position_of_the_first_match[i];

    this->r_quat.x() = r_quat.x();
    this->r_quat.y() = r_quat.y();
    this->r_quat.w() = r_quat.w();
    this->r_quat.z() = r_quat.z();

}

InteractiveMarkerRos::~InteractiveMarkerRos() {
}
