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

int createServerAndSendCoordinates() {
    /*
        C socket client example
     */
    using namespace std;
    
        //	Create a socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1)
    {
        return 1;
    }

    //	Create a hint structure for the server we're connecting with
    int port = 54600;
    string ipAddress = "192.168.212.99";
    int multiplier = 10;
    std::string coordinates = "<Server><Pos2><X>" + to_string(g_position_of_the_first_match[0]*multiplier) + "</X><Y>" + to_string(g_position_of_the_first_match[1]*multiplier) + "</Y><Z>" + to_string(g_position_of_the_first_match[2]*multiplier + 200) + "</Z><A>" + to_string(g_r_quat.toRotationMatrix().eulerAngles(2,1,0)(0)*180/3.1415 + 90) + "</A><B>" + to_string(1) + "</B><C>" + to_string(180) + "</C></Pos2></Server>";

    sockaddr_in hint;
    hint.sin_family = AF_INET;
    hint.sin_port = htons(port);
    inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);

    //	Connect to the server on the socket
    int connectRes = connect(sock, (sockaddr*)&hint, sizeof(hint));
    if (connectRes == -1)
    {
        return 1;
    }

    //	While loop:
    char buf[4096];
    string userInput;


    do {
        
                //		Wait for response
        memset(buf, 0, 4096);
        int bytesReceived = recv(sock, buf, 4096, 0);
        if (bytesReceived == -1)
        {
            cout << "There was an error getting response from server\r\n";
        }
        else
        {
            //		Display response
            cout << "SERVER> " << string(buf, bytesReceived) << "\r\n";
        }
//        //		Enter lines of text
//        cout << "> ";
//        getline(cin, userInput);

        //		Send to server
        cout << "sending this coordinates: " << coordinates << endl;
        int sendRes = send(sock, coordinates.c_str(), coordinates.size() + 1, 0);
        if (sendRes == -1)
        {
            cout << "Could not send to server! Whoops!\r\n";
            continue;
        }


    } while(true);

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

void graspObject(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    interactiveMarkerRos::shouldISendDetectedObject = 1;
    shouldISendDetectedObject2 = 34;
    createServerAndSendCoordinates();

    ROS_INFO("You chose 'yes'.");

}

void dontGraspObject(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    interactiveMarkerRos::shouldISendDetectedObject = 0;
    ROS_INFO("You chose 'no'.");
}

Marker InteractiveMarkerRos::makeBox(InteractiveMarker &msg) {
    Marker marker;
    float scale = 0.10;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://mesh/AX-01b_bearing_box.stl";
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
