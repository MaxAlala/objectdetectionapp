This application can be taught to detect and capture specific objects. The main algorithm is d2co. It also creates a 3d point cloud. Currently, you need a ZED camera to start this application.
Tutorial how to install all dependencies and this application u can find here:
link to install tutorial!
Two articles that describe D2CO algorithm:
1 article
2 article
bitbucket of the algorithm( source code, a bit of description etc):
default algorithm source code
Source code of this application can be found here:
app source
what is important to understand?
The main files that run the program (the rest of the files describe the operation of the d2c0 algorithm):
objectdetectionapp/apps: 
generate_models.cpp
frame_tf_broadcaster.cpp
detect_object.cpp
objectdetectionapp/include:
InteractiveMarkerRos.h
opencv.h
CMakeLists.txt
HOW TO LAUNCH THE APPLICATION: 
Firstly, u should install the application using the link above.
Secondly, you should create a model file using generate_model executable file:
Example:
./generate_models -m 3D_models/toGrasp.stl -c test_images/test_camera_calibHD.yml -p fileHD.yml -i saved_images/image.png  -s 1 --d_rp 5 --rp_s 5 --d_yaw 5 --yaw_s 5 --d_z 0.1 --z_s 0.05

after u should start these: 

roscore

rosrun rviz rviz

./frame_tf_broadcaster

./detect_object -m fileHD.yml  -c test_images/test_camera_calibHD.yml -d saved_images -o VIDEO -s 1


in rviz u should add:

PointCloud2 with topic poincloud

interactiveMarker

Marker

TF
Fixed Frame is "map".

Also an example of working with this application u can find in the video below:
video example
in detail: how the whole application works:

after succesfull creation of model file using generate_model executable
For example, launch command is:  
./generate_models -m 3D_models/toGraps.stl -c test_images/test_camera_calibHD.yml -p fileHD.yml -i saved_images/image.png  -s 1 --d_rp 5 --rp_s 5 --d_yaw 5 --yaw_s 5 --d_z 0.1 --z_s 0.05

where "--d_rp" specifies roll & pitch distance to reach, "--rp_s" specifies a step size.
Similar idea with d_z and d_yaw.

after u should start these auxiliary programs: 



roscore // it creates main controller of ROS nodes 

rosrun rviz rviz // it launches RVIZ program for visualization
in rviz u should add:
a) PointCloud2 with topic poincloud
b) interactiveMarker
c) Marker
d) TF
Fixed Frame is "map".

./frame_tf_broadcaster // it creates topic which sends camera frame data to the RVIZ

./detect_object -m fileHD.yml  -c test_images/test_camera_calibHD.yml -d saved_images -o VIDEO -s 1



./detect_object:



you take an image and chose its ROI(region of interest) to specify area to improve speed and d2c0 algorithm detection quality)

algorithm works, detects the best matching object and displays it in a window

you can accept a detected object then opencv object will create two RVIZ Markers(interactive and not)using orientation and location, pointcloud
also opencv object will calculate a tool orientation and set it to the global variable to send it to the robot

if u deny an detected object, u can repeat an attempt 2 more times. Also u can finish the program and start all from beginning.


So then u choose a detected object and sent it to the RVIZ here you can click on the right mouse button on the detected object model and chose to start the socket to send tool orientation( ABC-ZYX Euler angles) to the robot.
on the robot you must run the corresponding program - it will create server which will listen incoming
connections. When connection happens, robot will start an object grapsing process. It will lift it and after put it on the floor.


Idea how to find tool orientation using detected object orientation:  <br/>

// Robot tool orientation is expressed in map frame. So detected object orientation also expressed   <br/>
// in map frame.  <br/>
  // here is our goal to find out an angle to rotate the tool to grasp the detected object  <br/>
    // 1) we suppose that tool has the same orientation as the detected object, for example, this is our object and its XZ axes  <br/>
    //  ^ Z  <br/>
    //  |  <br/>
    //  ------------  <br/>
    //  |    |  <br/>
    //  ------------ ------> X  <br/>
    // To grasp an object we need to orient our tool so that it was directed toward our object and  the grasping gap of the tool was directed rightly  <br/>
    // |-----|  <br/>
    // | &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|*X == X axis directed toward us  <br/>
    // |_| |_| this is our tool   <br/>
    // |  <br/>
    // |  <br/>
    // |  <br/>
    // \/ Z  <br/>
    // 2) as we can see, our single goal to make tool Z be directed in opposite direction to Z of the object, and then a tool  <br/>
    // will easily grasp a detected object  <br/>
    // so we can find the angle between Z and tool's Z_1 using dot product from a rotation matrix  <br/>
    // and we will rotate this angle around X-axis so our Z & Z_1 will be directed in the opposite directions.  <br/>
    // 3) final thing to do is to define the sign of angle between Z and Z_1 to rotate around X  <br/>
    // We can define it with respect to Y1_Z, if the dot product is less then zero then we change   sign, else do nothing  <br/>
    // 4) Tool orientation is ready.  <br/>






