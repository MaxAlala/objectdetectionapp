This application can be taught to detect and capture specific objects. The main algorithm is d2co. It also creates a 3d point cloud. Currently, you need a ZED camera to start this application. 

**Tutorial how to install all dependencies and this application u can find here:**

[link to install tutorial!](https://docs.google.com/document/d/18XBOcXv9CPr6PLEfTI-7UOkYoK9YX4jHX02k7BjThcc/edit)


**Two articles that describe D2CO algorithm:**

[1 article](https://www.researchgate.net/publication/300780532_D_2_CO_Fast_and_Robust_Registration_of_3D_Textureless_Objects_Using_the_Directional_Chamfer_Distance
)

[2 article](http://www.dis.uniroma1.it/labrococo/D2CO/miap_arxiv2016.pdf)

**bitbucket of the algorithm( source code, a bit of description etc):**
[default algorithm source code](https://bitbucket.org/alberto_pretto/d2co/src/master/)

**Source code of this application can be found here:**
[app source](https://gitlab.com/LIRS_Projects/KUKA-object-detection-grasping.git)

**what is important to understand?**
The main files that run the program (the rest of the files describe the operation of the d2c0 algorithm):
objectdetectionapp/apps: <br/>
generate_models.cpp<br/>
frame_tf_broadcaster.cpp<br/>
detect_object.cpp<br/>

objectdetectionapp/include:<br/>
InteractiveMarkerRos.h<br/>
opencv.h<br/>

CMakeLists.txt




**HOW TO LAUNCH THE APPLICATION:** <br/>
Firstly, u should install the application using the link above.<br/>

Secondly, you should create a model file using generate_model executable file:<br/>

Example:<br/>
./generate_models -m 3D_models/toGrasp.stl -c test_images/test_camera_calibHD.yml -p fileHD.yml -i saved_images/image.png  -s 1 --d_rp 5 --rp_s 5 --d_yaw 5 --yaw_s 5 --d_z 0.1 --z_s 0.05 
<br/>
after u should start these: <br/>
1) roscore<br/>
2) rosrun rviz rviz<br/>
3) ./frame_tf_broadcaster<br/>
4) ./detect_object -m fileHD.yml  -c test_images/test_camera_calibHD.yml -d saved_images -o VIDEO -s 1<br/>

in rviz u should add:<br/>
1) PointCloud2 with topic poincloud<br/>
2) interactiveMarker<br/>
3) Marker<br/>
4) TF<br/>
Fixed Frame is "map".

Also an example of working with this application u can find in the video below:

[video example](https://drive.google.com/file/d/101cpjqpP12bDUX41HH18D0PB3zEDrllV/view?usp=sharing)




**in detail: how the whole application works:**
 * after succesfull creation of model file using generate_model executable
For example, launch command is:  <br/>
./generate_models -m 3D_models/toGraps.stl -c test_images/test_camera_calibHD.yml -p fileHD.yml -i saved_images/image.png  -s 1 --d_rp 5 --rp_s 5 --d_yaw 5 --yaw_s 5 --d_z 0.1 --z_s 0.05 

where "--d_rp" specifies roll & pitch distance to reach, "--rp_s" specifies a step size.

Similar idea with d_z and d_yaw.

* after u should start these auxiliary programs: <br/>
1) roscore // it creates main controller of ROS nodes <br/> 
2) rosrun rviz rviz // it launches RVIZ program for visualization<br/>
in rviz u should add:<br/>
a) PointCloud2 with topic poincloud<br/>
b) interactiveMarker<br/>
c) Marker<br/>
d) TF<br/>
Fixed Frame is "map".<br/>
3) ./frame_tf_broadcaster // it creates topic which sends camera frame data to the RVIZ<br/>
4) ./detect_object -m fileHD.yml  -c test_images/test_camera_calibHD.yml -d saved_images -o VIDEO -s 1<br/>

*  ./detect_object:<br/>
1) you take an image and chose its ROI(region of interest) to specify area to improve speed and d2c0 algorithm detection quality)<br/>
2) algorithm works, detects the best matching object and displays it in a window<br/>
3) you can accept a detected object then opencv object will create two RVIZ Markers(interactive and not)using orientation and location, pointcloud
also opencv object will calculate a tool orientation and set it to the global variable to send it to the robot<br/>
4) if u deny an detected object, u can repeat an attempt 2 more times. Also u can finish the program and start all from beginning.<br/>
   
   
So then u choose a detected object and sent it to the RVIZ here you can click on the right mouse button on the detected object model and chose to start the socket to send tool orientation( ABC-ZYX Euler angles) to the robot.<br/>


on the robot you must run the corresponding program - it will create server which will listen incoming
connections. When connection happens, robot will start an object grapsing process. It will lift it and after put it on the floor.<br/>






