#include "opencv.h"
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include "InteractiveMarkerRos.h"
#include <cstdlib>

#include <opencv2/tracking.hpp>
//#include <opencv2 core="" ocl.hpp="">
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
using namespace std;
using namespace cv;
using namespace std;
using namespace sl;
using namespace cv;
using namespace dnn;
using namespace std;

/*!
 * \brief position of the detected object
 */
Eigen::Vector3d g_position_of_the_first_match;

/*!
 * \brief orientation of the detected object
 */
Eigen::Quaterniond g_r_quat;

/*!
 * \brief orientation of the tool to grasp detected object
 */
Eigen::Quaterniond g_r_quat_tool;

#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

struct SelectionState {
    cv::Point startPt, endPt, mousePos;
    bool started = false, done = false;

    cv::Rect toRect() {
        return cv::Rect(
                min(this->startPt.x, this->mousePos.x),
                min(this->startPt.y, this->mousePos.y),
                abs(this->startPt.x - this->mousePos.x),
                abs(this->startPt.y - this->mousePos.y));
    }
};

void onMouse(int event, int x, int y, int, void *data) {
    SelectionState *state = (SelectionState*) data;

    switch (event) {
        case EVENT_LBUTTONDOWN:
            state->startPt.x = x;
            state->startPt.y = y;
            state->mousePos.x = x;
            state->mousePos.y = y;
            state->started = true;
            break;

        case EVENT_LBUTTONUP:
            state->endPt.x = x;
            state->endPt.y = y;
            state->done = true;
            break;

        case EVENT_MOUSEMOVE:
            state->mousePos.x = x;
            state->mousePos.y = y;
            break;
    }
}

/*!
 * \brief Method to select a ROI( rectangle area) using mouse.
 */
cv::Rect selectRect(cv::Mat image, cv::Scalar color = cv::Scalar(255, 0, 0), int thickness = 2) {
    const string window = "rect";
    SelectionState state;
    namedWindow(window, WINDOW_NORMAL);
    setMouseCallback(window, onMouse, &state);

    while (!state.done) {
        waitKey(100);

        if (state.started) {
            cv::Mat copy = image.clone();
            cv::Rect selection = state.toRect();
            rectangle(copy, selection, color, thickness);
            imshow(window, copy);
        } else {
            imshow(window, image);
        }
    }

    return state.toRect();
}

void opencv::updatePointCloud() {
    double mCamImageResizeFactor = 1.0;
    double mCamDepthResizeFactor = 1.0;
    int mCamWidth;
    int mCamHeight;
    mCamWidth = zed.getCameraInformation().camera_configuration.resolution.width;
    mCamHeight = zed.getCameraInformation().camera_configuration.resolution.height;
    int d_w = static_cast<int> (mCamWidth * mCamDepthResizeFactor);
    int d_h = static_cast<int> (mCamHeight * mCamDepthResizeFactor);
    sl::Resolution mMatResolDepth = sl::Resolution(d_w, d_h);
    sl::CameraInformation zedParam = zed.getCameraInformation(mMatResolDepth);
    zed.retrieveMeasure(current_point_cloud, sl::MEASURE::XYZBGRA, sl::MEM::CPU, mMatResolDepth);

}

void opencv::rotateVector(int X_rad, int Y_rad, int Z_rad, Eigen::Vector3d& inputVector) {
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(Y_rad, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(X_rad, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(Z_rad, Eigen::Vector3d::UnitZ())
            ;
    cout << m << endl << "received matrix: " << m.isUnitary() << endl;
    inputVector = m * inputVector;
}

void opencv::fromCMtoM(Eigen::Vector3d& vec) {

    for (int i = 0; i < 3; i++)
        vec[i] /= 100;
}

/*!
 * \brief it sends 2 markers & point cloud to RVIZ also creates a tool orientation for server
 **/
void opencv::runNode(int argc, char** argv, Eigen::Vector4d& position_of_the_first_match, Eigen::Quaterniond& r_quat) {

    cout << "detected object orientation in rotation matrix form: " << r_quat.toRotationMatrix() << endl;
    //    double Z = acos(r_quat.toRotationMatrix()(0,2));
    double X = 3.1415;
    // every value of rotation matrix is dot product of two axes
    // ZYX are map axes, Z_1 Y_1 X_1 are detected object axes.
    double DotProductZ_Z1 = r_quat.toRotationMatrix()(2, 2);
    double DotProductZ_Y1 = r_quat.toRotationMatrix()(2, 1);
    double DotProductX_X1 = r_quat.toRotationMatrix()(0, 0);

    cout << "DOT PRODUCT OF Z1_Z IS: " << DotProductZ_Z1 << endl;
    cout << "DOT PRODUCT OF Z1_Y IS: " << DotProductZ_Y1 << endl;
    cout << "DOT PRODUCT OF X1_X IS: " << DotProductX_X1 << endl;

    double angleBetweenZaxises = acos(DotProductZ_Z1);
    double angleBetweenY1_Zaxises = acos(DotProductZ_Y1);

    cout << "ANGLE OF DOT PRODUCT Z1_Z is: " << angleBetweenZaxises;
    cout << "ANGLE OF DOT PRODUCT Y1_Z is: " << angleBetweenY1_Zaxises;
    angleBetweenZaxises = abs(angleBetweenZaxises); // in case an angle is negative

Idea how to find tool orientation using detected object orientation:  

// Robot tool orientation is expressed in map frame. So detected object orientation also expressed   
// in map frame.  
  // here is our goal to find out an angle to rotate the tool to grasp the detected object  
    // 1) we suppose that tool has the same orientation as the detected object, for example, this is our object and its XZ axes  
    //  ^ Z  
    //  |  
    //  ------------  
    //  |          |  
    //  ------------ ------> X  
    // To grasp an object we need to orient our tool so that it was directed toward our object and  the grasping gap of the tool was directed rightly  
    // |-----|  
    // |  _  |*X == X axis directed toward us  
    // |_| |_| this is our tool   
    // |  
    // |  
    // |  
    // \/ Z  
    // 2) as we can see, our single goal to make tool Z be directed in opposite direction to Z of the object, and then a tool  
    // will easily grasp a detected object  
    // so we can find the angle between Z and tool's Z_1 using dot product from a rotation matrix  
    // and we will rotate this angle around X-axis so our Z & Z_1 will be directed in the opposite directions.  
    // 3) final thing to do is to define the sign of angle between Z and Z_1 to rotate around X  
    // We can define it with respect to Y1_Z, if the dot product is less then zero then we change   sign, else do nothing  
    // 4) Tool orientation is ready.  
            
            
    if (DotProductZ_Z1 < 0) X = angleBetweenZaxises;
    else X = X - angleBetweenZaxises; // 180 - a
    if (DotProductZ_Y1 < 0) X = -X;
    else X = X;

    cout << "ROTATION AROUND X( final rads): " << X << endl;

    Eigen::Matrix3d R_d2co_in_map; // start orientation before applying founded euler angles

    // it moves an object in z direction for better final position
    int z_movement = -2;

    // it moves an object in z direction for better final position
    Eigen::Matrix4d T_minus_z_detObject_in_detObject;
    T_minus_z_detObject_in_detObject <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, z_movement,
            0, 0, 0, 1
            ;

    Eigen::Matrix4d T_minus_z_detObject_in_map;
    Eigen::Matrix4d T_detObject_in_map;
    Eigen::Matrix3d R_detObject_in_map = r_quat.toRotationMatrix();
    T_detObject_in_map <<
            R_detObject_in_map(0, 0), R_detObject_in_map(0, 1), R_detObject_in_map(0, 2), position_of_the_first_match[0],
            R_detObject_in_map(1, 0), R_detObject_in_map(1, 1), R_detObject_in_map(1, 2), position_of_the_first_match[1],
            R_detObject_in_map(2, 0), R_detObject_in_map(2, 1), R_detObject_in_map(2, 2), position_of_the_first_match[2],
            0, 0, 0, 1
            ;

    T_minus_z_detObject_in_map = T_detObject_in_map * T_minus_z_detObject_in_detObject;
    Eigen::Matrix3d R_minus_z_detObject_in_map;
    R_minus_z_detObject_in_map <<
            T_minus_z_detObject_in_map(0, 0), T_minus_z_detObject_in_map(0, 1), T_minus_z_detObject_in_map(0, 2),
            T_minus_z_detObject_in_map(1, 0), T_minus_z_detObject_in_map(1, 1), T_minus_z_detObject_in_map(1, 2),
            T_minus_z_detObject_in_map(2, 0), T_minus_z_detObject_in_map(2, 1), T_minus_z_detObject_in_map(2, 2);
    Eigen::Vector4d P_minus_z_detObject_in_map;
    P_minus_z_detObject_in_map << T_minus_z_detObject_in_map(0, 3), T_minus_z_detObject_in_map(1, 3), T_minus_z_detObject_in_map(2, 3), 0;
    R_d2co_in_map <<
            0, -1, 0,
            -1, 0, 0,
            0, 0, -1
            ;
    cout << "DETECTED OBJECT POSITION: \n";
    cout << P_minus_z_detObject_in_map << endl;
    //-3.1415 / 2

    Eigen::Matrix<double, 3, 3> indentityMatrix;

    //rotation around X
    Eigen::Matrix<double, 3, 3> rotation_around_X;
    rotation_around_X = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(X, Eigen::Vector3d::UnitX());

    indentityMatrix <<
            1, 0, 0, 0, 1, 0, 0, 0, 1;

    indentityMatrix = rotation_around_X * indentityMatrix;

    // tool orientation
    Eigen::Matrix<double, 3, 3> R_tool_in_map;
    Eigen::Vector3d eulerAngs = r_quat.toRotationMatrix().eulerAngles(2, 1, 0);


    R_tool_in_map = R_minus_z_detObject_in_map * indentityMatrix;
    Eigen::Quaterniond detectedObjectOrientation_quat(R_minus_z_detObject_in_map);

    for (int i = 0; i < 3; i++)
        g_position_of_the_first_match[i] = P_minus_z_detObject_in_map[i];

    g_r_quat.x() = detectedObjectOrientation_quat.x();
    g_r_quat.y() = detectedObjectOrientation_quat.y();
    g_r_quat.w() = detectedObjectOrientation_quat.w();
    g_r_quat.z() = detectedObjectOrientation_quat.z();

    g_r_quat_tool = Eigen::Quaterniond(R_tool_in_map);

    ROS_INFO("It worked!");
    ros::Rate r(1);

    //it publishes detected object to ROS topic
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    //    sendPointCloud();
    // it sett our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
    // pointcloud publisher
    ros::Publisher mPubCloud;

    // it publishes a point cloud 
    mPubCloud = n.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    ////
    int mCamWidth = zed.getCameraInformation().camera_configuration.resolution.width;
    int mCamHeight = zed.getCameraInformation().camera_configuration.resolution.height;

    int d_w = static_cast<int> (mCamWidth * 1);
    int d_h = static_cast<int> (mCamHeight * 1);

    sl::Resolution mMatResolDepth = sl::Resolution(d_w, d_h);
    sl::CameraInformation zedParam = zed.getCameraInformation(mMatResolDepth);
    int ptsCount = mMatResolDepth.width * mMatResolDepth.height;

    sensor_msgs::PointCloud2Ptr pointcloudMsg = boost::make_shared<sensor_msgs::PointCloud2>();
    pointcloudMsg->header.stamp = ros::Time::now();
    pointcloudMsg->header.frame_id = "/camera";
    pointcloudMsg->is_bigendian = false;
    pointcloudMsg->is_dense = false;
    //        pointcloudMsg->data = current_point_cloud;
    pointcloudMsg->width = mMatResolDepth.width;
    pointcloudMsg->height = mMatResolDepth.height;
    sensor_msgs::PointCloud2Modifier modifier(*pointcloudMsg);
    modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::PointField::FLOAT32);
    sl::Vector4<float>* cpu_cloud = current_point_cloud.getPtr<sl::float4>();
    float* ptCloudPtr = (float*) (&pointcloudMsg->data[0]);
    // We can do a direct memcpy since data organization is the same
    memcpy(ptCloudPtr, (float*) cpu_cloud, 4 * ptsCount * sizeof (float));
    cout << "settings for point cloud are finished. \n " << current_point_cloud.getHeight() << "point cloud height . \n";

    while (ros::ok()) {
        const int size = 2;
        visualization_msgs::Marker markers[size];

        for (int i = 0; i < size; i++) {
            // it sets the frame ID and timestamp.  See the TF tutorials for information on these.
            markers[i].header.frame_id = "map";
            markers[i].header.stamp = ros::Time::now();

            // it sets the namespace and id for this marker.  This serves to create a unique ID
            // any marker sent with the same namespace and id will overwrite the old one
            markers[i].ns = "basic_shapes";
            markers[i].id = i;

            // it sets the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
            if (i % 2 == 0) markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
            else
                markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            //
            //it sets the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
            markers[i].action = visualization_msgs::Marker::ADD;
            //it sets the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            if (i % 2 == 0) {
                markers[i].pose.position.x = P_minus_z_detObject_in_map[0];
                markers[i].pose.position.y = P_minus_z_detObject_in_map[1];
                markers[i].pose.position.z = P_minus_z_detObject_in_map[2];
            } else {
                markers[i].pose.position.x = P_minus_z_detObject_in_map[0];
                markers[i].pose.position.y = P_minus_z_detObject_in_map[1];
                markers[i].pose.position.z = P_minus_z_detObject_in_map[2] + 10;
            }

            markers[i].pose.orientation.x = r_quat.x();
            markers[i].pose.orientation.y = r_quat.y();
            markers[i].pose.orientation.z = r_quat.z();
            markers[i].pose.orientation.w = r_quat.w();
            // it sets the scale of the marker -- 1x1x1 here means 1m on a side
            if (i % 2 == 0) {
                float scale = 0.1;
                markers[i].scale.x = scale;
                markers[i].scale.y = scale;
                markers[i].scale.z = scale;
            } else {

                markers[i].scale.x = 5;
                markers[i].scale.y = 5;
                markers[i].scale.z = 5;

            }
            // it sets the color -- be sure to set alpha to something non-zero!
            markers[i].color.r = 153.0f;
            markers[i].color.g = 0.0f;
            markers[i].color.b = 153.0f;
            markers[i].color.a = 0.7;

            if (i % 2 == 0) markers[i].mesh_resource = "package://mesh/toGrasp.stl";
            else markers[i].text = "DETECTED DETAIL";
            markers[i].lifetime = ros::Duration();
        }

        // it publishes the marker
        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                break;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        for (int i = 0; i < size; i++)
            marker_pub.publish(markers[i]);

        cout << "before sending. \n ";
        mPubCloud.publish(pointcloudMsg);
        cout << "after sending. \n ";
        r.sleep();
        // it creates another marker - interactive to have ability to chose option - to start the server
        InteractiveMarkerRos interactiveMarker(g_position_of_the_first_match, r_quat);
        interactiveMarker.server.reset(new InteractiveMarkerServer("test_node_outside_catkin_ws", "", false));
        interactiveMarker.initMenu();
        interactiveMarker.makeMenuMarker("objectPicker");
        interactiveMarker.menu_handler.apply(*interactiveMarker.server, "objectPicker");
        interactiveMarker.server->applyChanges();
        ros::spin();
        interactiveMarker.server.reset();
        r.sleep();
        if (interactiveMarkerRos::shouldISendDetectedObject == 1) std::cout << "start sending a data to robot... \n";
        if (interactiveMarkerRos::shouldISendDetectedObject == 0) {
            std::cout << "you don`t want to send a data to robot...";
            exit(0);
        }
    }
}

//void opencv::savePointCloud()
//{
//    // Object to write in file 
//    ofstream file_obj; 
//    
//     // Opening file in append mode 
//    file_obj.open("Input.txt", ios::out); 
//   // Writing the object's data in file 
//    file_obj.write((char*)&current_point_cloud, sizeof(current_point_cloud)); 
//    
//    file_obj.close();
//}

//void opencv::readPointCloud()
//{
//    // Object to read from file 
//    ifstream file_obj; 
//  
//    // Opening file in input mode 
//    file_obj.open("Input.txt", ios::in); 
//    
//    // Reading from all lines of the file
//    while(file_obj.read((char*)&current_point_cloud, sizeof(current_point_cloud)))
//{cout << "reading \n";}
//    sl::float4 s;
// cout << "after reading \n";   
//    current_point_cloud.getValue(400, 400, &s);
//     cout << "after reading2 \n";  
//    cout << s.x << " " << s.y << " " << s.z << " " << s.w << endl;
//
////    if(&pointCloudToRead)
////    {
////        cout << "Point Cloud is not empty \n";
////        current_point_cloud = pointCloudToRead;
////    }
////    
//    file_obj.close();
//}

opencv::opencv() {
    //    VideoCapture cap;
    //    cout << "first get \n";
    //    getXYZD(400, 400);
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::HD720; // Use HD1080 video mode
    init_params.camera_fps = 30; // Set fps at 30
    init_params.depth_mode = DEPTH_MODE::ULTRA; // Use ULTRA depth mode
    init_params.coordinate_units = UNIT::CENTIMETER;
    init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_params.depth_minimum_distance = 15; // Set the minimum depth perception distance to 15cm

    int mapScaleCoef = 2; // parameter to scale a map size
    //    cv::Mat map(500 * mapScaleCoef, 500 * mapScaleCoef, CV_8UC3, Scalar(0,0,0));
    //// Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS)
        exit(-1);
    //    readPointCloud(); // download current point from file if not zero
    //    cout << "second get \n";
    //    getXYZD(400, 400);
    //    cout << "is everything ok? Yes \n";
}

/*!
 * \brief get 3d from 2d point
 *
 */
void opencv::getXYZD(int x, int y, Eigen::Vector3d & input_vec) {

    sl::float4 s;
    current_point_cloud.getValue(x, y, &s);
    cout << s.x << " " << s.y << " " << s.z << " " << s.w << " XYZ of the point. " << endl;
    while (s.x != s.x || s.y != s.y || s.z != s.z) {
        x++;
        y++;
        current_point_cloud.getValue(x, y, &s);
        cout << s.x << " " << s.y << " " << s.z << " " << s.w << " XYZ of the new point. " << endl;
    }
    input_vec[0] = s.x;
    input_vec[1] = s.y;
    input_vec[2] = s.z;
}

/*!
 * \brief save 3d from 2d point
 *
 */
void opencv::saveXYZD(int x, int y) {
    ofstream ofs("saveCenterObjectData.txt");
    sl::float4 s;
    current_point_cloud.getValue(x, y, &s);
    ofs << s.x << " " << s.y << " " << s.z << " " << s.w << endl;
    cout << s.x << " " << s.y << " " << s.z << " " << s.w << " saving completed." << endl;
    //    xyzD_vec.push_back(s.x);
    //    xyzD_vec.push_back(s.y);
    //    xyzD_vec.push_back(s.z);
    //    xyzD_vec.push_back(s.w);
    //    for (std::vector<double>::const_iterator i = xyzD_vec.begin(); i != xyzD_vec.end(); ++i)
    //    std::cout << *i << ' ';
    cout << " saved. \n";

}

/*!
 * \brief it saves a chosen image and changes ROI 
 */
void opencv::runopencv(cv::Rect & rect) {

    cv::Mat cv_image;

    // Create a window
    static const string kWinName = "select image to detect an object by pressing SPACE";
    namedWindow(kWinName, WINDOW_NORMAL);
    int circleRadius = 100;
    bool should_video_stream_run = true;
    while (should_video_stream_run) // display image 
    {
        std::cout << " push SPACE to save an image or push ESC to continue the program \n";

        sl::Mat image_zed(zed.getCameraInformation().camera_resolution, MAT_TYPE::U8_C4);
        if (zed.grab() == ERROR_CODE::SUCCESS) {

            zed.retrieveImage(image_zed, VIEW::LEFT); // it gets the left image of ZED stereo camera

        }

        current_image_cv = slMat2cvMat(image_zed);

        cvtColor(current_image_cv, current_image_grayscale_cv, cv::COLOR_BGR2GRAY);

        imshow(kWinName, current_image_grayscale_cv); // it displays a gray image

        if (waitKey(30) == 32) // it saves an image + creates a point cloud then SPACE is pushed
        {

            opencv::updatePointCloud();

            needToSaveAChosenImage = true;
            rect = selectRect(current_image_cv);
            cout << current_image_cv.size() << endl;

            // it displays selected ROI
            rectangle(current_image_cv, rect, Scalar(255, 0, 0), 2, 1);

            // it displays a gray image
            imshow(kWinName, current_image_cv);

            // it saves grayscale image for object detection
            imwrite("saved_images/image.png", current_image_grayscale_cv);

            // then SPACE is pushed it returns to the image selection
            waitKey(0);
        }
        if (waitKey(30) == 27)// push ESC to finish image selection & to start object detection algorithm
        {
            should_video_stream_run = false;
        }
    }
}

/*!
 * \brief this method checks whether a point inside the circle or not
 * \param[in] circle_x x coordinate of a circle
 * \param[in] circle_y y coordinate of a circle
 * \param[in] rad radius of a circle
 * \param[in] x, y coordinates of an object`s center
 * \return if point was inside then the method returns true
 */
bool opencv::isInside(double circle_x, double circle_y, int rad, double x, double y) {
    //    cout << circle_x << " " << circle_y << "   " << x << " " << y << "end" << endl;
    // Compare radius of circle with distance
    // of its center from given point
    if ((x - circle_x) * (x - circle_x) +
            (y - circle_y) * (y - circle_y) <= rad * rad)
        return true;
    else
        return false;
}

cv::Mat opencv::slMat2cvMat(sl::Mat & input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1;
            break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2;
            break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3;
            break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4;
            break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1;
            break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2;
            break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3;
            break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4;
            break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM::CPU));
}

/*!
 * \brief this method changes format or type of the number
 */
string opencv::type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
        case CV_8U: r = "8U";
            break;
        case CV_8S: r = "8S";
            break;
        case CV_16U: r = "16U";
            break;
        case CV_16S: r = "16S";
            break;
        case CV_32S: r = "32S";
            break;
        case CV_32F: r = "32F";
            break;
        case CV_64F: r = "64F";
            break;
        default: r = "User";
            break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}




