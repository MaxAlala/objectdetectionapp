#ifndef OPENCV_H
#define OPENCV_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iostream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <sl/Camera.hpp>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Geometry>
using namespace std;
using namespace cv;
using namespace sl;
using namespace dnn;

/*!
    \brief this class takes an image, receives detected object position & orientation, sends 2 markers & point cloud to RVIZ
 */
class opencv {
public:
    Camera zed;

    //    /*!
    //     * \brief read point cloud from file 
    //     *
    //     */    
    //    void readPointCloud();
    //
    //    /*!
    //     * \brief save point cloud in the file 
    //     *
    //     */    
    //    void savePointCloud();


    void saveXYZD(int x, int y);
    /*!
     * \brief open zed camera, infill it with some parameters
     *
     */
    opencv();

    /*!
     * \brief it releases zed camera
     *
     */

    ~opencv() {
        zed.close();
    }

    /*!
     * \brief it gets 3d from 2d point using ZED Camera class
     */
    void getXYZD(int x, int y, Eigen::Vector3d&);

    /*!
     * \brief it saves a chosen image and changes ROI 
     */
    void runopencv(cv::Rect & rect);

    /*!
     * \brief it finds equal format
     */
    cv::Mat slMat2cvMat(sl::Mat& input);

    /*!
     * \brief it updates point cloud
     **/
    void updatePointCloud();

    /*!
     * \brief it finds equal format from ZED cam to openCV 
     */
    string type2str(int type);

    /*!
     * \brief this method checks whether a point inside the circle or not
     * \param[in] circle_x x coordinate of a circle
     * \param[in] circle_y y coordinate of a circle
     * \param[in] rad radius of a circle
     * \param[in] x, y coordinates of an object`s center
     * \return if point was inside then the method returns true
     */
    bool isInside(double circle_x, double circle_y, int rad, double x, double y);

    /*!
     * \brief this methods starts a ROS node
     */
    void initNode(int argc, char** argv);


    /*!
     * \brief it sends 2 markers & point cloud to RVIZ also creates a tool orientation for server
     **/
    void runNode(int argc, char** argv, Eigen::Vector4d& position_of_the_first_match, Eigen::Quaterniond& r_quat);
    void fromCMtoM(Eigen::Vector3d& vec);
    void rotateVector(int X_angle, int Y_angle, int Z_angle, Eigen::Vector3d& inputVector);

    //    void change_fromKukaBox_to_Rviz_coordinate(std::vector<double>& position);
    //    void change_fromKukaBox_to_Rviz_coordinate(Eigen::Vector3d& position);
    //    void change_from_KukaBox_to_Rviz_coordinate(sl::float4& point3d);
    //    void change_from_Camera_to_KukaBox_coordinate(sl::float4& point3d);
    //    void change_pointCloud_from_Camera_to_Rviz_through_KukaBox_coordinate(sl::Mat& mat);

    sl::Mat current_depth_map;

    /*!
     * \brief retrieved from ZED cam 3d point cloud 
     **/
    sl::Mat current_point_cloud;

    /*!
     * \brief image received from zed camera
     **/
    cv::Mat current_image_cv;

    /*!
     * \brief image received from zed camera
     **/
    cv::Mat current_image_grayscale_cv;

    /*!
     * \brief flag changes then image was chosen to be saved
     **/
    bool needToSaveAChosenImage = false;
    std::string pointCloudFrameId = "camera";

    /*!
     * \brief class to create a ROS node
     **/
    ros::NodeHandle n;

private:
    string absolutePath = "";

};

#endif // OPENCV_H
