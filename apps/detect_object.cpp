#include <cstdio>
#include <string>
#include <sstream>
#include <algorithm>
#include <map>
#include <boost/program_options.hpp>
#include "boost/filesystem.hpp"
#include <opencv2/rgbd.hpp>
#include "cv_ext/cv_ext.h"
#include "raster_object_model3D.h"
#include "raster_object_model2D.h"
#include "chamfer_matching.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "apps_utils.h"
#include "InteractiveMarkerRos.h"
////////
#include <ros/ros.h>
#include <opencv.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <stdlib.h>

/**
 * @file detect_object.cpp
 * @brief in this application : 
 *  1) you take an image and chose its ROI(region of interest) to specify area to improve speed and d2c0 algorithm detection quality)
 *  2) algorithm works, detects the best matching object and displays it in a window
 *  3) you can accept a detected object then opencv object will create two RVIZ Markers(interactive and not)using orientation and location, pointcloud
 *  also opencv object will calculate a tool orientation and set it to the global variable to send it to the robot
 *  4) if u deny an detected object, u can repeat an attempt 2 more times. Also u can finish the program and start all from beginning
 *  
 */

namespace po = boost::program_options;
namespace filesystem = boost::filesystem;
using namespace std;
using namespace cv;
using namespace cv_ext;

/*!
 * \brief AppOptions contains options - parameters to choose
 * 
 *
 **/
struct AppOptions {
    
    /*!
     * \brief
     *  models_filename == output of the generate_models
     **/
    string models_filename;
    
    /*!
     * \brief 
     *camera calibration file
     **/
    string camera_filename;
    
    /*!
     * \brief 
     * where to save a chosen image.
     **/
    string input_image_dir;
    string algorithm_type;
    string output_type;
    double scale_factor, match_threshold;
    bool has_roi = false;

    /*!
     * \brief 
     * region of interest. It saves computations and detection quality.
     **/
    cv::Rect roi;

    int top_boundary, bottom_boundary, left_boundary, rigth_boundary;
};

/*!
 * \brief parses command line and adds the help command
 * @param[out] options takes input data
 **/
void parseCommandLine(int argc, char **argv, AppOptions &options) {
    
    /*!first parameter always is the name of an application*/
    string app_name(argv[0]);

    // default values
    options.scale_factor = 1.0;
    options.match_threshold = 2.5;
    options.algorithm_type = "DCM";
    options.output_type = "VIDEO";
    options.top_boundary = options.bottom_boundary =
    options.left_boundary = options.rigth_boundary = -1;
    
    /*!OD == is great boost parser of input data; example == https://pastebin.com/yBmfCgvr */
    po::options_description desc("OPTIONS");
    desc.add_options()
            // help outputs all descriptions + parameters
            ("help,h", "Print this help messages")
            ("models_filename,m", po::value<string > (&options.models_filename)->required(), "Precomputed models file")
            ("camera_filename,c", po::value<string > (&options.camera_filename)->required(), "A YAML file that stores all the camera parameters (see the PinholeCameraModel object)")
            ("input_image_dir,d", po::value<string > (&options.input_image_dir)->required(), "input_image_dir")
            ("scale_factor,s", po::value<double> (&options.scale_factor), "Scale factor [1]")
            ("match_threshold,t", po::value<double > (&options.match_threshold), "Match threshold [2.5]")
            ("algorithm_type,a", po::value<string> (&options.algorithm_type), "Used algorithm, options: [DCM], OCM")
            ("output_type,o", po::value<string> (&options.output_type), "Output type, options: [VIDEO], FILE, STREAMING")

            ("tb", po::value<int> (&options.top_boundary), "Optional region of interest: top boundary ")
            ("bb", po::value<int> (&options.bottom_boundary), "Optional region of interest: bottom boundary ")
            ("lb", po::value<int> (&options.left_boundary), "Optional region of interest: left boundary ")
            ("rb", po::value<int> (&options.rigth_boundary), "Optional region of interest: rigth boundary");
    
    // dictionary of input parameters
    po::variables_map vm;

    try { // infill vm
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            cout << "USAGE: " << app_name << " OPTIONS"
                    << endl << endl << desc;
            exit(EXIT_SUCCESS);
        }
        
        // can call some function before putting data in OD options
        po::notify(vm);
    }// if required option wasn't called
    catch (boost::program_options::required_option& e) {
        cerr << "ERROR: " << e.what() << endl << endl;
        cout << "USAGE: " << app_name << " OPTIONS"
                << endl << endl << desc;
        exit(EXIT_FAILURE);
    } catch (boost::program_options::error& e) {
        cerr << "ERROR: " << e.what() << endl << endl;
        cout << "USAGE: " << app_name << " OPTIONS"
                << endl << endl << desc;
        exit(EXIT_FAILURE);
    }

    std::transform(options.algorithm_type.begin(),
            options.algorithm_type.end(),
            options.algorithm_type.begin(), ::toupper);

    std::transform(options.output_type.begin(),
            options.output_type.end(),
            options.output_type.begin(), ::toupper);

    if (options.algorithm_type.compare("DCM") &&
            options.algorithm_type.compare("OCM")) {
        cerr << "Unrecognized algorithm type (" << options.algorithm_type << ")" << endl;
        exit(EXIT_FAILURE);
    }
    
    // -1 if was not found, 0 if was found
    if (options.output_type.compare("VIDEO") &&
            options.output_type.compare("FILE")) {
        cerr << "Unrecognized output type (" << options.output_type << ")" << endl;
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char **argv) {

    // it starts ros node
    ros::init(argc, argv, "test_node_outside_catkin_ws");
    opencv oc;
    cv::Rect roi;

    // it takes an image and take ROI, it puts it in a directory by pushing space
    oc.runopencv(roi);

    // it takes parameter through a command line
    AppOptions options;

    // it infills options
    parseCommandLine(argc, argv, options);

    cv::Mat r_vec, t_vec;

    // it works with image, need for rasterization object
    cv_ext::PinholeCameraModel cam_model;

    // it adds camera calibration data
    cam_model.readFromFile(options.camera_filename);

    // scale factor == size of the image
    cam_model.setSizeScaleFactor(options.scale_factor);
    int img_w = cam_model.imgWidth(), img_h = cam_model.imgHeight();
    // region of interest == to save time 
    /**
     * @brief Set an image region of interest (RoI). 
     *        
     * 
     * Define a new image that is a rectangular portion of the original image. 
     * If the region of interest is enabled (see enableRegionOfInterest()) the PinholeCameraModel 
     * object will works with a new image plane represented by the provided RoI. For instance, 
     * the point projection operations will provided outputs with a 2D offset defind 
     * by top-left corner of the rectangle that defines the region of interest.
     * The region of interest should have non zero size, and it should be inside the image.
     * \note The region of interest set with this function refers to the original, not scaled, image 
     *       (see origImgWidth() and origImgHeight() to query the original image size): 
     *       to query the current (possibly scaled) RoI, used regionOfInterest().
     *       If the scale is changed, the region of interest changes accordling.
     * \warning The RoI is NOT enabled by default: you should explicitly call enableRegionOfInterest(true) 
     *          to enable it
     */
    bool has_roi = false;

    // OLD ROI data received from console
    //    if (options.top_boundary != -1 || options.bottom_boundary != -1 ||
    //            options.left_boundary != -1 || options.rigth_boundary != -1) {
    //        //tl == top-left br == bottom-right
    //        Point tl(0, 0), br(img_w, img_h);

    //        if (options.top_boundary != -1) tl.y = options.top_boundary;
    //        if (options.left_boundary != -1) tl.x = options.left_boundary;
    //        if (options.bottom_boundary != -1) br.y = options.bottom_boundary;
    //        if (options.rigth_boundary != -1) br.x = options.rigth_boundary;

    has_roi = true;
    //        roi = cv::Rect(tl, br);
    cam_model.setRegionOfInterest(roi);
    cam_model.enableRegionOfInterest(true);
    roi = cam_model.regionOfInterest();


    cout << "Loading precomputed models from file : " << options.models_filename << endl;
    cout << "Loading camera parameters from file : " << options.camera_filename << endl;
    cout << "Loading input images from directory : " << options.input_image_dir << endl;
    cout << "Scale factor : " << options.scale_factor << endl;
    cout << "Algorithm type : " << options.algorithm_type << endl;
    cout << "Output type : " << options.output_type << endl;
    cout << "Match threshold : " << options.match_threshold << endl;
    if (has_roi)
        cout << "Region of interest : " << roi << endl;

    cout << "Loading model ...." << std::flush;

    // class to work with 3d model objects
    RasterObjectModel3DPtr obj_model_ptr(new RasterObjectModel3D());
    RasterObjectModel3D &obj_model = *obj_model_ptr;

    // it sets cam model
    obj_model.setCamModel(cam_model);

    // it downloads many models with different points of view 
    obj_model.loadPrecomputedModelsViews(options.models_filename);

    const vector< cv::Point3f > depthVec;

    cout << "depth_values" << obj_model.vis_d_pts_.size() << endl;

    // DT == distance map == map with min distance from every pixel to edge pixel of rastered model
    DistanceTransform dc;

    /**
     * @brief Set the distance threshold used in the distance transform
     * Pixel with distance to the closes edgel greater than thresh , are set to thresh.
     */
    dc.setDistThreshold(30);
    /**
     * @brief Enable/disable parallelism 
     * 
     * enable If true, some algorithms are run in a parallel region,
     *                   otherwise they run as a single thread
     */
    dc.enableParallelism(true);

    // edge detector
    CannyEdgeDetectorUniquePtr edge_detector_ptr(new CannyEdgeDetector());
    edge_detector_ptr->setLowThreshold(40);
    edge_detector_ptr->setRatio(2);
    edge_detector_ptr->enableRGBmodality(true);
    // it puts edge detector to find DC edges of input img
    dc.setEdgeDetector(std::move(edge_detector_ptr));
    // in DT there are 60 bins of different orientations
    const int num_directions = 60, increment = 4;
    const double tensor_lambda = 6.0;
    const bool smooth_tensor = false;

    // here you can chose algorithm. But currently only DCM works
    DirectionalChamferMatching dcm;
    OrientedChamferMatching ocm;
    // if found == !0 == 1
    if (!options.algorithm_type.compare("DCM")) {
        // add our object of 3d model
        dcm.setTemplateModel(obj_model_ptr);
        dcm.setNumDirections(num_directions); // num of bins
        dcm.enableParallelism(true);
        dcm.setupExhaustiveMatching();
    } else if (!options.algorithm_type.compare("OCM")) {
        ocm.setNumDirections(num_directions);
        ocm.setTemplateModel(obj_model_ptr);
        ocm.enableParallelism(true);
        ocm.setupExhaustiveMatching();
    }

    // it specifies directory where it can download images
    filesystem::path images_path(options.input_image_dir);
    filesystem::directory_iterator end_itr;
    vector< TemplateMatch > matches;

    Eigen::Vector4d position_of_the_first_match_in_world;
    Eigen::Quaterniond q_of_the_first_match;
    // it finds all images in the specified directory
    for (filesystem::directory_iterator itr(images_path); itr != end_itr; ++itr) {//if img save path
        if (!filesystem::is_regular_file(itr->path()))
            continue;

        // if not directory save path
        string current_file = itr->path().string();

        cv::Mat src_img = imread(current_file, cv::IMREAD_COLOR), input_img;

        if (src_img.empty())
            continue;

        // it sets camera models width and height
        cv::resize(src_img, src_img, cv::Size(img_w, img_h));

        if (has_roi)
            input_img = src_img(roi); // roi == rectangle area
        else
            input_img = src_img;

        if (!options.algorithm_type.compare("DCM")) {
            cv_ext::BasicTimer timer;
            ImageTensorPtr dst_map_tensor_ptr;
            // it computes many maps of different orientation
            dc.computeDistanceMapTensor(input_img, dst_map_tensor_ptr, num_directions, tensor_lambda, smooth_tensor);
            dcm.setInput(dst_map_tensor_ptr);
            // it gets 3 best matches 
            dcm.match(3, matches, (int) increment);
            cout << "DCM elapsed time ms : " << timer.elapsedTimeMs() << endl;
        } else if (!options.algorithm_type.compare("OCM")) {
            cv_ext::BasicTimer timer;
            cv::Mat dist_map, dir_map;
            dc.computeDistDirMap(input_img, dist_map, dir_map, num_directions);
            ocm.setInput(dist_map, dir_map);
            ocm.match(3, matches, (int) increment);
            cout << "OCM elapsed time ms : " << timer.elapsedTimeMs() << endl;
        }

        int i_m = 0;

        TemplateMatch first_match;

        // it takes three best matches, u chose one of them
        for (auto iter = matches.begin(); iter != matches.end(); iter++, i_m++) {
            TemplateMatch &match = *iter;

            vector<Point2f> project_real_pts;

            vector<Point2f> project_calibration_pts;

            // it gets 3d model points from database
            const vector<Point3f> &pts = obj_model.getPrecomputedPoints(match.id);

            const vector<Point3f> &pts2 = obj_model.getPrecomputedPoints(match.id);

            // the projector consumes camera model + transformation = r & t + 3d points,
            // output is 2d vector 
            cv_ext::PinholeSceneProjector proj(obj_model.cameraModel());
            // from 3d to 2d
            //            match.get
            proj.setTransformation(match.r_quat, match.t_vec);

            cv_ext::PinholeSceneProjector proj2(obj_model.cameraModel());

            Eigen::Matrix<double, 3, 3> R_detObject_in_map;

            // 0.3490658504 rad == 20 grads
            // = 0.5235987756 rad == 30 grads
            double errorY = -0.3490658504;

            double errorZ = 0.5235987756;

            Eigen::Matrix<double, 3, 3> R_d2co_in_map;

            // it describes the frame d2co relative to the frame map 
            R_d2co_in_map = Eigen::AngleAxisd(-3.1415 / 2.0, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(3.1415, Eigen::Vector3d::UnitX());

            // it fixes orientation error by calibration == rotate an object a bit to calibrate it
            Eigen::Matrix<double, 3, 3> error_rot;

            error_rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(0.5235987756 / 2, Eigen::Vector3d::UnitX()); // 30 grads

            // it fixes orientation error by calibration == rotate an object a bit to calibrate it
            Eigen::Matrix<double, 3, 3> error_rot2;

            error_rot2 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(-0.5235987756 / 2, Eigen::Vector3d::UnitX());

            cout << "ROTATION MATRIX AROUND Z IS" << R_d2co_in_map;

            // it describes the frame of the detected object (described relative to d2c0 frame) relative
            // to the map frame + some little rotations     
            // R_Obj_in_map = R_error * R_D2co_in_map * R_Obj_in_d2c0
            R_detObject_in_map = error_rot * R_d2co_in_map * match.r_quat.toRotationMatrix();

            cout << "RESULTED MATRIX " << R_detObject_in_map;

            Eigen::Quaterniond rotated_quaternion(R_detObject_in_map);

            Eigen::Vector3d ea = R_d2co_in_map.eulerAngles(2, 1, 0);

            cout << "ZYX are " << ea << "\n";

            cout << "r_quat_real " << match.r_quat.coeffs() << endl;

            Eigen::Vector3d t_vec(match.t_vec);

            cout << " id is " << match.id << " i_m ==" << i_m << " distance ==" << match.distance << " Mat ==" << R_d2co_in_map << " Vector ==" << t_vec << endl;

            double yaw = 0.0732849;
            double pitch = 3.02223;
            double roll = 1.57988;
            std::cout << "start: \n " << match.r_quat.toRotationMatrix().eulerAngles(2, 1, 0) << "DETECTED OBJECTS ORIENTATION. \n";

            Eigen::Quaternion<double, Eigen::DontAlign> r_quat(0.0679299, -0.707034, 0.703714, 0.016597);

            cout << "\n beginning: " << r_quat.coeffs() << " coeffs \n";

            auto euler = r_quat.toRotationMatrix().eulerAngles(2, 1, 0);
            std::cout << "Euler from quaternion in roll, pitch, yaw" << std::endl << euler << std::endl;
            proj.projectPoints(pts, project_real_pts);

            Point2f top_left(project_real_pts[0]);
            Point2f top_right(project_real_pts[0]);
            Point2f bottom_left(project_real_pts[0]);
            Point2f bottom_right(project_real_pts[0]);

            cv::Mat display = src_img.clone(), draw_img;

            if (has_roi) {
                cv::Point dbg_tl = roi.tl(), dbg_br = roi.br();
                dbg_tl.x -= 1;
                dbg_tl.y -= 1;
                dbg_br.x += 1;
                dbg_br.y += 1;
                cv::rectangle(display, dbg_tl, dbg_br, cv::Scalar(255, 255, 255));

                // roi == rectangle
                draw_img = display(roi);
            } else
                draw_img = display;

            // DEPRECATED
            top_left = project_real_pts[0];
            top_right = project_real_pts[0];
            bottom_left = project_real_pts[0];
            bottom_right = project_real_pts[0];
            int x_sum = 0;
            int y_sum = 0;

            /////////////////////////////////// old version == top left == bottom right == DEPRECATED       
            for (int i = 1; i < project_real_pts.size(); i++) {

                if (project_real_pts.at(i).x < top_left.x && project_real_pts.at(i).y < top_left.y)
                    top_left = project_real_pts.at(i);
                if (project_real_pts.at(i).x > top_right.x && project_real_pts.at(i).y < top_left.y)
                    top_right = project_real_pts.at(i);
                if (project_real_pts.at(i).x < bottom_left.x && project_real_pts.at(i).y > bottom_left.y)
                    bottom_left = project_real_pts.at(i);
                if (project_real_pts.at(i).x > bottom_right.x && project_real_pts.at(i).y > bottom_right.y)
                    bottom_right = project_real_pts.at(i);
            }

            // it finds center of detected object
            for (int i = 0; i < project_real_pts.size(); i++) {
                x_sum += project_real_pts[i].x;
                y_sum += project_real_pts[i].y;
            }

            // a center of the detected point is average x and y + roi displacement
            cv::Point centerOftheDetectedObject(roi.x + x_sum / project_real_pts.size(), roi.y + y_sum / project_real_pts.size());

            ////////////////////////////////////

            //            cv::Point pt3(top_left.x, top_left.y);
            // and its bottom right corner.
            //            cv::Point pt4(bottom_right.x, bottom_right.y);
            //            int height_in_pixels = bottom_right.y - top_left.y;
            //            int width_in_pixels = bottom_right.x - top_left.x;
            //            cv::rectangle(draw_img, pt3, pt4, cv::Scalar(0, 0, 255));

            //            cv::Point centerOftheDetectedObject(roi.x + top_left.x + width_in_pixels / 2, roi.y + top_left.y + height_in_pixels / 2);
            Eigen::Vector3d distance_to_an_object_vector;

            // it returns vector with XYZ of the detected object using zed camera data
            oc.getXYZD(centerOftheDetectedObject.x, centerOftheDetectedObject.y, distance_to_an_object_vector);

            cout << distance_to_an_object_vector << "Distance to an object before rotation \n";

            cout << "calculated center of a detected object is: " << centerOftheDetectedObject.x << " " << centerOftheDetectedObject.y << endl;
            //            cv::rectangle(draw_img, pt3, pt4, cv::Scalar(0, 255, 0));

            // it draws points of the best match
            cv_ext::drawPoints(draw_img, project_real_pts, match.distance > options.match_threshold ? Scalar(0, 255, 0) : Scalar(0, 255, 0));
            //            double focal_length_y = 9.6344770300000005e+02;
            //            double focal_length_x = 9.6785787400000004e+02;
            //            double distance_to_the_object_using_one_camera = focal_length_y / height_in_pixels * height_in_pixels;
            //            double Y = height_in_pixels * distance_to_the_object_using_one_camera / focal_length_y;
            //            double X = height_in_pixels * distance_to_the_object_using_one_camera / focal_length_x;
            //            cout << "calculated y height in pixel is: " << height_in_pixels << endl;
            //            cout << "calculated x width in pixel is: " << width_in_pixels << endl;
            //            cout << "distance to the object using one camera is " << distance_to_the_object_using_one_camera << endl;
            //            cout << "X and Y are " << X << ", " << Y << endl;

            //  deprecated
            if (oc.needToSaveAChosenImage) oc.saveXYZD(centerOftheDetectedObject.x, centerOftheDetectedObject.y);
            cout << distance_to_an_object_vector[0] << " " << distance_to_an_object_vector[1] << " " << distance_to_an_object_vector[2] << " DISTANCE VECTOR \n";
            string text_about_distance_to_an_object = "The distance to the detected object: " + to_string(sqrt(distance_to_an_object_vector[0] * distance_to_an_object_vector[0] + distance_to_an_object_vector[1] * distance_to_an_object_vector[1] + distance_to_an_object_vector[2] * distance_to_an_object_vector[2]));
            cout << text_about_distance_to_an_object << endl;

            //it draws center of the detected object
            circle(display, centerOftheDetectedObject, 2, Scalar(0, 0, 255), 5);
            cv::putText(display, //target image
                    text_about_distance_to_an_object.c_str(), //text
                    cv::Point(30, 100), //top-left position
                    cv::FONT_HERSHEY_DUPLEX,
                    0.5,
                    CV_RGB(162, 41, 162), //font color
                    1);

            // this text about continuing or finishing the program
            string text_about_about_should_program_run_farther = "Please, input Y or N in the console, Y == YES, if you want to continue the program, N == NO, if u don`t want to continue the program";
            cv::putText(display, //target image
                    text_about_about_should_program_run_farther, //text
                    cv::Point(30, 130), //top-left position
                    cv::FONT_HERSHEY_DUPLEX,
                    0.5,
                    CV_RGB(162, 41, 162), //font color
                    1);

            //it shows camera coordinates in map frame
            Eigen::Vector4d camera_coordinates_in_map;
            camera_coordinates_in_map << 28, 0, 46, 1;

            // it describes the frame d2co relative to the frame map == R_d2co_in_map
            Eigen::Matrix3d d2co_orientation_in_map; // start orientation before applying founded euler angles
            d2co_orientation_in_map <<
                    0, -1, 0,
                    -1, 0, 0,
                    0, 0, -1
                    ;

            // it describes the frame CAM relative to the frame d2co == R_cam_in_d2co
            Eigen::Matrix4d camera_transformation_in_d2co;
            camera_transformation_in_d2co <<
                    1, 0, 0, 0,
                    0, -1, 0, 0,
                    0, 0, -1, 0,
                    1, 1, 1, 0
                    ;

            // it describes the frame cam relative to the frame map == R_cam_in_map
            Eigen::Matrix4d camera_transformation_in_map; // from camera to d2co
            camera_transformation_in_map <<
                    0, 1, 0, camera_coordinates_in_map(0),
                    -1, 0, 0, camera_coordinates_in_map(1),
                    0, 0, 1, camera_coordinates_in_map(2),
                    1, 1, 1, 0
                    ;


            //            Eigen::Matrix4d camera_transformation_in_Reversedmap; 
            //            camera_transformation_in_Reversedmap <<
            //                    0, 1, 0, -camera_coordinates_in_map(0),
            //                    1, 0, 0, camera_coordinates_in_map(1),
            //                    0, 0, -1, -camera_coordinates_in_map(2),
            //                    1, 1, 1, 0
            //                    ;

            Eigen::Vector4d detected_object_coordinates_in_map;
            //            Eigen::Vector4d detected_object_coordinates_in_reversed_map;
            Eigen::Vector4d detected_object_coordinates_in_camera;
            detected_object_coordinates_in_camera << distance_to_an_object_vector(0), distance_to_an_object_vector(1), distance_to_an_object_vector(2), 1;

            // it describes the transformation d2co relative to the frame map == T_d2co_in_map
            Eigen::Matrix4d transformationD2coInMap;
            transformationD2coInMap << d2co_orientation_in_map(0, 0), d2co_orientation_in_map(0, 1), d2co_orientation_in_map(0, 2), camera_coordinates_in_map(0),
                    d2co_orientation_in_map(1, 0), d2co_orientation_in_map(1, 1), d2co_orientation_in_map(1, 2), camera_coordinates_in_map(1),
                    d2co_orientation_in_map(2, 0), d2co_orientation_in_map(2, 1), d2co_orientation_in_map(2, 2), camera_coordinates_in_map(2),
                    1, 1, 1, 0;

            // P is position vector of the detected object
            // P_detected_object_in_map = T_d2co_in_map * P_det_obj_in_d2co
            detected_object_coordinates_in_map = camera_transformation_in_map * detected_object_coordinates_in_camera;

            cout << detected_object_coordinates_in_map << ": COORDINATES OF DETECTED OBJECT IN WORLD FRAME " << endl;

            if (!options.output_type.compare("VIDEO")) { // if video then display mat
                std::string name_detected_obj = "detected_object";
                //                namedWindow(name_detected_obj, WINDOW_NORMAL);
                cv_ext::showImage(display, "display", true);
                //                cv::imshow(name_detected_obj, display);
                //                waitKey(0); // display while something is pushed
            } else if (!options.output_type.compare("FILE")) {
                size_t found = current_file.find_last_of("/");
                string path = current_file.substr(0, found);
                string filename = current_file.substr(found + 1);
                found = filename.find_last_of(".");
                string basename = filename.substr(0, found);
                string extension = filename.substr(found + 1);
                stringstream sstr;
                sstr << path;
                sstr << "/results/";
                sstr << basename;
                sstr << "_";
                sstr << i_m;
                sstr << ".";
                sstr << extension;
                cout << "Saving result image to: " << sstr.str() << endl;
                imwrite(sstr.str(), display);
            } else {
                cout << "Error: Unknown streaming type!" << endl;
            }
            cout << rotated_quaternion.toRotationMatrix().eulerAngles(2, 1, 0) << ": final angles \n";

            // it prints message: do u want to take the current best match or not? If Yes then program sends saves orientation and position of a det obj.
            char modelWasChoosen = ' ';
            while (modelWasChoosen != 'Y' && modelWasChoosen != 'N') {
                std::cout << "Error invalid input please try again " << std::endl;
                std::cout << "Please enter [Y/N] if u want or not to select current model: ";
                std::cin >> modelWasChoosen;
                modelWasChoosen = toupper(modelWasChoosen);
            }
            std::cout << "u have pressed: " << modelWasChoosen;

            ////// it saves XYZ AND orientation of the first match
            if (modelWasChoosen == 'Y') {
                q_of_the_first_match = rotated_quaternion;

                cout << rotated_quaternion.toRotationMatrix().eulerAngles(2, 1, 0) << ": final angles \n";
                position_of_the_first_match_in_world = detected_object_coordinates_in_map;
                position_of_the_first_match_in_world[0] += 0;
                position_of_the_first_match_in_world[1] += 1;
                position_of_the_first_match_in_world[2] += 0;
            }

            // it prints message: do u want to continue the program or not? If Yes then program sends saved orientation and position to the opencv object and
            // it sends det obj position & orientation to the rviz and also calculate tool`s orientation.
            if (modelWasChoosen == 'Y') {
                std::cout << text_about_about_should_program_run_farther;
                char shouldIStartToSendDetectedObjectToRVIZ; /////////////////// END OF OBJECT DETECTION
                std::cout << "Please enter [Y/N]: ";
                std::cin >> shouldIStartToSendDetectedObjectToRVIZ;

                shouldIStartToSendDetectedObjectToRVIZ = toupper(shouldIStartToSendDetectedObjectToRVIZ);

                while (shouldIStartToSendDetectedObjectToRVIZ != 'Y' && shouldIStartToSendDetectedObjectToRVIZ != 'N') {
                    std::cout << "Error invalid input please try again " << std::endl;
                    std::cout << "Please enter [Y/N]: ";
                    std::cin >> shouldIStartToSendDetectedObjectToRVIZ;
                    shouldIStartToSendDetectedObjectToRVIZ = toupper(shouldIStartToSendDetectedObjectToRVIZ);
                }

                std::cout << "You entered " << shouldIStartToSendDetectedObjectToRVIZ << std::endl;

                if (shouldIStartToSendDetectedObjectToRVIZ == 'N') exit(0);
                //opencv object sends detected object location and point cloud data to the rviz and calculate tool orientation
                if (shouldIStartToSendDetectedObjectToRVIZ == 'Y')
                    oc.runNode(argc, argv, position_of_the_first_match_in_world, q_of_the_first_match);
            }
        }
    }
    return 0;
}
