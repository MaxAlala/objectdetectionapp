#include "apps_utils.h"

#include <iostream>
using namespace std;
void objectPoseControlsHelp()
{
  std::cout << "Object pose controls"<< std::endl;
  std::cout << "[j-l-k-i-u-o] translate the model along the X-Y-Z axis " << std::endl;
  std::cout << "[a-s-q-e-z-w] handle the rotation through axis-angle notation " << std::endl;  
}

    /*!
    * \brief function to change rotation and translation vectors.
    * \param[out] r_vec rotation vector
    * \param[out] t_vec translation vector
    * \param[in] key pressed button
    * \param[in] t_inc translation increment, default ==  t_inc = 0.01
    * \param[in] r_inc rotation increment, default ==  r_inc = 0.01
    **/

void parseObjectPoseControls ( int key ,cv::Mat &r_vec, cv::Mat &t_vec,
                               double r_inc, double t_inc )
{
  switch( key )
  {
    case 'a':
    case 'A':
      r_vec.at<double>(1,0) += r_inc;
      cout << r_vec << " " << t_vec << endl;
      break;
    case 's':
    case 'S':
      r_vec.at<double>(1,0) -= r_inc;
      cout << r_vec << " " << t_vec << endl;
      break;
    case 'w':
    case 'W':
      r_vec.at<double>(0,0) += r_inc;
      cout << r_vec << " " << t_vec << endl;
      break;
    case 'z':
    case 'Z':
      r_vec.at<double>(0,0) -= r_inc;
      cout << r_vec << " " << t_vec << endl;
      break;
    case 'q':
    case 'Q':
      r_vec.at<double>(2,0) += r_inc;
      cout << r_vec << " " << t_vec << endl;
      break;
    case 'e':
    case 'E':
      r_vec.at<double>(2,0) -= r_inc;
      cout << r_vec << " " << t_vec << endl;
      break;
    case 'i':
    case 'I':
      t_vec.at<double>(1,0) -= t_inc;
      cout << r_vec << " " << t_vec << endl;
      break;
    case 'm':
    case 'M':
      t_vec.at<double>(1,0) += t_inc;
      cout << r_vec << " " << t_vec << endl;
      break;
    case 'j':
    case 'J':
      t_vec.at<double>(0,0) -= t_inc;
      cout << r_vec << " " << t_vec << endl;
      break;
    case 'k':
    case 'K':
      t_vec.at<double>(0,0) += t_inc;
      cout << r_vec << " " << t_vec << endl;
      break;
    case 'u':
    case 'U':
      t_vec.at<double>(2,0) -= t_inc;
      cout << r_vec << " " << t_vec << endl;
      break;
    case 'o':
    case 'O':
      t_vec.at<double>(2,0) += t_inc;
      cout << r_vec << " " << t_vec << endl;
      break;
    default:
      break;
  }
}
