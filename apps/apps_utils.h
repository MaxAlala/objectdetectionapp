#pragma once

#include "cv_ext/cv_ext.h"

void objectPoseControlsHelp();
    /*!
    * \brief function to change rotation and translation vectors.
    * \param[out] r_vec rotation vector
    * \param[out] t_vec translation vector
    * \param[in] num of key pressed button
    * \param[in] t_inc translation increment, default ==  t_inc = 0.01
    * \param[in] r_inc rotation increment, default ==  r_inc = 0.01
    **/
void parseObjectPoseControls ( int key ,cv::Mat &r_vec, cv::Mat &t_vec,
                               double r_inc = 0.01, double t_inc = 0.01 );
