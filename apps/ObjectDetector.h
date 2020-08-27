/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ObjectDetector.h
 * Author: northlight
 *
 * Created on July 10, 2020, 3:22 PM
 */

#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

namespace po = boost::program_options;
namespace filesystem = boost::filesystem;
using namespace std;
using namespace cv;
using namespace cv_ext;


class ObjectDetector {
public:
    ObjectDetector();
    ObjectDetector(const ObjectDetector& orig);
    virtual ~ObjectDetector();
private:

};

#endif /* OBJECTDETECTOR_H */

