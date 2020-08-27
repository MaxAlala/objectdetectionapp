project( d2co )

cmake_minimum_required(VERSION 3.0)

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Build type automatically set to 'Release'")
    set(CMAKE_BUILD_TYPE "Release" )
else(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Build type: '${CMAKE_BUILD_TYPE}'") 
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        message(STATUS "WARNING: you will get poor performance!")
    endif(CMAKE_BUILD_TYPE STREQUAL "Debug")
endif(NOT CMAKE_BUILD_TYPE)

set (CMAKE_CXX_STANDARD 14)
option(USE_SSE "Build with SSE3+ support" ON)
option(USE_AVX "Build with AVX/AVX2 support" OFF)
option(USE_NEON "Build with NEON support" OFF)
option(USE_CAFFE_TRIPLET "Build with Caffe Triplet CNN support" OFF)
### ZED
option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()
#################################


set(CMAKE_CURRENT_SOURCE_DIR ~/object_detection_app)
# This is to be complinat with OpenCV >= 3.3.1
set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -DOPENCV_TRAITS_ENABLE_DEPRECATED")

#############################ZED CAMERA

    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})

 ##   SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})


################################################
################################################
find_package(OpenMP REQUIRED)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  message(STATUS "Compiling for x86 / x86_64 architectures")
  
  if(USE_SSE)
      add_definitions(-DD2CO_USE_SSE )
      set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -msse3")
  endif(USE_SSE)
    
  if(USE_AVX)
      add_definitions(-DD2CO_USE_AVX )
      set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -mavx2 -mfma -march=core-avx2")
  endif(USE_AVX)
  
  set(EXTRA_CXX_FLAGS "${EXTRA_CXX_FLAGS} -Wall -Wpedantic -Wno-narrowing -Wno-deprecated -std=c++14 -march=nocona")
  
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${OpenMP_CXX_FLAGS} ${EXTRA_CXX_FLAGS}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS} ${EXTRA_CXX_FLAGS} -O3")
  
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(armv7)")
  message(STATUS "Compiling for ARM v7 architectures")
  
  if(USE_NEON)
      add_definitions(-DD2CO_USE_NEON )
  endif(USE_NEON)
  
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${OpenMP_CXX_FLAGS} -std=c++11")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS} -std=c++11 -mcpu=cortex-a53 -mfloat-abi=hard -mfpu=neon-fp-armv8 -mneon-for-64bits -O3")  
else()
  message(STATUS "Unsupported architecture: using default compiler flags")
endif ()

find_library( LIBUUID_LIBRARIES uuid)
#set(OpenCV_DIR "/usr/local")
find_package(OpenCV REQUIRED PATHS /usr/local/share/OpenCV NO_DEFAULT_PATH)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(Eigen3 REQUIRED)
find_package (PCL 1.8 REQUIRED)
set(PCL_LIBRARY_DIRS /home/northlight/Downloads/pcl-pcl-1.8.0/build/lib)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

#add_definitions( ${PCL_DEFINITIONS}
 #                -DTW_NO_LIB_PRAGMA
 #                -DTW_NO_DIRECT3D
 #                -DGLM_FORCE_RADIANS)

find_package(Ceres  REQUIRED)

find_library( DIME_LIBS dime )
find_library( GLEW_LIBS GLEW )
set(OPENMESH_HEADER MeshIO.hh)

find_library( OPENMESHCORE_LIBS OpenMeshCore PATHS "/home/northlight/OpenMesh-3.3/build/Build/lib" )
find_package( Boost COMPONENTS program_options filesystem system REQUIRED )


if(NOT DEFINED EIGEN3_INCLUDE_DIR)
  find_path(EIGEN3_INCLUDE_DIR eigen3)
endif(NOT DEFINED EIGEN3_INCLUDE_DIR)
set(OPENMESH_INCLUDE_DIR /home/northlight/libs/include)
#set(PATH_INCLUDE_OPENCV /usr/local/include )
#SET(OpenCV_INSTALL_PATH /opt/opencv/release)

#SET(OpenCV_INCLUDE_DIRS "${OpenCV_INSTALL_PATH}/include/opencv" "${OpenCV_INSTALL_PATH}/include/opencv2")

#SET(OpenCV_LIB_DIR "${OpenCV_INSTALL_PATH}/lib")

#LINK_DIRECTORIES(${OpenCV_LIB_DIR})

#set(OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect )
add_definitions(${PCL_DEFINITIONS})
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)
message(STATUS "ZED_INCLUDE_DIRS = ${ZED_INCLUDE_DIRS}")
message(STATUS "CUDA_INCLUDE_DIRS= ${CUDA_INCLUDE_DIRS}")
message(STATUS "ZED_LIBRARY_DIR = ${ZED_LIBRARY_DIR}")
message(STATUS "CUDA_LIBRARY_DIRS = ${CUDA_LIBRARY_DIRS}")


#set(PCL_INCLUDE_DIRS /home/northlight/Downloads/pcl-pcl-1.8.0/build/include/pcl-1.8)
#set(VTK_INCLUDE_DIRS /home/northlight/Downloads/pcl-pcl-1.8.0/build/include/vtk-7.1)
#set(OpenCV_INCLUDE_DIRS /usr/local/include /usr/local/include/opencv2 /usr/local/include/opencv)
include_directories( ./include
                      /usr/include
                      /usr/local/include
		      /usr/local/include/opencv2
                      ${OpenCV_INCLUDE_DIRS}
                      ${Boost_INCLUDE_DIRS}
                      ${EIGEN3_INCLUDE_DIR}
		      ${VTK_INCLUDE_DIRS}
                      ${PCL_INCLUDE_DIRS}
                      ${CERES_INCLUDE_DIRS}
                      ${OPENGL_INCLUDE_DIRS}
                      ${GLFW_INCLUDE_DIRS}
		      ${OPENMESH_INCLUDE_DIR}
	              ${PATH_INCLUDE_OPENCV}
/opt/ros/kinetic/include
${CUDA_INCLUDE_DIRS}
${ZED_INCLUDE_DIRS}
)

if(USE_CAFFE_TRIPLET)
  find_library(Caffe_LIBS NAMES caffe HINTS /opt/caffe_triplet/lib)
  include_directories(/opt/caffe_triplet/include)
endif(USE_CAFFE_TRIPLET)                      
#set(PCL_LIBRARY_DIRS /home/northlight/Downloads/pcl-pcl-1.8.0/build/lib)
#set(PCL_LIBRARIES pcl_common;pcl_kdtree;pcl_octree;pcl_search;pcl_sample_consensus;pcl_filters;pcl_io;pcl_features;pcl_registration;pcl_ml;pcl_recognition;pcl_visualization;pcl_segmentation;pcl_people;pcl_outofcore;pcl_surface;pcl_keypoints;pcl_tracking;pcl_stereo;pcl_apps;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so;/usr/lib/libOpenNI.so;/usr/lib libOpenNI2.so;vtkChartsCore;vtkCommonColor;vtkCommonCore;vtksys;vtkCommonDataModel;vtkCommonMath;vtkCommonMisc;vtkCommonSystem;vtkCommonTransforms;vtkCommonExecutionModel;vtkFiltersGeneral;vtkCommonComputationalGeometry;vtkFiltersCore;vtkInfovisCore;vtkFiltersExtraction;vtkFiltersStatistics;vtkImagingFourier;vtkImagingCore;vtkalglib;vtkRenderingContext2D;vtkRenderingCore;vtkFiltersGeometry;vtkFiltersSources;vtkRenderingFreeType;vtkfreetype;vtkzlib;vtkFiltersModeling;vtkImagingSources;vtkInteractionStyle;vtkInteractionWidgets;vtkFiltersHybrid;vtkImagingColor;vtkImagingGeneral;vtkImagingHybrid;vtkIOImage;vtkDICOMParser;vtkmetaio;vtkjpeg;vtkpng;vtktiff;vtkRenderingAnnotation;vtkRenderingVolume;vtkIOXML;vtkIOCore;vtkIOXMLParser;vtkexpat;vtkIOGeometry;vtkIOLegacy;vtkIOPLY;vtkRenderingLOD;vtkViewsContext2D;vtkViewsCore;vtkRenderingContextOpenGL2;vtkRenderingOpenGL2;vtkglew;FLANN::FLANN
#)
aux_source_directory( src D2CO_SRC )
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
link_directories(${PCL_LIBRARY_DIRS})
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")
message(STATUS "PCL_LIBRARY_DIRS = ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_LIBRARIES = ${PCL_LIBRARIES}")
message(STATUS "PCL_COMPONENTS = ${PCL_COMPONENTS}")

message(STATUS "D2CO_SRC = ${D2CO_SRC}")
link_directories(/home/northlight/object_detection_app/lib)
link_directories(/opt/ros/kinetic/lib)
###create cv_ext library
set(CV_EXT src/InteractiveMarkerRos.cpp src/opencv.cpp src/ros_publisher.cpp src/3d_shapes.cpp src/aligned_mat.cpp src/camera_calibration.cpp src/drawing.cpp src/hdr.cpp 
src/image_inspection.cpp src/image_pyramid.cpp src/image_skeleton.cpp src/image_statistics.cpp src/image_tensor.cpp 
src/iterative_pnp.cpp src/pinhole_camera_model.cpp src/serialization.cpp src/stereo_camera.cpp src/debug_tools.cpp)
add_library(cv_ext SHARED ${CV_EXT})


##############################ZED
#include_directories(${GLEW_INCLUDE_DIRS})
#include_directories(${GLUT_INCLUDE_PATH})

#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(/opt/ros/kinetic/lib)
#link_directories(${GLEW_LIBRARY_DIRS})
#link_directories(${GLUT_LIBRARY_DIRS})
#link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
#link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
#ADD_EXECUTABLE(${execName} ${SRC_FILES})

#add_definitions(-std=c++0x -g -O3)

# Add the required libraries for linking:
#TARGET_LINK_LIBRARIES(${execName}
#                        ${ZED_LIBRARIES}
#                        ${SPECIAL_OS_LIBS}
#                        ${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY} )

#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

##############################ZED
target_link_libraries( cv_ext PUBLIC
                       ${Boost_LIBRARIES}
                       ${DIME_LIBS}
                       ${OpenCV_LIBS}
                       ${PCL_LIBRARIES}
                       ${CV_EXT_LIBS}
                       ${CERES_LIBRARIES}
                       ${OPENGL_LIBRARY}
                       ${GLEW_LIBS}
                       ${GLFW_STATIC_LIBRARIES}
                       ${OPENMESHCORE_LIBS}
                       ${ZED_LIBS}


)

set_target_properties(cv_ext PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set_target_properties(cv_ext PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

###create d2co library

add_library(d2co SHARED ${D2CO_SRC})
target_link_libraries( d2co cv_ext
		       ${PCL_LIBRARIES}
		       ${OpenCV_LIBS}
		       -L/usr/local/lib/
                       ${Boost_LIBRARIES}
                       ${DIME_LIBS}
                       ${CERES_LIBRARIES}
                       ${OPENGL_LIBRARY}
                       ${GLEW_LIBS}
                       ${GLFW_STATIC_LIBRARIES}
                       ${OPENMESHCORE_LIBS}
		       ${LIBUUID_LIBRARIES}
### ROS for NODE
                       -I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib
-lroscpp -lrostime -lrosconsole -lroscpp_serialization -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lrosconsole_backend_interface
-linteractive_markers
### ZED CAMERA
#${SPECIAL_OS_LIBS} 
                        ${ZED_LIBS}
#                        ${OPENGL_LIBRARIES}
#                        ${GLUT_LIBRARY}
#                        ${GLEW_LIBRARIES})
)

set_target_properties(d2co PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set_target_properties(d2co PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


#add_executable(cv_ext  ${CV_EXT} )

#add_executable(cv_ext ${CV_EXT} )

#target_link_libraries(  cv_ext d2co  )
#set_target_properties( cv_ext PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# add_executable(test_multiview apps/test_multiview.cpp apps/apps_utils.cpp )
# target_link_libraries( test_multiview
#                        d2co)
# set_target_properties( test_multiview PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(take_image apps/take_image.cpp )
target_link_libraries( take_image
                       d2co 
		       cv_ext)
set_target_properties( take_image PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_executable(test_localization apps/test_localization.cpp apps/apps_utils.cpp )
target_link_libraries( test_localization
                       d2co 
		       cv_ext)
set_target_properties( test_localization PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin) 


add_executable(test_model apps/test_model.cpp apps/apps_utils.cpp )
target_link_libraries( test_model d2co cv_ext)
set_target_properties( test_model PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_executable(test_line2D apps/test_line2D.cpp apps/apps_utils.cpp )
target_link_libraries( test_line2D
                       d2co cv_ext
                       )
set_target_properties( test_line2D PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_executable(test_distance_maps apps/test_distance_maps.cpp apps/apps_utils.cpp )
target_link_libraries( test_distance_maps
                       d2co cv_ext)
set_target_properties( test_distance_maps PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_executable(test_charger_arm apps/test_charger_arm.cpp apps/apps_utils.cpp )
target_link_libraries( test_charger_arm
                       d2co)
set_target_properties( test_charger_arm PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_executable(generate_models apps/generate_models.cpp apps/apps_utils.cpp )
target_link_libraries( generate_models
                       d2co cv_ext)
set_target_properties( generate_models PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_executable(detect_object apps/detect_object.cpp apps/apps_utils.cpp )
target_link_libraries( detect_object
                       d2co cv_ext
                       ${ZED_LIBS}
### ROS for NODE
                       -I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib
-lroscpp -lrostime -lrosconsole -lroscpp_serialization -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lrosconsole_backend_interface
-linteractive_markers


#-I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib
#-lroscpp -lrostime -lrosconsole -lroscpp_serialization
#		       ${OpenCV_LIBS}
                     #  ${SPECIAL_OS_LIBS}
                     # ${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY} )
)

set_target_properties( detect_object PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)





add_executable(frame_tf_broadcaster apps/frame_tf_broadcaster.cpp )
target_link_libraries( frame_tf_broadcaster
                       d2co cv_ext
                    #   ${ZED_LIBS}
### ROS for NODE
                       -I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib
-lroscpp -lrostime -lrosconsole -lroscpp_serialization -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lrosconsole_backend_interface
-linteractive_markers -ltf


#-I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib
#-lroscpp -lrostime -lrosconsole -lroscpp_serialization
#		       ${OpenCV_LIBS}
                     #  ${SPECIAL_OS_LIBS}
                     # ${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY} )
)

set_target_properties( frame_tf_broadcaster PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)


if(USE_CAFFE_TRIPLET)

  add_executable(sphereview_data apps/sphereview_data.cpp )
  target_link_libraries( sphereview_data
                         d2co cv_ext
                         ${Caffe_LIBS})
  set_target_properties( sphereview_data PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

  add_executable(cnn_3dobj_classify apps/cnn_3dobj_classify.cpp )
  target_link_libraries( cnn_3dobj_classify
                         d2co cv_ext
                         ${Caffe_LIBS})
  set_target_properties( cnn_3dobj_classify PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

  add_executable(test_cnn_triplet apps/test_cnn_triplet.cpp )
  target_link_libraries( test_cnn_triplet
                         d2co cv_ext
                         ${Caffe_LIBS})
  set_target_properties( test_cnn_triplet PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
  

endif(USE_CAFFE_TRIPLET)   
