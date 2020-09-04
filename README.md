
install CUDA 10 == https://developer.nvidia.com/cuda-10.0-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1604&target_type=deblocal
install ZED == https://www.stereolabs.com/docs/installation/linux/
 install ROS kinetic full package == http://wiki.ros.org/kinetic/Installation/Ubuntu
install java == 
sudo apt-get update
sudo apt-get install default-jre
sudo apt-get install default-jdk
install netbeans ==
https://netbeans.apache.org/download/index.html
install c++ addition == add netbeans 8.2 plugin portal in Plugins->settings
available plugins ⇒ c++ install
sudo apt-get install build-essential cmake libeigen3-dev libdime-dev libdime1 libglew-dev libglew1.10 libglm-dev libglfw3-dev
install eigen libarry == https://gitlab.com/libeigen/eigen
cd eigen
mkdir build 
cmake ..
sudo make install
Follow this guide to install Ceres Solver
Click here to download OpenMesh
install it == in a main folder
mkdir build
dc build 
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install

install pcl library its and dependencies( don't install  java and eigen libraries == already were installed)
https://askubuntu.com/questions/916260/how-to-install-point-cloud-library-v1-8-pcl-1-8-0-on-ubuntu-16-04-2-lts-for 
install opencv 3.4 with opencv_contrib
cd ~/d2co/libsForD2co
mkdir opencv
cd opencv
download here opencv-3.4 and opencv_contrib-3.4 https://github.com/opencv/opencv_contrib/tree/3.4
https://github.com/opencv/opencv/tree/3.4

mkdir release
cd release
cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=~/d2co/libsForD2co/opencv/install -D OPENCV_EXTRA_MODULES_PATH=~/d2co/libsForD2co/opencv/opencv_contrib-3.4/modules/ ~/d2co/libsForD2co/opencv/opencv-3.4.0/

     additional example http://www.codebind.com/cpp-tutorial/install-opencv-ubuntu-cpp/
cd ~/d2co
git clone https://github.com/MaxAlala/objectdetectionapp.git
cd objectdetectionapp
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j 4

cd d2co
catkin_create_pkg mesh
put here AX-01b_bearing_box.stl
sudo gedit ~/.bashrc
add this to the end ==
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/username/d2co/mesh/
TO LAUNCH PROJECT:
roscore
rosrun rviz rviz
cd ~/d2co/objectdetectionapp/bin
./frame_tf_broadcaster
./detect_object -m fileHD.yml  -c test_images/test_camera_calibHD.yml -d saved_images -o VIDEO -s 1






