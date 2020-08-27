#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vector>

void change_fromKukaBox_to_Rviz_coordinate(std::vector<double>& position)
{
    double xsaver = position[0]; //save x
    position[0] = position[1];
    position[2] = -position[2];
    position[1] = xsaver; 
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
  // here write coordinates in the arm base
  std::vector<double > world_vector;
  world_vector.push_back(-100);
  world_vector.push_back(100);
  world_vector.push_back(-100);
  std::vector<double> camera_coordinates{28, 0, 46};
    std::vector<double > camera_vector;
  camera_vector.push_back(camera_coordinates[0]);
  camera_vector.push_back(camera_coordinates[1]);
  camera_vector.push_back(camera_coordinates[2]);
//  change_fromKukaBox_to_Rviz_coordinate(world_vector);
//  change_fromKukaBox_to_Rviz_coordinate(camera_vector);
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Transform transform2;
  ros::Rate rate(10.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(world_vector[0], world_vector[1], world_vector[2]) );
    transform2.setOrigin( tf::Vector3(camera_vector[0], camera_vector[1], camera_vector[2]) );
  tf::Quaternion q;
  q.setRPY(0, 0, -1.570795);
  
//  q.setRPY(0, 0, 0);
//    transform.setRotation( q );
//      q.setRPY(3.14159, 0, 0);
    transform2.setRotation( q );
//    
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map" , "world"));
    br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "map" , "camera"));
    rate.sleep();
  }
  return 0;
};

 
