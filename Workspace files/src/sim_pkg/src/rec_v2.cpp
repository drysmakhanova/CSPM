#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/Bool.h"
#include "sim_pkg/Position.h"
#include "sim_pkg/ResetMotors.h"
#include "geometry_msgs/Point.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "std_msgs/String.h"
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

sim_pkg::Position p1;
std_msgs::Bool t;
bool checkCol; 
bool proceed;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rec_v2");
  ros::NodeHandle n;
  
  std_msgs::String str;
  str.data = "Collision detected";
  
  ros::Publisher collision = n.advertise<std_msgs::String>("/collision", 10);
  ros::Publisher joints = n.advertise<sim_pkg::Position>("/joints_phys", 1000); 
  
  rosbag::Bag bag;
  bag.open("main.bag", rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("Collsion"));
  topics.push_back(std::string("Points"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  foreach(rosbag::MessageInstance const m, view) {
        std_msgs::Bool::ConstPtr c = m.instantiate<std_msgs::Bool>();
        
        if (c != NULL){
            collision.publish(str);
            break;
         }
    }
    
  ros::Rate loop_rate(10);
  ros::Rate loop_rate2(1);

  int position[3] = {0,0,0};
  	    p1.id = {1, 2, 3};
            p1.position = {position[0], position[1], position[2]};
            
  while (ros::ok())
  {	
    
  foreach(rosbag::MessageInstance const m, view) {
        geometry_msgs::Point::ConstPtr p = m.instantiate<geometry_msgs::Point>();
        
        if (p != NULL){
        
            position[0] = p->x;
            position[1] = p->y;
            position[2] = p->z;
            position[2] = -position[2];// clockwise -> positive
	    p1.id = {1, 2, 3};
            p1.position = {position[0], position[1], position[2]};
            joints.publish(p1);
	    loop_rate.sleep();
         }
    }
    
    break;
  }
  
  bag.close();
  return 0;
}
