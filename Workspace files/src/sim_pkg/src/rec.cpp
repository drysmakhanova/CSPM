    #include "ros/ros.h"
    #include "rosbag/bag.h"
    #include "rosbag/view.h"
    #include "std_msgs/Bool.h"
    #include "geometry_msgs/Point.h"
    #include "std_msgs/String.h"
    #include <boost/foreach.hpp>
    #define foreach BOOST_FOREACH

  geometry_msgs::Point p1;
  std_msgs::Bool t;
  bool checkCol; 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rec");
  ros::NodeHandle n;
  
  std_msgs::String str;
  str.data = "Collision detected";
  
  ros::Publisher start = n.advertise<std_msgs::Bool>("/startSimulation", 10);
  ros::Publisher stop = n.advertise<std_msgs::Bool>("/stopSimulation", 10);
  ros::Publisher sync = n.advertise<std_msgs::Bool>("/enableSyncMode", 10);
  ros::Publisher next = n.advertise<std_msgs::Bool>("/triggerNextStep", 10); 
  ros::Publisher collision = n.advertise<std_msgs::String>("/collision", 10);
  ros::Publisher joints = n.advertise<geometry_msgs::Point>("/joints_set", 100); 
  
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
    
  ros::Rate loop_rate(20);
  ros::Rate loop_rate2(1);
  double t1 =ros::Time::now().toSec();
  
  while (ros::ok())
  {	
    
  foreach(rosbag::MessageInstance const m, view) {
        geometry_msgs::Point::ConstPtr p = m.instantiate<geometry_msgs::Point>();
        
        if (p != NULL){
        
        double t2 =ros::Time::now().toSec();
	if (t2 - t1 < 1){
	  	start.publish(t);
	  	sync.publish(t);
	  	
	  	p1.x = 0;
	  	p1.y = 0;
	  	p1.z = 0;
	  	joints.publish(p1);
	}
	  
            p1.x = p->x;
            p1.y = p->y;
            p1.z = p->z;
            joints.publish(p1);
            next.publish(t);
	    loop_rate.sleep();
         }
    }
    
    stop.publish(t);
    break;
  }
  
  bag.close();
  return 0;
}
