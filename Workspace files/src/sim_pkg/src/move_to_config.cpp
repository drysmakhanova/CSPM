#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  geometry_msgs::Point p3;
  std_msgs::String str;
  std_msgs::Bool t;
  bool checkCol; 


void subColCallback(const std_msgs::Bool::ConstPtr& msg)
{
  // Collision check
  checkCol = msg->data;
}

void subJointsCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  // Feedback from CoppeliaSim
  p3.x = msg->x;
  p3.y = msg->y;
  p3.z = msg->z;
}

void checkJointsCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  // Input from topic or GUI
  p1.x = msg->x;
  p1.y = msg->y;
  p1.z = msg->z;
}


int main(int argc, char **argv)
{

  float step = 5.0;
  int count = 1;
  t.data = true;
  str.data = "Collision detected";	
  
  ros::init(argc, argv, "mtc_v1");
  ros::NodeHandle n;
  
  // Topics
    
  ros::Publisher start = n.advertise<std_msgs::Bool>("/startSimulation", 10);
  ros::Publisher stop = n.advertise<std_msgs::Bool>("/stopSimulation", 10);
  ros::Publisher sync = n.advertise<std_msgs::Bool>("/enableSyncMode", 10);
  ros::Publisher next = n.advertise<std_msgs::Bool>("/triggerNextStep", 10);  
  ros::Publisher joints = n.advertise<geometry_msgs::Point>("/joints_set", 100); 
  ros::Publisher jointsCheck = n.advertise<geometry_msgs::Point>("/joints_check", 100); 
  ros::Publisher collision = n.advertise<std_msgs::String>("/collision", 10);
  
  ros::Subscriber subJoints = n.subscribe("/joints_get", 100, subJointsCallback);
  ros::Subscriber checkJoints = n.subscribe("/joints_mtc", 100, checkJointsCallback);
  ros::Subscriber subCol = n.subscribe("/col", 100, subColCallback);

  ros::Rate loop_rate(1);
  ros::Rate loop_rate2(1);
  double t1 =ros::Time::now().toSec();
  

  while (ros::ok())
  {	
  
  	double t2 =ros::Time::now().toSec();
	if (t2 - t1 < 2){
	  	start.publish(t);
	  	sync.publish(t);
	  	p2.x = 0;
	  	p2.y = 0;
	  	p2.z = 0;

	}
	
	// Simulation is stopped when 60 seconds pass or collision is detected ot congig achieved
	
	if (t2 - t1 > 60){
	  	stop.publish(t);
	  	break;
	}
	if (checkCol == true){
		ROS_INFO("Collision detected");
		collision.publish(str);
		stop.publish(t);
		break;
	} 
	if (count != 1) {
		ROS_INFO("Joint feedback: %f, %g, %f", p3.x, p3.y, -p3.z);
		ROS_INFO("No collisions");
		jointsCheck.publish(p3);
		stop.publish(t);
		break;
	}
	
	// Implementation
	
	int xx = abs(p1.x/step);
	int yy = abs(p1.y/step);
	int zz = abs(p1.z/step);
	
	int steps = xx;
	if (steps < yy) {
		steps = yy;
	}
	if (steps < zz) {
		steps = yy;
	}
	count = 0;
	for (int i = 1; i<steps+2; i++) {
	
		if (i <= xx) {
			p2.x = i*(p1.x/xx);
		} else if (i+1==xx){
			p2.x = (i-1)*(p1.x/xx)+(int(p1.x)%xx);
			}
		if (i <= yy) {
			p2.y = i*(p1.y/yy);
		} else if (i+1==yy){
			p2.y = (i-1)*(p1.y/yy)+(int(p1.y)%yy);
			}
		if (i <= zz) {
			p2.z = -i*(p1.z/zz);
		} else if (i+1==zz){
			p2.z = -(i-1)*(p1.z/xx)-(int(p1.z)%zz);
			}
		 
		joints.publish(p2);
		next.publish(t);
		loop_rate.sleep();
		count++;
  	}
  	ros::spinOnce();
  }

  return 0;
}

