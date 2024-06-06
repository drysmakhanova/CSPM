#include "ros/ros.h"
#include "math.h"
#include "rosbag/bag.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Joy.h"
#include "Eigen/Dense"

#define PI 3.14159265359
#define deg2rad 0.01745329251
#define rad2deg 57.2957795131

  geometry_msgs::Point joints_cb;
  std_msgs::String str;
  geometry_msgs::Point joy;
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  bool checkCol; 
  int rotate;
  int dont;
  int stop_sim;
  std_msgs::Bool t;
  	  Eigen::Matrix <float, 1, 3> point = {0.71,0,0.71};
	  Eigen::Matrix <float, 1, 3> refline_fixed = {-1,0,0};
	  Eigen::Matrix <float, 1, 3> v1;
	  Eigen::Matrix <float, 1, 3> v2;
	  Eigen::Matrix <float, 1, 3> v3;
	  Eigen::Matrix <float, 1, 3> scale;
  
	
void subColCallback(const std_msgs::Bool::ConstPtr& msg)
{
  // Collision check
  checkCol = msg->data;
}
	
void subJointsCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  // Feedback from CoppeliaSim
  joints_cb.x = msg->x;
  joints_cb.y = msg->y;
  joints_cb.z = msg->z;
}

void subJoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Input from joystick
  joy.x = msg->axes[0];
  joy.y = msg->axes[1];
  joy.z = 0;
  rotate = msg->buttons[0];
  dont = msg->buttons[1];
  stop_sim = msg->buttons[2];
}

int main(int argc, char **argv)
{
  t.data = true;
  str.data = "Collision detected";
  ros::init(argc, argv, "joy_v1");
  ros::NodeHandle n;
  rosbag::Bag bag("test.bag", rosbag::bagmode::Write);
  
  // Topics
  
  ros::Publisher joints = n.advertise<geometry_msgs::Point>("/joints_set", 100);  
  ros::Publisher start = n.advertise<std_msgs::Bool>("/startSimulation", 10);
  ros::Publisher stop = n.advertise<std_msgs::Bool>("/stopSimulation", 10);
  ros::Publisher sync = n.advertise<std_msgs::Bool>("/enableSyncMode", 10);
  ros::Publisher next = n.advertise<std_msgs::Bool>("/triggerNextStep", 10);
  ros::Publisher collision = n.advertise<std_msgs::String>("/collision", 10);
    
  ros::Subscriber subJoints = n.subscribe("/joints_get", 100, subJointsCallback);
  ros::Subscriber jj = n.subscribe("/joy", 100, subJoyCallback);
  ros::Subscriber subCol = n.subscribe("/col", 100, subColCallback);
	
  // Design parameters
  
  float increment_deg = 0;
  int rot = 0;
  float beta = 90;
  float gamma = 0;
  float alpha1 = 45;
  float alpha2 = 90;
  float alpha3 = 2*asin(sin(beta*deg2rad)*cos(PI/6));
  float eta [3];
  for (int i=0;i<3;i++) {
  	eta[i] = 2*i*180/3;
  }

  ros::Rate loop_rate(20); //50ms 
  ros::Rate loop_rate2(1);
  double t1 =ros::Time::now().toSec();
  
while (ros::ok())
  {
  	  // Starting the simulation and syncing times at the beginning of the code
  	  
	  double t2 =ros::Time::now().toSec();
	  if (t2 - t1 < 1){
	  	start.publish(t);
	  	sync.publish(t);
	  }
	  
	  // Simulation is stopped when button is pressed or 60 seconds pass
	  
	  if (t2 - t1 > 60 || stop_sim == 1){
	  	p1.x = 5;//green
		p1.y = 5;//yellow
		p1.z = -5;//red
		bag.write("Points", ros::Time::now(), p1);
		joints.publish(p1);
	  	bag.close();
	  	stop.publish(t);
	  	break;
	  }
	  

	  // Input values
	  
	  scale[0] = -joy.x*(sqrt(2)/2);  //x -> leftt/right
	  scale[1] = joy.y*(sqrt(2)/2);  //y -> up/down
	  scale[2] = sqrt(1 - scale[0]*scale[0] - scale[1]*scale[1]);
	  
	  if (rotate == 1) {
	  	rot = 1;
	  	p2.x = scale[0];
	  	p2.y = scale[1];
	  	p2.z = scale[2];
	  }
	  
	  if (dont == 1 && rot ==1) {
	  	rot = 0;
	  }
	  
	  if (rot == 0) {
	    for(int i = 0; i<3; i++){
	  	point[i] = scale[i];
	    }
	    } else {
	    	point[0] = p2.x;
	    	point[1] = p2.y;
	    	point[2] = p2.z;
	  }
	  
	  v1 = point.cross(refline_fixed.transpose());
	  v1 = v1/sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]);
	  
	  if (rot == 1){
	  	increment_deg = increment_deg - 1;
	  	v1 = cos(increment_deg*deg2rad)*v1+sin(increment_deg*deg2rad)*point.cross(v1);
	  	v1 = v1/sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]);
	  }
	  
	  v2 = cos(120*deg2rad)*v1+sin(120*deg2rad)*point.cross(v1);
	  v2 = v2/sqrt(v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2]);
	  
	  v3 = cos(240*deg2rad)*v1+sin(240*deg2rad)*point.cross(v1);
	  v3 = v3/sqrt(v3[0]*v3[0]+v3[1]*v3[1]+v3[2]*v3[2]);
	  
	  Eigen::Matrix3f  vs;
	  vs.col(0) = v1.transpose();
	  vs.col(1) = v2.transpose();
	  vs.col(2) = v3.transpose();
	  
	  //IK 
	  
	  	  Eigen::Matrix< float, 1, 3>  A = Eigen::Matrix< float, 1, 3>::Zero();
	  Eigen::Matrix< float, 1, 3>  B = Eigen::Matrix< float, 1, 3>::Zero();
	  Eigen::Matrix< float, 1, 3>  C = Eigen::Matrix< float, 1, 3>::Zero();
	  Eigen::Matrix< float, 2, 3>  T = Eigen::Matrix< float, 2, 3>::Zero();
	  Eigen::Matrix< float, 2, 3>  theta = Eigen::Matrix< float, 2, 3>::Zero();
	  
	  for(int i = 0; i<3; i++) {
	  	
	  	A[i] = cos(eta[i]*deg2rad)*sin(gamma*deg2rad)*cos(alpha1*deg2rad)*vs(0,i)
	  		-cos(eta[i]*deg2rad)*cos(gamma*deg2rad)*sin(alpha1*deg2rad)*vs(0,i)
	  		+sin(eta[i]*deg2rad)*sin(gamma*deg2rad)*cos(alpha1*deg2rad)*vs(1,i)
	  		-sin(eta[i]*deg2rad)*cos(gamma*deg2rad)*sin(alpha1*deg2rad)*vs(1,i)
	  					-cos(gamma*deg2rad)*cos(alpha1*deg2rad)*vs(2,i)
	  					-sin(gamma*deg2rad)*sin(alpha1*deg2rad)*vs(2,i)
	  							-cos(alpha2*deg2rad);
	  							
	  	B[i] = sin(eta[i]*deg2rad)*sin(alpha1*deg2rad)*vs(0,i)
	  		-cos(eta[i]*deg2rad)*sin(alpha1*deg2rad)*vs(1,i);
	  		
	  	C[i] = cos(eta[i]*deg2rad)*sin(gamma*deg2rad)*cos(alpha1*deg2rad)*vs(0,i)
	  		+cos(eta[i]*deg2rad)*cos(gamma*deg2rad)*sin(alpha1*deg2rad)*vs(0,i)
	  		+sin(eta[i]*deg2rad)*sin(gamma*deg2rad)*cos(alpha1*deg2rad)*vs(1,i)
	  		+sin(eta[i]*deg2rad)*cos(gamma*deg2rad)*sin(alpha1*deg2rad)*vs(1,i)
	  					-cos(gamma*deg2rad)*cos(alpha1*deg2rad)*vs(2,i)
	  					+sin(gamma*deg2rad)*sin(alpha1*deg2rad)*vs(2,i)
	  							-cos(alpha2*deg2rad);
	  							
	  	T(1,i) = (-2*B(i)+sqrt(((2*B(i))*(2*B(i)))-4*A(i)*C(i)))/(2*A(i));
		T(2,i) = (-2*B(i)-sqrt(((2*B(i))*(2*B(i)))-4*A(i)*C(i)))/(2*A(i));
		
		theta(1,i) = 2*atan(T(1,i))*rad2deg;
		theta(2,i) = 2*atan(T(2,i))*rad2deg;
	  }
	  
	  // Implementation
	  
	  float angles [3];
	  for (int i=0;i<3;i++) {
	  	if (theta(1,i) != theta(1,i)) {
	  		angles[i] = 0;
	  	} else {
	  		angles[i] = theta(1,i);
	  	}
	}

	float offset[3];
	offset[0] = angles[1]-angles[1];
	offset[1] = angles[0]-angles[2];
	offset[2] = angles[2]-angles[1];
	if (offset[0]>120){
		angles[0] = angles[0]+360;
	}
	if (offset[1]>120){
		angles[2] = angles[2]+360;
	}
	if (offset[2]>120){
		angles[1] = angles[1]+360;
	}
	 
	 
	if (checkCol == true){
		ROS_INFO("Collision detected");
		collision.publish(str);
		bag.write("Collision", ros::Time::now(), t);
		stop.publish(t);
		bag.close();
		break;
	} 
	
	p1.x = angles[0];//green
	p1.y = angles[1];//yellow
	p1.z = -angles[2];//red
	next.publish(t);
	
	bag.write("Points", ros::Time::now(), p1);
	joints.publish(p1);
	loop_rate.sleep();

    	ros::spinOnce();
	}

  return 0;
}

