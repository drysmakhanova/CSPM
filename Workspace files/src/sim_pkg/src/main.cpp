#include "ros/ros.h"
#include "math.h"
#include "rosbag/bag.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "sim_pkg/Position.h"
#include "sim_pkg/ResetMotors.h"
#include "geometry_msgs/Point.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Joy.h"
#include "Eigen/Dense"
#include <vector>

#define PI 3.14159265359
#define deg2rad 0.01745329251
#define rad2deg 57.2957795131

  sim_pkg::Position p;
  geometry_msgs::Point joints_cb;
  std_msgs::String str;
  std_msgs::String str2;
  std_msgs::String gui_msg2;
  std_msgs::String mtc;
  std_msgs::String joystick;
  std_msgs::String recorded;
  std_msgs::String reclive;
  geometry_msgs::Point joy;
  geometry_msgs::Point p1;
  geometry_msgs::Point p2; 
  std_msgs::Bool t;
  
  std::vector<geometry_msgs::Point> rec;
  bool checkCol; 
  int rotate;
  int dont;
  int stop_sim;
  int mode = 0;
  Eigen::Matrix <float, 1, 3> point = {0.71,0,0.71};
  Eigen::Matrix <float, 1, 3> refline_fixed = {-1,0,0};
  Eigen::Matrix <float, 1, 3> v1;
  Eigen::Matrix <float, 1, 3> v2;
  Eigen::Matrix <float, 1, 3> v3;
  Eigen::Matrix <float, 1, 3> scale;
  double t1;
  
void subGuiCallback(const std_msgs::String::ConstPtr& msg)
{
  gui_msg2.data = msg->data;
  if (gui_msg2.data == mtc.data) {
  	mode = 1;
  	t1 =ros::Time::now().toSec();
  } else if (gui_msg2.data == joystick.data) {
  	mode = 2;
  	t1 =ros::Time::now().toSec();
  } else if (gui_msg2.data == recorded.data) {
  	mode = 3;
  	 t1 =ros::Time::now().toSec();
  } else if (gui_msg2.data == reclive.data) {
  	mode = 4;
  	t1 =ros::Time::now().toSec();
  } 
}
	
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

void checkJointsCallback(const std_msgs::String::ConstPtr& msg)
{
  // Mode selection from GUI topic
  str2.data = msg->data;
  std::string str3 = str2.data;
  std::vector<int> vect;
  std::stringstream ss(str3);

    for (int i; ss >> i;) {
        vect.push_back(i);    
        if (ss.peek() == ',')
            ss.ignore();
    }

  p1.x = vect[0];
  p1.y = vect[1];
  p1.z = vect[2];
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
  mtc.data = "mtc";
  joystick.data = "joystick";
  recorded.data = "recorded_sim";
  reclive.data = "recorded";
  float step = 5.0;
  int count = 1;
  t.data = true;
  str.data = "Collision detected";
  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  rosbag::Bag bag("main.bag", rosbag::bagmode::Write);
  
  // Topics
  
  ros::Publisher start = n.advertise<std_msgs::Bool>("/startSimulation", 10);
  ros::Publisher stop = n.advertise<std_msgs::Bool>("/stopSimulation", 10);
  ros::Publisher sync = n.advertise<std_msgs::Bool>("/enableSyncMode", 10);
  ros::Publisher next = n.advertise<std_msgs::Bool>("/triggerNextStep", 10);
  ros::Publisher joints = n.advertise<geometry_msgs::Point>("/joints_set", 100); 
  ros::Publisher joints_phys = n.advertise<sim_pkg::Position>("/joints_phys", 1000); 
    
  ros::Subscriber subJoints = n.subscribe("/joints_feedback", 100, subJointsCallback);
  ros::Subscriber checkJoints = n.subscribe("/hello_gui_node/chatter", 100, checkJointsCallback);
  ros::Subscriber jj = n.subscribe("/joy", 100, subJoyCallback);
  ros::Subscriber subCol = n.subscribe("/collision", 100, subColCallback);
  ros::Subscriber subGui = n.subscribe("/hello_gui_node/gui_msgs", 100, subGuiCallback);
	
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
  ros::Rate loop_rate2(0.5);

  rec.resize(200);
  int rec_count = 0;
  int position[3] = {0,0,0};
  p.id = {1, 2, 3};
  p.position = {position[0], position[1], position[2]};
            
       
while (ros::ok())
  {
  	// Move to configuration
  	
  	if (mode == 1) {
	double t2 = ros::Time::now().toSec();
	if (t2 - t1 < 1){
	  	start.publish(t);
	  	sync.publish(t);

	}
	
	// Simulation is stopped when 60 seconds pass and return to main loop
	
	if (t2 - t1 > 60){
	  	stop.publish(t);
	  	mode = 0;
	}
	
	int xx = abs(p1.x/step);
	int yy = abs(p1.y/step);
	int zz = abs(p1.z/step);
	
	int steps = xx;
	if (steps < yy) {
		steps = yy;
	}
	if (steps < zz) {
		steps = zz;
	}
	count = 0;
	for (int i = 1; i<steps+1; i++) {
	
		if (i <= xx) {
			p2.x = i*(p1.x/xx);
		} 
		if (i <= yy) {
			p2.y = i*(p1.y/yy);
		} 
		if (i <= zz) {
			p2.z = -i*(p1.z/zz);
		} 
		joints.publish(p2);
		next.publish(t); 
		loop_rate.sleep();
		count++;
		
		if (checkCol == true){
		ROS_INFO("Collision detected");
		stop.publish(t);
		mode = 0;
		} 
  	}
  	if (count > 1){
  		ros::Duration(1).sleep();
		stop.publish(t);
		mode = 0;
  	}
	}
	
	
	
	// Joystick
	
	
  	if (mode == 2) {
		
	  double t2 =ros::Time::now().toSec();
	  if (t2 - t1 < 1){
	  	start.publish(t);
	  	sync.publish(t);
	  	
	  }
	  
	  if (t2 - t1 > 60 || stop_sim == 1){
	  	p1.x = 5;//green
		p1.y = 5;//yellow
		p1.z = -5;//red
		bag.write("Points", ros::Time::now(), p1);
		rec.push_back(p1);
		rec_count++;
		joints.publish(p1);
	  	stop.publish(t);
	  	mode = 0;
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
	  
	  std::vector<float> A(1,3);
	  std::vector<float> B(1,3);
	  std::vector<float> C(1,3);
	  std::vector<float> T(1,3);
	  std::vector<float> theta(1,3);
	  

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
	  							
	  	T[i] = (-2*B[i]+sqrt(((2*B[i])*(2*B[i]))-4*A[i]*C[i]))/(2*A[i]);
		T[2][i] = (-2*B(i)-sqrt(((2*B(i))*(2*B(i)))-4*A(i)*C(i)))/(2*A(i));
		
		theta[i] = 2*atan(T[i])*rad2deg;
		theta[2][i] = 2*atan(T(2,i))*rad2deg;
	  }
	
	  // Check input angles
	  
	  float angles [3];
	  for (int i=0;i<3;i++) {
	  	if (theta[i] != theta[i]) {
	  		angles[i] = 0;
	  	} else {
	  		angles[i] = theta[i];
	  	}
	}
	  
	  // Implementation

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
		bag.write("Collision", ros::Time::now(), t);
		stop.publish(t);
		mode = 0;
	} 
	
	p1.x = angles[0];//green
	p1.y = angles[1];//yellow
	p1.z = -angles[2];//red
	next.publish(t);
	
	bag.write("Points", ros::Time::now(), p1);
	rec.push_back(p1);
	rec_count++;
	joints.publish(p1);
	loop_rate.sleep();
	
	}
	
	// Recorded replay sim
	
	if (mode == 3) {
	 double t2 =ros::Time::now().toSec();
	  if (t2 - t1 < 1){
	  	start.publish(t);
	  	sync.publish(t);
	  	
	  }
		for (int j=100;j<rec.size();j++){
			p1 = rec.at(j);
			next.publish(t);
			joints.publish(p1);
			loop_rate.sleep();
		}
		mode = 0;
		stop.publish(t);
	}
	
	// Recorded replay live
	
		if (mode == 4) {

		for (int j=100;j<rec.size();j++){
			p1 = rec.at(j);
			position[0] = p1.x;
            		position[1] = p1.y;
            		position[2] = p1.z;
           		position[2] = -position[2];// clockwise -> positive
	  		p.id = {1, 2, 3};
            		p.position = {position[0], position[1], position[2]};
            		joints_phys.publish(p);
			loop_rate.sleep();
		}
		mode = 0;
	}
	
	ros::spinOnce();
	}
  bag.close();
  return 0;
}

