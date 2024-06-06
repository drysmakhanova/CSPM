/* Produced by CVXGEN, 2022-04-22 18:20:13 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
#include <cstdio>
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
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0

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
  checkCol = msg->data;
}
	
void subJointsCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  joints_cb.x = msg->x;
  joints_cb.y = msg->y;
  joints_cb.z = msg->z;
  //ROS_INFO("Received: %f, %f, %f", joints_cb.x, joints_cb.y, joints_cb.z);
}

void subJoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  joy.x = msg->axes[0];
  joy.y = msg->axes[1];
  joy.z = 0;
  rotate = msg->buttons[0];
  dont = msg->buttons[1];
  stop_sim = msg->buttons[2];
}


int main(int argc, char **argv) {

  t.data = true;
  str.data = "Collision detected";
  ros::init(argc, argv, "testsolver");
  ros::NodeHandle n; 
  rosbag::Bag bag("test.bag", rosbag::bagmode::Write);
  
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
    // topics
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
  start.publish(t);

  
while (ros::ok())
  {
	  double t2 =ros::Time::now().toSec();
	  if (t2 - t1 < 1){
	  	start.publish(t);
	  	sync.publish(t);
	  }
	  
	  if (t2 - t1 > 60 || stop_sim == 1){
	  	bag.close();
	  	stop.publish(t);
	  	break;
	  }
	  

	  // Input values

	  scale[0] = -joy.x;  //x -> leftt/right
	  scale[1] = joy.y;  //y -> up/down
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
	
	//for (int i = 0; i<3; i++){
	//	if (angles[i]<0){
	//		angles[i] = angles[i] + 360;
	//	}
	//}
	
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
	params.ref[0] = angles[0]*deg2rad;
  	params.ref[1] = angles[1]*deg2rad;
  	params.ref[2] = angles[2]*deg2rad;
	num_iters = solve();
	
	p1.x = vars.theta[0]*rad2deg;//green
	p1.y = vars.theta[1]*rad2deg;//yellow
	p1.z = -vars.theta[2]*rad2deg;//red
	
	next.publish(t);
	
	bag.write("Points", ros::Time::now(), p1);
	joints.publish(p1);
	loop_rate.sleep();

    	ros::spinOnce();
	}
  
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  ROS_INFO("  here", vars.theta[0]);
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  float big_M[18][3] = {
	{-0.0023809321350169,0.002380932135017,0.002380932135017},
	{0.002380932135017,0.002380932135017,-0.00238093213501689},
	{0.0023809321350169,-0.00238093213501689,0.0023809321350169},
	{-2.76057425895318E-018,0.0111097396229727,-0.0111097396229727},
	{-0.0111097396229727,6.15458428984808E-018,0.0111097396229727},
	{1.73472347597681E-018,-0.0111097396229727,0.0111097396229727},
	{-1,0,0},
	{0,0,-1},
	{0,-1,-1.11022302462516E-016},
	{-0.0111097396229727,0.0111097396229727,5.55156015548123E-018},
	{-0.0133315559110321,0.00666577795551606,0.00666577795551606},
	{0.0111097396229727,-1.02329049441555E-017,-0.0111097396229727},
	{0.00666577795551606,0.00666577795551606,-0.0133315559110321},
	{0.0111097396229727,-0.0111097396229727,-3.46805474677816E-018},
	{0.00666577795551606,-0.0133315559110321,0.00666577795551606},
	{6.97882215034351E-019,0.00277776706110454,1.21636781046588E-018},
	{1.77893310652328E-018,1.71686466178787E-018,0.00277776706110454},
	{0.00277776706110454,2.17572883806635E-018,-8.04236792298464E-019}
  };
  int k=0;
  for (int i=0; i<3; i++){
	for (int j=0; j<18; j++){
		params.M[k] = big_M[j][i];
		k++;
	};
  };

  float small_M[18] = {0.9999914967071,0.9999914967071,0.9999914967071,
	0.999876566067542,0.999876566067542,0.999876566067542,0,0,0,0.999876566067542,
	0.999866693327409,0.999876566067542,0.999866693327409,0.999876566067542,
	0.999866693327409,0.999996141997635,0.999996141997635,0.999996141997635};

  for (int i=0; i<18; i++){
	params.m[i] = small_M[i];

  };
  

}
/*
  params.ref[0] = 0.20319161029830202;
  params.ref[1] = 0.8325912904724193;
  params.ref[2] = -0.8363810443482227;
  params.M[0] = 0.04331042079065206;
  params.M[1] = 1.5717878173906188;
  params.M[2] = 1.5851723557337523;
  params.M[3] = -1.497658758144655;
  params.M[4] = -1.171028487447253;
  params.M[5] = -1.7941311867966805;
  params.M[6] = -0.23676062539745413;
  params.M[7] = -1.8804951564857322;
  params.M[8] = -0.17266710242115568;
  params.M[9] = 0.596576190459043;
  params.M[10] = -0.8860508694080989;
  params.M[11] = 0.7050196079205251;
  params.M[12] = 0.3634512696654033;
  params.M[13] = -1.9040724704913385;
  params.M[14] = 0.23541635196352795;
  params.M[15] = -0.9629902123701384;
  params.M[16] = -0.3395952119597214;
  params.M[17] = -0.865899672914725;
  params.M[18] = 0.7725516732519853;
  params.M[19] = -0.23818512931704205;
  params.M[20] = -1.372529046100147;
  params.M[21] = 0.17859607212737894;
  params.M[22] = 1.1212590580454682;
  params.M[23] = -0.774545870495281;
  params.M[24] = -1.1121684642712744;
  params.M[25] = -0.44811496977740495;
  params.M[26] = 1.7455345994417217;
  params.M[27] = 1.9039816898917352;
  params.M[28] = 0.6895347036512547;
  params.M[29] = 1.6113364341535923;
  params.M[30] = 1.383003485172717;
  params.M[31] = -0.48802383468444344;
  params.M[32] = -1.631131964513103;
  params.M[33] = 0.6136436100941447;
  params.M[34] = 0.2313630495538037;
  params.M[35] = -0.5537409477496875;
  params.M[36] = -1.0997819806406723;
  params.M[37] = -0.3739203344950055;
  params.M[38] = -0.12423900520332376;
  params.M[39] = -0.923057686995755;
  params.M[40] = -0.8328289030982696;
  params.M[41] = -0.16925440270808823;
  params.M[42] = 1.442135651787706;
  params.M[43] = 0.34501161787128565;
  params.M[44] = -0.8660485502711608;
  params.M[45] = -0.8880899735055947;
  params.M[46] = -0.1815116979122129;
  params.M[47] = -1.17835862158005;
  params.M[48] = -1.1944851558277074;
  params.M[49] = 0.05614023926976763;
  params.M[50] = -1.6510825248767813;
  params.M[51] = -0.06565787059365391;
  params.M[52] = -0.5512951504486665;
  params.M[53] = 0.8307464872626844;
  params.m[0] = 0.9869848924080182;
  params.m[1] = 0.7643716874230573;
  params.m[2] = 0.7567216550196565;
  params.m[3] = -0.5055995034042868;
  params.m[4] = 0.6725392189410702;
  params.m[5] = -0.6406053441727284;
  params.m[6] = 0.29117547947550015;
  params.m[7] = -0.6967713677405021;
  params.m[8] = -0.21941980294587182;
  params.m[9] = -1.753884276680243;
  params.m[10] = -1.0292983112626475;
  params.m[11] = 1.8864104246942706;
  params.m[12] = -1.077663182579704;
  params.m[13] = 0.7659100437893209;
  params.m[14] = 0.6019074328549583;
  params.m[15] = 0.8957565577499285;
  params.m[16] = -0.09964555746227477;
  params.m[17] = 0.38665509840745127;
}*/
