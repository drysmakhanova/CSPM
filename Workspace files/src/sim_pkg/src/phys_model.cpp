#include "ros/ros.h"
#include "sim_pkg/Position.h"
#include "sim_pkg/ResetMotors.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

ros::ServiceClient client;


void set_position_callback(const sim_pkg::Position& msg) {
	for (size_t i = 0; i < msg.id.size(); ++i) {
		dynamixel_workbench_msgs::DynamixelCommand srv;
		srv.request.id = msg.id[i];
		srv.request.addr_name = "Goal_Position";
		srv.request.value = double(msg.position[i]) / 360 * 4096;
		client.call(srv);
	}
}

bool reset(sim_pkg::ResetMotors::Request  &req,
		 sim_pkg::ResetMotors::Response &res) {
	for (size_t id = 1; id <= 3; ++id) {
		dynamixel_workbench_msgs::DynamixelCommand srv;
		srv.request.id = id;
		srv.request.addr_name = "Goal_Position";
		srv.request.value = 0;
		client.call(srv);
	}
	res.result = true;
	ROS_INFO("Reseting all motors position to zero");
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "phys_model");

	ros::NodeHandle n;
	client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command"); //create the client
	ros::Subscriber sub = n.subscribe("/joints_phys", 1, set_position_callback);
	ros::ServiceServer service = n.advertiseService("platform_reset_motors", reset);
	ros::spin();

	return 0;
}
