#include <ros/ros.h>
#include <shared-control-balancing/GetTorques.h>

bool gettorques(shared-control-balancing::GetTorques::Request &req,
				shared-control-balancing::GetTorques::Response &res)
{
	res.b = 60.75;
	ROS_INFO("sent back the torque");
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_torques_server");
	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("get_torques", gettorques);
	ROS_INFO("GetTorques service called");
	ros::spin();

	return 0;
}