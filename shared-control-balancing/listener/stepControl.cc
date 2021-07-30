#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
// #include <shared_control_msgs/GetTorques.h>
// #include <shared-control-balancing/GetTorques.h>

static bool stop_simulation;

void rosShutdownHandler(int sig)
{
stop_simulation = true;
}

class SubAndPub
{
public:
	SubAndPub()
	{
		pub_ = nh.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_1_effort_controller/command", 1);
		//sub_ = nh.subscribe("/torque_joint1", 1, &SubAndPub::torqueCallback, this);
	}
	/*
	void torqueCallback(const std_msgs::Float64::ConstPtr& val)
	{
		ROS_INFO("Got : [%f]", val->data);
		std_msgs::Float64 output;
		output.data = val->data; 
		pub_.publish(output);
	}
	*/
private:
	ros::NodeHandle nh;
	ros::Publisher pub_;
	ros::Subscriber sub_;
};

int main(int argc, char **argv) {
	gazebo::client::setup(argc,argv);
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	ros::init(argc, argv, "gazebo_iterator"); // register the node on ROS
	ros::NodeHandle n;
	//Using serice start
	// ros::ServiceClient client = n.serviceClient<shared-control-balancing::GetTorques>("get_torques");
	// shared-control-balancing::GetTorques srv;
	// srv.request.a = true;
	// if(client.call(stv))
	// {
	// 	ROS_INFO("Client got: %f", srv.response.b);
	// }
	// else
	// {
	// 	ROS_ERROR("Failed to call service get_torques");
	// 	return 1;
	// }
	//Using serice end
	//Create publisher for topic ~/world_control
	gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
	//Create message
	gazebo::msgs::WorldControl msg_step;

	stop_simulation = false;

	//Publish to topic ~/world_control
	pub->WaitForConnection();
	msg_step.set_pause(0);
	ros::Rate r(0.5);
	while( ros::ok() ) {
	// while( 1 ) {
		//Set step to true
		msg_step.set_step(1);
		pub->Publish(msg_step, true);
		// msg_step.set_step(0);
		msg_step.set_pause(0);
		std::cout << "step:\n";
		r.sleep();
	}
	//Publish 1st step

	// SubAndPub spobj;

	gazebo::shutdown();
	ros::spinOnce();
	return 0;
} 