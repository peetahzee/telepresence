#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

geometry_msgs::Twist userCommand;
geometry_msgs::Twist controlCommand;

void userCommandCallback(const geometry_msgs::Twist::ConstPtr& cmd) {
	// ROS_INFO("Received user command: [%f] / [%f] / [%f]", cmd->linear.x, cmd->linear.y, cmd->angular.z);
	userCommand = *cmd;
}

void controlCommandCallback(const geometry_msgs::Twist::ConstPtr& cmd) {
	// ROS_INFO("Received control command: [%f] / [%f] / [%f]", cmd->linear.x, cmd->linear.y, cmd->angular.z);
	controlCommand = *cmd;
}

bool isCommandEmpty(const geometry_msgs::Twist& cmd) {
	return cmd.linear.x == 0.0 && cmd.linear .y == 0.0 && cmd.angular.z == 0.0;
}

int main (int argc, char **argv) {

	ros::init(argc, argv, "usc_mrp");	
	ros::NodeHandle n;

	ros::Publisher cmdVel_pub = n.advertise<geometry_msgs::Twist>("/base_controller/command", 1000);
	ros::Subscriber cmdVel_userSub = n.subscribe("/usc_mrp/user_command", 1000, userCommandCallback);
	ros::Subscriber cmdVel_controlSub = n.subscribe("/usc_mrp/control_command", 1000, controlCommandCallback);

	ros::Rate loop_rate(10);


	ROS_INFO("Starting Server...");

	while(ros::ok()) {
		geometry_msgs::Twist msg;
		if(isCommandEmpty(controlCommand)) {
			if(!isCommandEmpty(userCommand)) {
				msg = userCommand;
				ROS_INFO("Publishing user command");
			}
		} else {
			msg = controlCommand;
			ROS_INFO("Publishing control command");
		}
		ros::spinOnce();
		loop_rate.sleep();
		// cmdVel_pub.publish(msg);
	}
		// ros::spin();

}

