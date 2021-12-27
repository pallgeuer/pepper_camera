// Pepper camera node
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Includes
#include <pepper_camera/pepper_camera.h>

// Main function
int main(int argc, char** argv)
{
	// Initialise the ROS node
	ros::init(argc, argv, "pepper_camera");
	ros::NodeHandle nh, nh_private("~");

	// Start an asynchronous ROS spinner
	ros::AsyncSpinner ros_spinner(2);
	ROS_INFO("Starting ROS main loop...");
	ros_spinner.start();

	// Run the Pepper camera streamer
	pepper_camera::PepperCamera pcamera(nh, nh_private);
	pcamera.run();

	// Stop the asynchronous ROS spinner
	ROS_INFO("Stopping ROS main loop...");
	ros_spinner.stop();

	// Shut down ROS (would happen anyway when last node handle is destructed)
	ROS_INFO("Shutting down ROS...");
	ros::shutdown();

	// Return success
	return 0;
}
// EOF
