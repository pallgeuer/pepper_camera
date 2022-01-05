// Pepper camera server
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Includes
#include <pepper_camera_server/pepper_camera_server.h>
#include <iostream>
#include <sstream>

// Namespaces
using namespace pepper_camera;

//
// Main
//

// Main function
int main(int argc, char** argv)
{
	// Decide on the command port number
	int cmd_port = PepperCameraServer::DEFAULT_CMD_PORT;
	if(argc >= 2)
	{
		int parsed_cmd_port;
		std::istringstream ss(argv[1]);
		if(ss >> parsed_cmd_port)
			cmd_port = parsed_cmd_port;
		else
			std::cout << "Failed to parse port number from '" << argv[1] << "' => Using " << cmd_port << " instead" << std::endl;
	}

	// Create a camera server
	PepperCameraServer server(cmd_port);

	// Continually process network communications
	server.run();

	// Return success
	return 0;
}

//
// Pepper camera server class
//

// Constructor
PepperCameraServer::PepperCameraServer(int cmd_port)
{
	// TODO
	std::cout << "Using command port " << cmd_port << std::endl;
}

// Run
void PepperCameraServer::run()
{
	// TODO
	std::cout << "RUNNING" << std::endl;
}
// EOF
