// Pepper camera remote class
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Ensure header is only included once
#ifndef PEPPER_CAMERA_REMOTE_H
#define PEPPER_CAMERA_REMOTE_H

// Includes
#include <sys/socket.h>
#include <string>

// Pepper camera namespace
namespace pepper_camera
{
	// Pepper camera remote class
	class PepperCameraRemote
	{
	public:
		// Remote commands
		static bool start_camera(const std::string& remote_host, int remote_port, int udp_port, int device, int width, int height, int quality);
		static bool stop_camera(const std::string& remote_host, int remote_port, int udp_port, int device);

	private:
		// Constructor
		PepperCameraRemote() = delete;

		// Connection utilities
		static int connect(const std::string& remote_host, int remote_port);
		static uint32_t get_send_addr(int connection);
		template<class S> static bool send_data(int connection, const S& data) { return send(connection, &data, sizeof(S), 0) == sizeof(S); }
		static void disconnect(int connection);
	};
}

#endif
// EOF
