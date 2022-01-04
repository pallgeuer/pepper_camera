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

	// Start camera packet struct
	struct __attribute((__packed__)) StartCameraStruct
	{
		char cmd;  // Should be 'B'
		uint8_t device;
		uint16_t port;
		uint32_t ip;
		uint16_t width;
		uint16_t height;
		uint8_t quality;
	};

	// Stop camera packet struct
	struct __attribute((__packed__)) StopCameraStruct
	{
		char cmd;  // Should be 'E'
		uint8_t device;
		uint16_t port;
		uint32_t ip;
	};
}

#endif
// EOF
