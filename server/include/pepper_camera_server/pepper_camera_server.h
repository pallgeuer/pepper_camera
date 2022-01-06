// Pepper camera server class
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Ensure header is only included once
#ifndef PEPPER_CAMERA_SERVER_H
#define PEPPER_CAMERA_SERVER_H

// Includes
#include <pepper_camera_server/pepper_camera_packets.h>
#include <cstdint>
#include <string>

// Pepper camera namespace
namespace pepper_camera
{
	// Pepper camera server class
	class PepperCameraServer
	{
	public:
		// Constants
		static const int DEFAULT_CMD_PORT = 3006;
		static const int RECV_BUFFER_SIZE = 1024;

		// Constructor
		explicit PepperCameraServer(int cmd_port = DEFAULT_CMD_PORT);
		virtual ~PepperCameraServer();

		// Run
		bool run();

	private:
		// Socket methods
		bool create_socket();
		bool initialise_socket();
		bool bind_socket();
		bool start_listen();
		bool receive_command(int fd);
		void disconnect_socket();

		// Stream methods
		void start_stream(const StartCameraStruct* S, const std::string& target_ip);
		void stop_stream(uint8_t device);
		void stop_all_streams();
		void kill_stream(uint8_t device);

		// Internal variables
		const int m_cmd_port;
		int m_socket;
		uint8_t m_recv_buffer[RECV_BUFFER_SIZE];
		int m_stream_pid[MAX_VIDEO_DEVICE + 1];
	};
}

#endif
// EOF
