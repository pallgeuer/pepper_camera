// Pepper camera server
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Includes
#include <pepper_camera_server/pepper_camera_server.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <cstring>

// Namespaces
using namespace pepper_camera;

//
// Main
//

// Global namespace
namespace
{
	// Shutdown flag
	volatile sig_atomic_t perform_shutdown = 0;
}

// SIGINT/SIGTERM handler
void signal_handler(int /*signum*/)
{
	// Signal a shutdown to the remaining code
	perform_shutdown = 1;
}

// Main function
int main(int argc, char** argv)
{
	// Prevent any child processes from remaining zombies until they are waited upon when they are killed
	struct sigaction sigchld_action = {};
	sigchld_action.sa_handler = SIG_DFL;
	sigchld_action.sa_flags = SA_NOCLDWAIT;
	sigaction(SIGCHLD, &sigchld_action, NULL);

	// Install SIGINT and SIGTERM handlers that just make the main loop exit
	struct sigaction sig_action = {};
	sig_action.sa_handler = signal_handler;
	sigaction(SIGINT, &sig_action, NULL);
	sigaction(SIGTERM, &sig_action, NULL);

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
PepperCameraServer::PepperCameraServer(int cmd_port) :
	m_cmd_port(cmd_port),
	m_socket(-1),
	m_recv_buffer(),
	m_stream_pid()
{
	// Display the port that the server will listen to commands on
	std::cout << "Configured server command port as " << m_cmd_port << std::endl;
}

// Destructor
PepperCameraServer::~PepperCameraServer()
{
	// Disconnect the socket if applicable
	disconnect_socket();
	stop_all_streams();
}

// Run
bool PepperCameraServer::run()
{
	// Display that Pepper camera server is running
	std::cout << "Starting Pepper camera server..." << std::endl;

	// Initialise server
	bool success = false;
	if(!create_socket())
		std::cout << "Failed to create socket!" << std::endl;
	else if(!initialise_socket())
		std::cout << "Failed to initialise socket!" << std::endl;
	else if(!bind_socket())
		std::cout << "Failed to bind socket!" << std::endl;
	else if(!start_listen())
		std::cout << "Failed to start listening on socket!" << std::endl;
	else
		success = true;
	if(!success)
	{
		disconnect_socket();
		return false;
	}

	// Initialise a file descriptor set to contain only the main socket file descriptor
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(m_socket, &fds);
	int fd_max = m_socket;

	// Main loop
	while(!perform_shutdown)
	{
		fd_set fds_ready = fds;
		int ret = pselect(fd_max + 1, &fds_ready, NULL, NULL, NULL, NULL);
		if(ret < 0)
		{
			int select_err = errno;
			if(select_err == EINTR)
				continue;
			else
			{
				std::cout << "Select call failed with errno " << select_err << std::endl;
				break;
			}
		}
		else if(ret > 0)
		{
			for(int fd = 0; fd <= fd_max; fd++)
			{
				if(FD_ISSET(fd, &fds_ready))
				{
					if(fd == m_socket)
					{
						sockaddr_in client_addr;
						socklen_t client_addr_len = sizeof(client_addr);
						int new_socket = accept(m_socket, (sockaddr*) &client_addr, &client_addr_len);
						if(new_socket < 0)
							std::cout << "Failed to accept incoming connection" << std::endl;
						else
						{
							FD_SET(new_socket, &fds);
							if(new_socket > fd_max)
								fd_max = new_socket;
							char client_ip[INET_ADDRSTRLEN];
							const char* client_ip_success = inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
							if(client_ip_success)
								std::cout << "Client " << client_ip_success << " connected (FD " << new_socket << ")" << std::endl;
							else
								std::cout << "Unknown client connected (FD " << new_socket << ")" << std::endl;
						}
					}
					else if(!receive_command(fd))
					{
						std::cout << "Client FD " << fd << " disconnected" << std::endl;
						close(fd);
						FD_CLR(fd, &fds);
					}
				}
			}
		}
	}

	// Disconnect the socket and stop all streams
	disconnect_socket();
	stop_all_streams();

	// Return success
	return true;
}

// Create socket
bool PepperCameraServer::create_socket()
{
	// Create a new socket
	return ((m_socket = socket(AF_INET, SOCK_STREAM, 0)) >= 0);
}

// Initialise socket
bool PepperCameraServer::initialise_socket()
{
	// Configure the socket to bind anyway even if its target address is in TIME_WAIT from a previous connection that was recently closed
	int set_reuse_addr = 1;
	return (setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, (const void*) &set_reuse_addr, sizeof(set_reuse_addr)) == 0);
}

// Bind socket
bool PepperCameraServer::bind_socket()
{
	// Set the server address
	sockaddr_in server_addr = {};
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htons(INADDR_ANY);
	server_addr.sin_port = htons(m_cmd_port);

	// Bind the socket to the address
	return (bind(m_socket, (sockaddr*) &server_addr, sizeof(server_addr)) == 0);
}

// Start listening
bool PepperCameraServer::start_listen()
{
	// Start listening for connection requests on a socket
	return (listen(m_socket, 3) == 0);
}

// Accept a new connection
bool PepperCameraServer::receive_command(int fd)
{
	// Receive data
	ssize_t bytes = recv(fd, m_recv_buffer, RECV_BUFFER_SIZE, 0);
	if(bytes <= 0)
		return false;

	// Handle the received command
	char cmd = m_recv_buffer[0];
	if(cmd == 'B' && bytes == sizeof(StartCameraStruct))
	{
		StartCameraStruct* S = (StartCameraStruct*) m_recv_buffer;
		in_addr ina = {S->ip};
		char target_ip[INET_ADDRSTRLEN];
		const char* target_ip_success = inet_ntop(AF_INET, &ina, target_ip, INET_ADDRSTRLEN);
		if(!target_ip_success)
			printf("Received start video device command but could not parse target IP from: %u\n", S->ip);
		else
		{
			printf("Starting video device /dev/video%u with size %ux%u and quality %u%%, and streaming it to %s:%u\n", S->device, S->width, S->height, S->quality, target_ip_success, S->port);
			start_stream(S, std::string(target_ip_success));
		}
	}
	else if(cmd == 'E' && bytes == sizeof(StopCameraStruct))
	{
		StopCameraStruct* S = (StopCameraStruct*) m_recv_buffer;
		in_addr ina = {S->ip};
		char target_ip[INET_ADDRSTRLEN];
		const char* target_ip_success = inet_ntop(AF_INET, &ina, target_ip, INET_ADDRSTRLEN);
		printf("Stopping video device /dev/video%u from streaming to %s:%u\n", S->device, (target_ip_success ? target_ip_success : "UNKNOWN"), S->port);
		stop_stream(S->device);
	}
	else
	{
		printf("Received unrecognised command bytes: ");
		for(int i = 0; i < bytes; i++)
			printf("%02x ", (unsigned char) m_recv_buffer[i]);
		printf("\n");
	}

	// Return that the file descriptor should remain open for more commands
	return true;
}

// Disconnect socket
void PepperCameraServer::disconnect_socket()
{
	// Close the socket if it exists
	if(m_socket >= 0)
	{
		std::cout << "Shutting down Pepper camera server..." << std::endl;
		shutdown(m_socket, SHUT_RDWR);
		close(m_socket);
		m_socket = -1;
	}
}

// Start camera stream
void PepperCameraServer::start_stream(const StartCameraStruct* S, const std::string& target_ip)
{
	// Stop any existing stream of the device
	stop_stream(S->device);

	// Fork the current process
	int pid = fork();
	if(pid < 0)
		std::cout << "Failed to fork the process in order to start the video device " << (unsigned int) S->device << " camera stream" << std::endl;
	else if(pid != 0)
	{
		std::cout << "Starting video device " << (unsigned int) S->device << " camera stream in PID " << pid << std::endl;
		m_stream_pid[S->device] = pid;
	}
	else
	{
		std::string device = "device=/dev/video" + std::to_string(S->device);
		std::string caps = std::to_string(S->width) + ",height=" + std::to_string(S->height);
		std::string quality = "quality=" + std::to_string(S->quality);
		std::string host = "host=" + target_ip;
		std::string port = "port=" + std::to_string(S->port);
		execlp("gst-launch-0.10", "gst-launch-0.10", "v4l2src", device.c_str(), "!", ("video/x-raw-yuv,width=" + caps).c_str(), "!", "jpegenc", quality.c_str(), "!", "rtpjpegpay", "!", "udpsink", "sync=false", host.c_str(), port.c_str(), NULL);
		execlp("gst-launch-1.0", "gst-launch-1.0", "v4l2src", device.c_str(), "!", ("video/x-raw,width=" + caps).c_str(), "!", "jpegenc", quality.c_str(), "!", "rtpjpegpay", "!", "udpsink", "sync=false", host.c_str(), port.c_str(), NULL);
		std::cout << "Failed to execute GStreamer pipeline process" << std::endl;
		exit(1);
	}
}

// Stop camera stream
void PepperCameraServer::stop_stream(uint8_t device)
{
	// Kill the child stream corresponding to the device
	kill_stream(device);

	// Now to be safe execute a system kill command that searches for gst-launch command lines relating to this device
	system(("pkill -TERM -ef 'gst-launch-[0-9]+\\.[0-9]+\\s+.*=/dev/video" + std::to_string(device) + "(\\s|$)'").c_str());
}

// Stop all camera streams
void PepperCameraServer::stop_all_streams()
{
	// Kill all child stream processes that we started and are still running
	for(int device = 0; device <= MAX_VIDEO_DEVICE; device++)
		kill_stream((uint8_t) device);

	// Now to be safe execute a system kill command that searches for all gst-launch command lines relating to video devices
	system("pkill -TERM -ef 'gst-launch-[0-9]+\\.[0-9]+\\s+.*=/dev/video[0-9]+(\\s|$)'");
}

// Kill a camera stream that we know about
void PepperCameraServer::kill_stream(uint8_t device)
{
	// Kill the child stream process if we started one and it is still running
	int stream_pid = m_stream_pid[device];
	if(stream_pid != 0)
	{
		if(waitpid(stream_pid, NULL, WNOHANG) == 0)
		{
			std::cout << "Stopping video device " << (unsigned int) device << " camera stream by sending SIGTERM to PID " << stream_pid << std::endl;
			kill(stream_pid, SIGTERM);
		}
		m_stream_pid[device] = 0;
	}
}
// EOF
