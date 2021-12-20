// Pepper camera class
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Includes
#include <pepper_camera/pepper_camera.h>
#include <thread>
#include <chrono>

// Namespaces
using namespace pepper_camera;

// Constructor
PepperCamera::PepperCamera(const ros::NodeHandle& nh_interface, const ros::NodeHandle& nh_param) :
	m_nh_interface(nh_interface),
	m_nh_param(nh_param)
{
	// Initialise configuration variables
	m_port = 0;
	m_auto_reopen = false;

	// Initialise GStreamer variables
	m_elem = NULL;
	m_pipeline = NULL;
	m_main_loop = NULL;
}

// Run
void PepperCamera::run()
{
	// Configure and run the required pipeline
	while(ros::ok())
	{
		if(!configure())
		{
			ROS_FATAL("Failed to configure Pepper camera class");
			break;
		}
		if(!init_stream())
		{
			ROS_FATAL("Failed to initialise Pepper camera stream");
			cleanup_stream();
			break;
		}
		if(!run_stream())
		{
			ROS_FATAL("Failed to run Pepper camera stream");
			cleanup_stream();
			break;
		}
		cleanup_stream();
		if(m_auto_reopen)
		{
			ROS_INFO("Reopening Pepper camera stream...");
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
		else
			break;
	}
	ROS_INFO("Pepper camera stream has exited");
}

// Configuration
bool PepperCamera::configure()
{
	// Set configuration variables to fixed values for now
	m_port = 19327;
	m_auto_reopen = false;

	// Return success
	return true;
}

// Initialise the Pepper camera stream
bool PepperCamera::init_stream()
{
	// Initialise the GStreamer library
	if(!gst_is_initialized())
	{
		ROS_INFO("Initialising GStreamer library...");
		gst_init(0, NULL);
		ROS_INFO_STREAM("Loaded " << gst_version_string());
	}

	// Create the pipeline elements
	m_elem = new GSElements();
	m_elem->udpsrc = gst_element_factory_make("udpsrc", "udpsrc");
	m_elem->rtpjpegdepay = gst_element_factory_make("rtpjpegdepay", "rtpjpegdepay");
	m_elem->jpegdec = gst_element_factory_make("jpegdec", "jpegdec");
	m_elem->fpsdisplaysink = gst_element_factory_make("fpsdisplaysink", "fpsdisplaysink");

	// Check that all pipeline elements were successfully created
	if(!m_elem->valid())
	{
		ROS_ERROR("Failed to create some of the required pipeline elements");
		return false;
	}

	// Configure UDP source
	ROS_INFO("Will listen to UDP on port %d", m_port);
	GstCaps* udpsrc_caps = gst_caps_new_simple("application/x-rtp", "encoding-name", G_TYPE_STRING, "JPEG", NULL);
	g_object_set(m_elem->udpsrc, "port", m_port, "caps", udpsrc_caps, NULL);
	gst_caps_unref(udpsrc_caps);

	// Create the pipeline
	m_pipeline = gst_pipeline_new("pipeline");

	// Add all elements to the pipeline
	gst_bin_add_many(GST_BIN(m_pipeline),
		m_elem->udpsrc,
		m_elem->rtpjpegdepay,
		m_elem->jpegdec,
		m_elem->fpsdisplaysink,
		NULL
	);

	// Link all elements with 'always' pads
	if(gst_element_link_many(m_elem->udpsrc, m_elem->rtpjpegdepay, m_elem->jpegdec, m_elem->fpsdisplaysink, NULL) != TRUE)
	{
		ROS_ERROR("Failed to link some of the required pipeline elements");
		return false;
	}

	// Create a main loop
	m_main_loop = g_main_loop_new(NULL, FALSE);

	// Enable and listen to error messages on the bus
	GstBus* bus = gst_element_get_bus(m_pipeline);
	gst_bus_add_signal_watch(bus);
	g_signal_connect(G_OBJECT(bus), "message::error", (GCallback) PepperCamera::error_callback, m_main_loop);
	gst_object_unref(bus);

	// Return success
	return true;
}

// Run the Pepper camera stream
bool PepperCamera::run_stream()
{
	// Run the pipeline
	ROS_INFO("Running Pepper camera stream...");
	if(gst_element_set_state(m_pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
	{
		ROS_ERROR("Failed to change pipeline state to PLAYING");
		return false;
	}

	// Run the main loop
	g_main_loop_run(m_main_loop);

	// Return success
	return true;
}

// Clean up after the Pepper camera stream
void PepperCamera::cleanup_stream()
{
	// Delete the main loop
	if(m_main_loop != NULL)
	{
		g_main_loop_unref(m_main_loop);
		m_main_loop = NULL;
	}

	// Stop and delete the pipeline
	if(m_pipeline != NULL)
	{
		ROS_INFO("Stopping Pepper camera stream...");
		gst_element_set_state(m_pipeline, GST_STATE_NULL);
		ROS_INFO("Deleting Pepper camera stream...");
		gst_object_unref(m_pipeline);
		m_pipeline = NULL;
	}

	// Delete all the pipeline elements
	if(m_elem != NULL)
	{
		delete m_elem;
		m_elem = NULL;
	}
}

// GStreamer error callback
void PepperCamera::error_callback(GstBus* bus, GstMessage* msg, GMainLoop* main_loop)
{
	// Declare variables
	GError *err;
	gchar *debug_info;

	// Print error details to the screen
	gst_message_parse_error(msg, &err, &debug_info);
	ROS_ERROR("Error received from element %s: %s\nDebugging information: %s", GST_OBJECT_NAME(msg->src), err->message, (debug_info ? debug_info : "None"));
	g_clear_error(&err);
	g_free(debug_info);

	// Quit the main loop
	g_main_loop_quit(main_loop);
}
// EOF
