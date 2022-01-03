// Pepper camera class
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Includes
#include <pepper_camera/pepper_camera.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <glib-unix.h>
#include <algorithm>
#include <sstream>
#include <thread>
#include <chrono>

// Namespaces
using namespace pepper_camera;

//
// Constructor
//

// Constructor
PepperCamera::PepperCamera(ros::NodeHandle& nh_interface, ros::NodeHandle& nh_param) :
	m_nh_interface(nh_interface),
	m_nh_param(nh_param),
	m_camera_info_manager(m_nh_interface),
	m_image_transport(m_nh_interface)
{
}

//
// Run
//

// Run the Pepper camera loop
void PepperCamera::run()
{
	// Advertise the config ID publisher
	m_pub_config_id = m_nh_interface.advertise<std_msgs::UInt32>("camera/config_id", 1, true);
	m_pub_config_id.publish(m_config_id);

	// Configure and run the required pipeline
	while(ros::ok())
	{
		m_pending_reconfigure = false;
		if(!configure())
			ROS_FATAL("Failed to configure Pepper camera stream");
		else
		{
			bool ran_stream = false;
			if(!init_stream())
				ROS_FATAL("Failed to initialise Pepper camera stream");
			else if(!run_stream())
				ROS_FATAL("Failed to run Pepper camera stream");
			else
				ran_stream = true;
			cleanup_stream();
			if(ran_stream)
			{
				if(m_pending_reconfigure)
				{
					ROS_INFO("Reconfiguring Pepper camera stream...");
					continue;
				}
				else if(m_config.auto_retry)
				{
					ROS_INFO("Retrying Pepper camera stream...");
					std::this_thread::sleep_for(std::chrono::seconds(1));
					continue;
				}
			}
		}
		break;
	}
	ROS_INFO("Pepper camera stream has exited");

	// Shut down the config ID publisher
	m_pub_config_id.shutdown();
}

//
// Configuration
//

// Reset the configuration variables
void PepperCamera::Config::reset()
{
	// Reset all configuration variables to their default values
	port = 3016;                                   // Port to listen for UDP packets
	auto_retry = false;                            // Whether to auto-retry the camera pipeline if it exits
	publish_jpeg = false;                          // Whether to publish JPEG images
	publish_yuv = false;                           // Whether to publish YUV images
	publish_rgb = true;                            // Whether to publish RGB images
	record_jpegs.clear();                          // Format: path/to/filename%05d.jpg
	record_jpegs_max = 1000;                       // Maximum number of JPEGs to save before the oldest are deleted again (0 = Unlimited)
	record_mjpeg.clear();                          // Format: path/to/filename.mkv
	record_h264.clear();                           // Format: path/to/filename.mkv
	record_h264_bitrate = 1500;                    // Units: kbit/sec
	record_h264_speed = 5;                         // Allowed values: gst-inspect-1.0 x264enc | grep 'speed-preset' -A 15
	record_h264_profile = "constrained-baseline";  // Allowed values: gst-inspect-1.0 x264enc | grep 'profile:'
	preview = false;                               // Whether to show a preview window
	camera_name.clear();                           // Camera name to use (e.g. for topics, camera info manager)
	camera_frame.clear();                          // Camera TF frame (e.g. /camera_top)
	camera_info_url.clear();                       // URL specifying the camera info file to load (see https://docs.ros.org/en/api/camera_info_manager/html/classcamera__info__manager_1_1CameraInfoManager.html)
	time_offset = 0.0;                             // Fixed time offset to apply to the camera frame timestamps (+/- seconds)
	queue_size_mb = 30;                            // Queue byte sizes to use (MB)
	publish_queue_size = 3;                        // Queue size to use for the ROS publishers
}

// Equality operator for configuration variables
bool PepperCamera::Config::operator==(const Config& other)
{
	return (
		port == other.port &&
		auto_retry == other.auto_retry &&
		publish_jpeg == other.publish_jpeg &&
		publish_yuv == other.publish_yuv &&
		publish_rgb == other.publish_rgb &&
		record_jpegs == other.record_jpegs &&
		record_jpegs_max == other.record_jpegs_max &&
		record_mjpeg == other.record_mjpeg &&
		record_h264 == other.record_h264 &&
		record_h264_bitrate == other.record_h264_bitrate &&
		record_h264_speed == other.record_h264_speed &&
		record_h264_profile == other.record_h264_profile &&
		preview == other.preview &&
		camera_name == other.camera_name &&
		camera_frame == other.camera_frame &&
		camera_info_url == other.camera_info_url &&
		time_offset == other.time_offset &&
		queue_size_mb == other.queue_size_mb &&
		publish_queue_size == other.publish_queue_size
	);
}

// Reset the configuration variables
void PepperCamera::reset_config()
{
	// Reset the configuration variables
	m_config.reset();
}

// Configure the Pepper camera loop
bool PepperCamera::configure()
{
	// Update the configuration variables
	return configure(m_config);
}

// Configure the Pepper camera loop (implementation)
bool PepperCamera::configure(Config& config) const
{
	// Reset configuration to default values
	config.reset();

	// Customise configuration variables based on ROS parameters
	m_nh_param.getParam("port", config.port);
	m_nh_param.getParam("auto_retry", config.auto_retry);
	m_nh_param.getParam("publish_jpeg", config.publish_jpeg);
	m_nh_param.getParam("publish_yuv", config.publish_yuv);
	m_nh_param.getParam("publish_rgb", config.publish_rgb);
	m_nh_param.getParam("record_jpegs_path", config.record_jpegs);
	m_nh_param.getParam("record_jpegs_max", config.record_jpegs_max);
	m_nh_param.getParam("record_mjpeg_path", config.record_mjpeg);
	m_nh_param.getParam("record_h264_path", config.record_h264);
	m_nh_param.getParam("record_h264_bitrate", config.record_h264_bitrate);
	m_nh_param.getParam("record_h264_speed", config.record_h264_speed);
	m_nh_param.getParam("record_h264_profile", config.record_h264_profile);
	m_nh_param.getParam("preview", config.preview);
	m_nh_param.getParam("camera_name", config.camera_name);
	m_nh_param.getParam("camera_frame", config.camera_frame);
	m_nh_param.getParam("camera_info_url", config.camera_info_url);
	m_nh_param.getParam("time_offset", config.time_offset);
	m_nh_param.getParam("queue_size_mb", config.queue_size_mb);
	m_nh_param.getParam("publish_queue_size", config.publish_queue_size);

	// Ensure the recording paths have the appropriate extension
	ensure_extension(config.record_jpegs, ".jpg");
	ensure_extension(config.record_mjpeg, ".mkv");
	ensure_extension(config.record_h264, ".mkv");

	// Range checking
	config.record_jpegs_max = std::max(config.record_jpegs_max, 0);
	config.record_h264_bitrate = std::max(config.record_h264_bitrate, 50);
	config.queue_size_mb = std::max(config.queue_size_mb, 1);
	config.publish_queue_size = std::max(config.publish_queue_size, 1);

	// Dynamic default values
	if(config.camera_name.empty())
		config.camera_name = "top";
	if(config.camera_frame.empty())
		config.camera_frame = "/camera_" + config.camera_name;
	if(config.camera_info_url.empty())
		config.camera_info_url = "package://pepper_camera/calibration/pepper_" + config.camera_name + ".yaml";

	// Return success
	return true;
}

//
// Stream management
//

// Initialise the Pepper camera stream
bool PepperCamera::init_stream()
{
	// Print that the stream is being initialised
	ROS_INFO("Initialising Pepper camera stream...");

	// Initialise the GStreamer library
	if(!gst_is_initialized())
	{
		gst_init(0, NULL);
		ROS_INFO("Loaded %s", gst_version_string());
	}

	// Flags whether certain parts of the pipeline are required
	bool record_jpegs = !m_config.record_jpegs.empty();
	bool record_mjpeg = !m_config.record_mjpeg.empty();
	bool record_h264 = !m_config.record_h264.empty();
	int tee_yuv_count = m_config.publish_yuv + m_config.publish_rgb + record_h264 + m_config.preview;
	bool have_tee_yuv = (tee_yuv_count >= 2);
	bool have_jpegdec = (tee_yuv_count >= 1);
	int tee_jpeg_count = m_config.publish_jpeg + record_jpegs + record_mjpeg + have_jpegdec;
	bool have_tee_jpeg = (tee_jpeg_count >= 2);
	bool have_inspect = (tee_jpeg_count < 1);

	// Advertise ROS interface
	std::string camera_base = "camera/" + m_config.camera_name;
	if(m_config.publish_jpeg || m_config.publish_yuv || m_config.publish_rgb)
	{
		m_camera_info_manager.setCameraName(m_config.camera_name);
		std::string camera_info_url;
		if(m_camera_info_manager.validateURL(m_config.camera_info_url))
			camera_info_url = m_config.camera_info_url;
		m_camera_info_manager.loadCameraInfo(m_config.camera_info_url);
		m_pub_camera_info = m_nh_interface.advertise<sensor_msgs::CameraInfo>(camera_base + "/camera_info", m_config.publish_queue_size);
		ROS_INFO("Configured a ROS publisher queue length of %d", m_config.publish_queue_size);
	}
	m_pub_camera_info_stamp.fromNSec(0);
	if(m_config.publish_jpeg)
		m_pub_jpeg = m_nh_interface.advertise<sensor_msgs::CompressedImage>(camera_base + "/jpeg/compressed", m_config.publish_queue_size);
	if(m_config.publish_yuv)
		m_pub_yuv = m_nh_interface.advertise<sensor_msgs::Image>(camera_base + "/yuv", m_config.publish_queue_size);
	if(m_config.publish_rgb)
		m_pub_rgb = m_image_transport.advertise(camera_base + "/rgb", m_config.publish_queue_size);

	// Create the required pipeline elements
	bool any_elem_invalid = false;
	m_elem = new GSElements();
	any_elem_invalid |= !(m_elem->udpsrc = gst_element_factory_make("udpsrc", "udpsrc"));
	any_elem_invalid |= !(m_elem->rtpjpegdepay = gst_element_factory_make("rtpjpegdepay", "rtpjpegdepay"));
	if(have_tee_jpeg)
	{
		any_elem_invalid |= !(m_elem->tee_jpeg = gst_element_factory_make("tee", "tee_jpeg"));
		if(m_config.publish_jpeg)
			any_elem_invalid |= !(m_elem->publish_jpeg_queue = gst_element_factory_make("queue", "publish_jpeg_queue"));
		if(record_jpegs)
			any_elem_invalid |= !(m_elem->record_jpegs_queue = gst_element_factory_make("queue", "record_jpegs_queue"));
		if(record_mjpeg)
			any_elem_invalid |= !(m_elem->record_mjpeg_queue = gst_element_factory_make("queue", "record_mjpeg_queue"));
		if(have_jpegdec)
			any_elem_invalid |= !(m_elem->jpegdec_queue = gst_element_factory_make("queue", "jpegdec_queue"));
	}
	if(m_config.publish_jpeg)
		any_elem_invalid |= !(m_elem->publish_jpeg = gst_element_factory_make("appsink", "publish_jpeg"));
	if(record_jpegs)
		any_elem_invalid |= !(m_elem->record_jpegs = gst_element_factory_make("multifilesink", "record_jpegs"));
	if(record_mjpeg)
	{
		any_elem_invalid |= !(m_elem->record_mjpeg_mux = gst_element_factory_make("matroskamux", "record_mjpeg_mux"));
		any_elem_invalid |= !(m_elem->record_mjpeg = gst_element_factory_make("filesink", "record_mjpeg"));
	}
	if(have_jpegdec)
		any_elem_invalid |= !(m_elem->jpegdec = gst_element_factory_make("jpegdec", "jpegdec"));
	if(have_tee_yuv)
	{
		any_elem_invalid |= !(m_elem->tee_yuv = gst_element_factory_make("tee", "tee_yuv"));
		if(m_config.publish_yuv)
			any_elem_invalid |= !(m_elem->publish_yuv_queue = gst_element_factory_make("queue", "publish_yuv_queue"));
		if(m_config.publish_rgb)
			any_elem_invalid |= !(m_elem->publish_rgb_queue = gst_element_factory_make("queue", "publish_rgb_queue"));
		if(record_h264)
			any_elem_invalid |= !(m_elem->record_h264_queue = gst_element_factory_make("queue", "record_h264_queue"));
		if(m_config.preview)
			any_elem_invalid |= !(m_elem->preview_queue = gst_element_factory_make("queue", "preview_queue"));
	}
	if(m_config.publish_yuv)
		any_elem_invalid |= !(m_elem->publish_yuv = gst_element_factory_make("appsink", "publish_yuv"));
	if(m_config.publish_rgb)
	{
		any_elem_invalid |= !(m_elem->publish_rgb_convert = gst_element_factory_make("videoconvert", "publish_rgb_convert"));
		any_elem_invalid |= !(m_elem->publish_rgb = gst_element_factory_make("appsink", "publish_rgb"));
	}
	if(record_h264)
	{
		any_elem_invalid |= !(m_elem->record_h264_enc = gst_element_factory_make("x264enc", "record_h264_enc"));
		any_elem_invalid |= !(m_elem->record_h264_mux = gst_element_factory_make("matroskamux", "record_h264_mux"));
		any_elem_invalid |= !(m_elem->record_h264 = gst_element_factory_make("filesink", "record_h264"));
	}
	if(m_config.preview)
		any_elem_invalid |= !(m_elem->preview = gst_element_factory_make("fpsdisplaysink", "preview"));
	if(have_inspect)
		any_elem_invalid |= !(m_elem->inspect = gst_element_factory_make("fakesink", "inspect"));

	// Abort if not all pipeline elements were created successfully
	if(any_elem_invalid)
	{
		ROS_ERROR("Failed to create some of the required pipeline elements");
		return false;
	}

	// Create the pipeline
	m_pipeline = gst_pipeline_new("pipeline");
	m_pipeline_stalled = false;
	GstBin* pipeline_bin = GST_BIN(m_pipeline);

	// Configure/add the UDP source
	GstCaps* udpsrc_caps = gst_caps_new_simple("application/x-rtp", "encoding-name", G_TYPE_STRING, "JPEG", NULL);
	g_object_set(m_elem->udpsrc, "port", (gint) m_config.port, "caps", udpsrc_caps, NULL);
	gst_caps_unref(udpsrc_caps);
	gst_bin_add_many_ref(pipeline_bin, m_elem->udpsrc, m_elem->rtpjpegdepay, NULL);

	// Configure/add the JPEG tee
	if(m_elem->tee_jpeg)
		gst_bin_add_ref(pipeline_bin, m_elem->tee_jpeg);

	// Configure/add the JPEG publisher app sink
	if(m_elem->publish_jpeg)
	{
		configure_queue(m_elem->publish_jpeg_queue);
		GstCaps* publish_jpeg_caps = gst_caps_new_empty_simple("image/jpeg");
		g_object_set(m_elem->publish_jpeg, "caps", publish_jpeg_caps, "emit-signals", TRUE, "sync", FALSE, NULL);
		gst_caps_unref(publish_jpeg_caps);
		g_signal_connect(m_elem->publish_jpeg, "new-sample", G_CALLBACK(PepperCamera::publish_jpeg_callback), this);
		gst_bin_add_many_ref(pipeline_bin, m_elem->publish_jpeg, m_elem->publish_jpeg_queue, NULL);
	}

	// Configure/add the JPEGs recorder
	if(m_elem->record_jpegs)
	{
		configure_queue(m_elem->record_jpegs_queue);
		g_object_set(m_elem->record_jpegs, "location", m_config.record_jpegs.c_str(), "index", (gint) 1, "max-files", (guint) m_config.record_jpegs_max, "sync", FALSE, NULL);
		gst_bin_add_many_ref(pipeline_bin, m_elem->record_jpegs, m_elem->record_jpegs_queue, NULL);
	}

	// Configure/add the MJPEG recorder
	if(m_elem->record_mjpeg)
	{
		configure_queue(m_elem->record_mjpeg_queue);
		g_object_set(m_elem->record_mjpeg, "location", m_config.record_mjpeg.c_str(), "sync", FALSE, NULL);
		gst_bin_add_many_ref(pipeline_bin, m_elem->record_mjpeg_mux, m_elem->record_mjpeg, m_elem->record_mjpeg_queue, NULL);
	}

	// Configure/add the JPEG decoder
	if(m_elem->jpegdec)
	{
		configure_queue(m_elem->jpegdec_queue);
		gst_bin_add_many_ref(pipeline_bin, m_elem->jpegdec, m_elem->jpegdec_queue, NULL);
	}

	// Configure/add the YUV tee
	if(m_elem->tee_yuv)
		gst_bin_add_ref(pipeline_bin, m_elem->tee_yuv);

	// Configure/add the YUV publisher app sink
	if(m_elem->publish_yuv)
	{
		configure_queue(m_elem->publish_yuv_queue);
		GstCaps* publish_yuv_caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "I420", NULL);  // TODO: Want another YUV format compatible with ROS?
		g_object_set(m_elem->publish_yuv, "caps", publish_yuv_caps, "emit-signals", TRUE, "sync", FALSE, NULL);
		gst_caps_unref(publish_yuv_caps);
		g_signal_connect(m_elem->publish_yuv, "new-sample", G_CALLBACK(PepperCamera::publish_yuv_callback), this);
		gst_bin_add_many_ref(pipeline_bin, m_elem->publish_yuv, m_elem->publish_yuv_queue, NULL);
	}

	// Configure/add the RGB publisher app sink
	if(m_elem->publish_rgb)
	{
		configure_queue(m_elem->publish_rgb_queue);
		GstCaps* publish_rgb_caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "RGB", NULL);
		g_object_set(m_elem->publish_rgb, "caps", publish_rgb_caps, "emit-signals", TRUE, "sync", FALSE, NULL);
		gst_caps_unref(publish_rgb_caps);
		g_signal_connect(m_elem->publish_rgb, "new-sample", G_CALLBACK(PepperCamera::publish_rgb_callback), this);
		gst_bin_add_many_ref(pipeline_bin, m_elem->publish_rgb_convert, m_elem->publish_rgb, m_elem->publish_rgb_queue, NULL);
	}

	// Configure/add the H264 recorder
	if(m_elem->record_h264)
	{
		configure_queue(m_elem->record_h264_queue);
		g_object_set(m_elem->record_h264_enc, "pass", 0, "bitrate", (guint) m_config.record_h264_bitrate, "speed-preset", m_config.record_h264_speed, NULL);
		g_object_set(m_elem->record_h264, "location", m_config.record_h264.c_str(), "sync", FALSE, NULL);
		gst_bin_add_many_ref(pipeline_bin, m_elem->record_h264_enc, m_elem->record_h264_mux, m_elem->record_h264, m_elem->record_h264_queue, NULL);
	}

	// Configure/add the preview window sink
	if(m_elem->preview)
	{
		configure_queue(m_elem->preview_queue);
		g_object_set(m_elem->preview, "text-overlay", TRUE, "sync", FALSE, NULL);
		gst_bin_add_many_ref(pipeline_bin, m_elem->preview, m_elem->preview_queue, NULL);
	}

	// Configure/add the inspect sink
	if(m_elem->inspect)
	{
		g_object_set(m_elem->inspect, "signal-handoffs", TRUE, "sync", FALSE, NULL);
		g_signal_connect(m_elem->inspect, "handoff", G_CALLBACK(PepperCamera::inspect_callback), this);
		gst_bin_add_ref(pipeline_bin, m_elem->inspect);
	}

	// Display selected configuration information
	ROS_INFO_COND(m_elem->tee_jpeg || m_elem->tee_yuv, "Configured a GStreamer queue size of %dMB", m_config.queue_size_mb);

	// Link the UDP source
	int link_success = TRUE;
	link_success &= gst_element_link(m_elem->udpsrc, m_elem->rtpjpegdepay);

	// Link the JPEG over RTP depayloader source/tee
	if(m_elem->tee_jpeg)
	{
		link_success &= gst_element_link(m_elem->rtpjpegdepay, m_elem->tee_jpeg);
		if(m_elem->publish_jpeg_queue)
			link_success &= link_tee_queue(m_elem->tee_jpeg, m_elem->publish_jpeg_queue, m_elem->publish_jpeg_pad) & gst_element_link(m_elem->publish_jpeg_queue, m_elem->publish_jpeg);
		if(m_elem->record_jpegs_queue)
			link_success &= link_tee_queue(m_elem->tee_jpeg, m_elem->record_jpegs_queue, m_elem->record_jpegs_pad) & gst_element_link(m_elem->record_jpegs_queue, m_elem->record_jpegs);
		if(m_elem->record_mjpeg_queue)
			link_success &= link_tee_queue(m_elem->tee_jpeg, m_elem->record_mjpeg_queue, m_elem->record_mjpeg_pad) & gst_element_link(m_elem->record_mjpeg_queue, m_elem->record_mjpeg_mux);
		if(m_elem->jpegdec_queue)
			link_success &= link_tee_queue(m_elem->tee_jpeg, m_elem->jpegdec_queue, m_elem->jpegdec_pad) & gst_element_link(m_elem->jpegdec_queue, m_elem->jpegdec);
	}
	else if(m_elem->publish_jpeg)
		link_success &= gst_element_link(m_elem->rtpjpegdepay, m_elem->publish_jpeg);
	else if(m_elem->record_jpegs)
		link_success &= gst_element_link(m_elem->rtpjpegdepay, m_elem->record_jpegs);
	else if(m_elem->record_mjpeg_mux)
		link_success &= gst_element_link(m_elem->rtpjpegdepay, m_elem->record_mjpeg_mux);
	else if(m_elem->jpegdec)
		link_success &= gst_element_link(m_elem->rtpjpegdepay, m_elem->jpegdec);
	else if(m_elem->inspect)
		link_success &= gst_element_link(m_elem->rtpjpegdepay, m_elem->inspect);
	else
	{
		ROS_ERROR("Failed to link src pad of element: rtpjpegdepay");
		return false;
	}

	// Link the MJPEG recorder
	if(m_elem->record_mjpeg)
		link_success &= gst_element_link(m_elem->record_mjpeg_mux, m_elem->record_mjpeg);

	// Link the JPEG decoder source/tee
	if(m_elem->jpegdec)
	{
		if(m_elem->tee_yuv)
		{
			link_success &= gst_element_link(m_elem->jpegdec, m_elem->tee_yuv);
			if(m_elem->publish_yuv_queue)
				link_success &= link_tee_queue(m_elem->tee_yuv, m_elem->publish_yuv_queue, m_elem->publish_yuv_pad) & gst_element_link(m_elem->publish_yuv_queue, m_elem->publish_yuv);
			if(m_elem->publish_rgb_queue)
				link_success &= link_tee_queue(m_elem->tee_yuv, m_elem->publish_rgb_queue, m_elem->publish_rgb_pad) & gst_element_link(m_elem->publish_rgb_queue, m_elem->publish_rgb_convert);
			if(m_elem->record_h264_queue)
				link_success &= link_tee_queue(m_elem->tee_yuv, m_elem->record_h264_queue, m_elem->record_h264_pad) & gst_element_link(m_elem->record_h264_queue, m_elem->record_h264_enc);
			if(m_elem->preview_queue)
				link_success &= link_tee_queue(m_elem->tee_yuv, m_elem->preview_queue, m_elem->preview_pad) & gst_element_link(m_elem->preview_queue, m_elem->preview);
		}
		else if(m_elem->publish_yuv)
			link_success &= gst_element_link(m_elem->jpegdec, m_elem->publish_yuv);
		else if(m_elem->publish_rgb_convert)
			link_success &= gst_element_link(m_elem->jpegdec, m_elem->publish_rgb_convert);
		else if(m_elem->record_h264_enc)
			link_success &= gst_element_link(m_elem->jpegdec, m_elem->record_h264_enc);
		else if(m_elem->preview)
			link_success &= gst_element_link(m_elem->jpegdec, m_elem->preview);
		else
		{
			ROS_ERROR("Failed to link src pad of element: jpegdec");
			return false;
		}
	}

	// Link the RGB publisher
	if(m_elem->publish_rgb)
		link_success &= gst_element_link(m_elem->publish_rgb_convert, m_elem->publish_rgb);

	// Link the H264 recorder
	if(m_elem->record_h264)
	{
		GstCaps* record_h264_caps = gst_caps_new_simple("video/x-h264", "stream-format", G_TYPE_STRING, "avc", "profile", G_TYPE_STRING, m_config.record_h264_profile.c_str(), NULL);
		link_success &= gst_element_link_filtered(m_elem->record_h264_enc, m_elem->record_h264_mux, record_h264_caps);
		gst_caps_unref(record_h264_caps);
		link_success &= gst_element_link(m_elem->record_h264_mux, m_elem->record_h264);
	}

	// Abort if not all pipeline elements were linked successfully
	if(link_success != TRUE)
	{
		ROS_ERROR("Failed to link some of the required pipeline elements");
		ROS_ERROR("Try: GST_DEBUG=4 COMMAND |& grep fail");
		return false;
	}
	else
	{
		GstPad* unlinked_src_pad = gst_bin_find_unlinked_pad(pipeline_bin, GST_PAD_SRC);
		GstPad* unlinked_sink_pad = gst_bin_find_unlinked_pad(pipeline_bin, GST_PAD_SINK);
		bool have_unlinked = (unlinked_src_pad || unlinked_sink_pad);
		if(unlinked_src_pad)
		{
			GstElement* unlinked_src_elem = gst_pad_get_parent_element(unlinked_src_pad);
			ROS_ERROR("Found unlinked src pad on element: %s", (unlinked_src_elem ? GST_ELEMENT_NAME(unlinked_src_elem) : "Unknown"));
			if(unlinked_src_elem)
				gst_object_unref(unlinked_src_elem);
			gst_object_unref(unlinked_src_pad);
		}
		if(unlinked_sink_pad)
		{
			GstElement* unlinked_sink_elem = gst_pad_get_parent_element(unlinked_sink_pad);
			ROS_ERROR("Found unlinked sink pad on element: %s", (unlinked_sink_elem ? GST_ELEMENT_NAME(unlinked_sink_elem) : "Unknown"));
			if(unlinked_sink_elem)
				gst_object_unref(unlinked_sink_elem);
			gst_object_unref(unlinked_sink_pad);
		}
		if(have_unlinked)
			return false;
	}

	// Create a main loop
	m_main_loop = g_main_loop_new(NULL, FALSE);

	// Listen to selected messages on the bus
	m_bus = gst_element_get_bus(m_pipeline);
	gst_bus_add_signal_watch(m_bus);
	g_signal_connect(G_OBJECT(m_bus), "message::eos", G_CALLBACK(PepperCamera::stream_eos_callback), this);
	g_signal_connect(G_OBJECT(m_bus), "message::warning", G_CALLBACK(PepperCamera::stream_warning_callback), this);
	g_signal_connect(G_OBJECT(m_bus), "message::error", G_CALLBACK(PepperCamera::stream_error_callback), this);
	g_signal_connect(G_OBJECT(m_bus), "message::state-changed", G_CALLBACK(PepperCamera::stream_state_changed_callback), this);

	// Add unix signal handler
	m_sigint_callback_id = g_unix_signal_add(SIGINT, PC_G_SOURCE_FUNC(PepperCamera::sigint_callback), this);

	// GStreamer and ROS time calibration
	GstClock* pipeline_clock = gst_pipeline_get_clock(GST_PIPELINE_CAST(m_pipeline));
	GstClockTime pipeline_now_a = gst_clock_get_time(pipeline_clock);
	ros::Time ros_now = ros::Time::now();
	GstClockTime pipeline_now_b = gst_clock_get_time(pipeline_clock);
	m_pipeline_time_offset.fromSec(ros_now.toSec() - (pipeline_now_a + pipeline_now_b) / (2.0 * GST_SECOND));
	gst_object_unref(pipeline_clock);

	// Return success
	return true;
}

// Run the Pepper camera stream
bool PepperCamera::run_stream()
{
	// Summarise the pipeline that's about to run
	ROS_INFO("Running Pepper camera stream...");
	ROS_INFO("Listening to UDP packets on port %d", m_config.port);
	ROS_INFO("Streaming %s camera (TF frame: %s)", m_config.camera_name.c_str(), m_config.camera_frame.c_str());
	ROS_INFO_COND(m_config.time_offset != 0.0, "Applying frame timestamp offset of %+.3fs", m_config.time_offset);
	ROS_INFO_COND(m_elem->publish_jpeg || m_elem->publish_yuv || m_elem->publish_rgb, "Publishing camera info on topic: %s", m_pub_camera_info.getTopic().c_str());
	ROS_INFO_COND(m_elem->publish_jpeg, "Publishing JPEG images on topic: %s", m_pub_jpeg.getTopic().c_str());
	ROS_INFO_COND(m_elem->publish_yuv, "Publishing YUV images on topic: %s", m_pub_yuv.getTopic().c_str());
	ROS_INFO_COND(m_elem->publish_rgb, "Publishing RGB images on topic: %s", m_pub_rgb.getTopic().c_str());
	if(m_elem->record_jpegs)
	{
		if(m_config.record_jpegs_max == 0)
			ROS_INFO("Recording unlimited JPEG frames to: %s", m_config.record_jpegs.c_str());
		else
			ROS_INFO("Recording latest %d JPEG frames to: %s", m_config.record_jpegs_max, m_config.record_jpegs.c_str());
	}
	ROS_INFO_COND(m_elem->record_mjpeg, "Recording MJPEG video to: %s", m_config.record_mjpeg.c_str());
	ROS_INFO_COND(m_elem->record_h264, "Recording H264 video to: %s", m_config.record_h264.c_str());

	// Start the pipeline
	if(gst_element_set_state(m_pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
	{
		ROS_ERROR("Failed to change pipeline state to PLAYING");
		return false;
	}

	// Update and publish the config ID
	m_config_id.data++;
	ROS_INFO("Running pipeline with config ID %u", m_config_id.data);
	m_pub_config_id.publish(m_config_id);

	// Advertise the reconfigure service
	m_srv_reconfigure = m_nh_interface.advertiseService("camera/" + m_config.camera_name + "/reconfigure", &PepperCamera::handle_reconfigure, this);

	// Run the main loop
	ROS_INFO("Waiting for UDP data to arrive...");
	g_main_loop_run(m_main_loop);

	// Unadvertise the reconfigure service
	m_srv_reconfigure.shutdown();

	// Return success
	return true;
}

// Clean up after the Pepper camera stream
void PepperCamera::cleanup_stream()
{
	// Print that the stream is being cleaned up
	ROS_INFO("Cleaning up Pepper camera stream...");

	// Remove unix signal handler
	if(m_sigint_callback_id > 0U)
	{
		g_source_remove(m_sigint_callback_id);
		m_sigint_callback_id = 0U;
	}

	// Remove the bus signal watch
	if(m_bus != NULL)
	{
		gst_bus_remove_signal_watch(m_bus);
		gst_object_unref(m_bus);
		m_bus = NULL;
	}

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
		gst_object_unref(m_pipeline);
		m_pipeline = NULL;
	}

	// Delete all the pipeline elements
	if(m_elem != NULL)
	{
		m_elem->clear();
		delete m_elem;
		m_elem = NULL;
	}

	// Shut down the ROS interface
	m_pub_camera_info.shutdown();
	m_pub_jpeg.shutdown();
	m_pub_yuv.shutdown();
	m_pub_rgb.shutdown();
}

//
// ROS callbacks
//

// Reconfigure service handler
bool PepperCamera::handle_reconfigure(Reconfigure::Request& req, Reconfigure::Response& res)
{
	// Display that a reconfigure event has occurred
	ROS_INFO("Received reconfigure service call%s", (req.force ? " (force)..." : "..."));

	// Check whether a reconfigure is needed
	bool reconfigure = true;
	if(!req.force)
	{
		Config new_config(false);
		if(!configure(new_config))
		{
			ROS_ERROR("Failed to reconfigure Pepper camera stream => Leaving stream as it is...");
			res.config_id = m_config_id.data;
			return false;
		}
		reconfigure = (new_config != m_config);
	}

	// Trigger the reconfigure if required
	if(reconfigure)
	{
		if(!m_pending_reconfigure)
		{
			m_pending_reconfigure = true;
			post_cancel_main_loop();
		}
		res.config_id = m_config_id.data + 1U;
	}
	else
	{
		ROS_INFO("Configuration is unchanged => No reconfigure action required");
		res.config_id = m_config_id.data;
	}

	// Return that the service call succeeded
	return true;
}

//
// ROS utilities
//

// Publish a new camera info message
void PepperCamera::publish_camera_info(const ros::Time& stamp)
{
	// Publish a camera info message if the timestamp is new
	if(stamp != m_pub_camera_info_stamp)
	{
		m_pub_camera_info_stamp = stamp;
		sensor_msgs::CameraInfoPtr camera_info(new sensor_msgs::CameraInfo(m_camera_info_manager.getCameraInfo()));
		camera_info->header.frame_id = m_config.camera_frame;
		camera_info->header.stamp = stamp;
		m_pub_camera_info.publish(camera_info);
	}
}

//
// GSElements
//

// Clear GStreamer pipeline elements
void pepper_camera::PepperCamera::GSElements::clear()
{
	// Release all request pads
	if(tee_jpeg)
	{
		if(publish_jpeg_pad) gst_element_release_request_pad(tee_jpeg, publish_jpeg_pad);
		if(record_jpegs_pad) gst_element_release_request_pad(tee_jpeg, record_jpegs_pad);
		if(record_mjpeg_pad) gst_element_release_request_pad(tee_jpeg, record_mjpeg_pad);
		if(jpegdec_pad)      gst_element_release_request_pad(tee_jpeg, jpegdec_pad);
	}
	if(tee_yuv)
	{
		if(publish_yuv_pad) gst_element_release_request_pad(tee_yuv, publish_yuv_pad);
		if(publish_rgb_pad) gst_element_release_request_pad(tee_yuv, publish_rgb_pad);
		if(record_h264_pad) gst_element_release_request_pad(tee_yuv, record_h264_pad);
		if(preview_pad)     gst_element_release_request_pad(tee_yuv, preview_pad);
	}

	// Unref all elements if they exist and reset the pointers to NULL
	gst_object_unref_safe((GstObject**) &udpsrc);
	gst_object_unref_safe((GstObject**) &rtpjpegdepay);
	gst_object_unref_safe((GstObject**) &tee_jpeg);
	gst_object_unref_safe((GstObject**) &publish_jpeg_pad);
	gst_object_unref_safe((GstObject**) &publish_jpeg_queue);
	gst_object_unref_safe((GstObject**) &publish_jpeg);
	gst_object_unref_safe((GstObject**) &record_jpegs_pad);
	gst_object_unref_safe((GstObject**) &record_jpegs_queue);
	gst_object_unref_safe((GstObject**) &record_jpegs);
	gst_object_unref_safe((GstObject**) &record_mjpeg_pad);
	gst_object_unref_safe((GstObject**) &record_mjpeg_queue);
	gst_object_unref_safe((GstObject**) &record_mjpeg_mux);
	gst_object_unref_safe((GstObject**) &record_mjpeg);
	gst_object_unref_safe((GstObject**) &jpegdec_pad);
	gst_object_unref_safe((GstObject**) &jpegdec_queue);
	gst_object_unref_safe((GstObject**) &jpegdec);
	gst_object_unref_safe((GstObject**) &tee_yuv);
	gst_object_unref_safe((GstObject**) &publish_yuv_pad);
	gst_object_unref_safe((GstObject**) &publish_yuv_queue);
	gst_object_unref_safe((GstObject**) &publish_yuv);
	gst_object_unref_safe((GstObject**) &publish_rgb_pad);
	gst_object_unref_safe((GstObject**) &publish_rgb_queue);
	gst_object_unref_safe((GstObject**) &publish_rgb_convert);
	gst_object_unref_safe((GstObject**) &publish_rgb);
	gst_object_unref_safe((GstObject**) &record_h264_pad);
	gst_object_unref_safe((GstObject**) &record_h264_queue);
	gst_object_unref_safe((GstObject**) &record_h264_enc);
	gst_object_unref_safe((GstObject**) &record_h264_mux);
	gst_object_unref_safe((GstObject**) &record_h264);
	gst_object_unref_safe((GstObject**) &preview_pad);
	gst_object_unref_safe((GstObject**) &preview_queue);
	gst_object_unref_safe((GstObject**) &preview);
	gst_object_unref_safe((GstObject**) &inspect);
}

//
// GStreamer callbacks
//

// GStreamer SIGINT callback
gboolean PepperCamera::sigint_callback(PepperCamera* pc)
{
	// Display that SIGINT signal was caught
	ROS_INFO("Caught SIGINT signal");

	// Cancel the main loop
	pc->cancel_main_loop();

	// Keep the signal handler in place
	return G_SOURCE_CONTINUE;
}

// GStreamer EOS callback
void PepperCamera::stream_eos_callback(GstBus* bus, GstMessage* msg, PepperCamera* pc)
{
	// Display that end of stream (EOS) has occurred
	ROS_INFO("End of stream (EOS) occurred");

	// Quit the main loop
	pc->quit_main_loop();
}

// GStreamer warning callback
void PepperCamera::stream_warning_callback(GstBus* bus, GstMessage* msg, PepperCamera* pc)
{
	// Declare variables
	GError *err;
	gchar *debug;

	// Print warning details to the screen
	gst_message_parse_warning(msg, &err, &debug);
	ROS_WARN("Warning received from element %s: %s", (msg->src ? GST_OBJECT_NAME(msg->src) : "unknown"), err->message);
	ROS_WARN("Debugging information: %s", (debug ? debug : "None"));
	g_clear_error(&err);
	g_free(debug);
}

// GStreamer error callback
void PepperCamera::stream_error_callback(GstBus* bus, GstMessage* msg, PepperCamera* pc)
{
	// Declare variables
	GError *err;
	gchar *debug;

	// Print error details to the screen
	gst_message_parse_error(msg, &err, &debug);
	if(std::strcmp(err->message, "Output window was closed") != 0)
	{
		ROS_ERROR("Error received from element %s: %s", (msg->src ? GST_OBJECT_NAME(msg->src) : "unknown"), err->message);
		ROS_ERROR("Debugging information: %s", (debug ? debug : "None"));
		pc->m_pipeline_stalled = true;
	}
	g_clear_error(&err);
	g_free(debug);

	// Cancel the main loop
	pc->cancel_main_loop();
}

// GStreamer state changed callback
void PepperCamera::stream_state_changed_callback(GstBus* bus, GstMessage* msg, PepperCamera* pc)
{
	// Wait for the pipeline to enter the PLAYING state
	if(msg->src == GST_OBJECT_CAST(pc->m_pipeline))
	{
		GstState new_state;
		gst_message_parse_state_changed(msg, NULL, &new_state, NULL);
		if(new_state == GST_STATE_PLAYING)
			ROS_INFO("Started receiving UDP camera frames");
	}
}

// Queue overrun callback
void PepperCamera::queue_overrun_callback(GstElement* queue, PepperCamera* pc)
{
	// A queue overrun makes the pipeline stall
	pc->m_pipeline_stalled = true;

	// Retrieve the current queue levels
	guint64 cur_time = 0LU;
	guint cur_buffers = 0U, cur_bytes = 0U;
	g_object_get(queue, "current-level-buffers", &cur_buffers, "current-level-bytes", &cur_bytes, "current-level-time", &cur_time, NULL);

	// Display that a queue overrun has occurred
	ROS_WARN("Queue overrun of %s: Contains %u buffers, %.1fMB, %.3fs", GST_OBJECT_NAME(queue), cur_buffers, cur_bytes / 1048576.0, cur_time * 1e-9);
	ROS_WARN("Try increasing the queue MB limit using queue_size_mb ROS param?");
}

// Publish image callback
GstFlowReturn PepperCamera::publish_callback(GstElement* appsink, PublishImageType type, const char* name)
{
	// Declare variables
	GstFlowReturn flow = GST_FLOW_ERROR;

	// Retrieve and process the sample
	GstSample* sample;
	g_signal_emit_by_name(appsink, "pull-sample", &sample);
	if(!sample)
		ROS_ERROR("%s publisher failed to pull a sample", name);
	else
	{
		// Retrieve and process the sample buffer
		GstBuffer* buffer = gst_sample_get_buffer(sample);
		GstCaps* caps = gst_sample_get_caps(sample);
		if(!buffer)
			ROS_ERROR("%s publisher failed to obtain the buffer for a pulled sample", name);
		else if(!caps)
			ROS_ERROR("%s publisher failed to obtain the caps for a pulled sample", name);
		else
		{
			// Retrieve and process the sample buffer memory
			guint num_memories = gst_buffer_n_memory(buffer);
			guint num_caps_structs = gst_caps_get_size(caps);
			if(num_memories < 1U)
				ROS_ERROR("%s publisher received a buffer with no memory", name);
			else if(num_caps_structs < 1U)
				ROS_ERROR("%s publisher received a caps with no structures", name);
			else
			{
				// One-time warnings for silently ignored memories/structures
				if(num_memories != 1U)
					ROS_WARN_ONCE("%s publisher is receiving buffers with multiple memories in it => Always just taking first one", name);
				if(num_caps_structs != 1U)
					ROS_WARN_ONCE("%s publisher is receiving caps with multiple structures in it => Always just taking first one", name);

				// Retrieve and process the raw data inside the sample buffer memory
				GstMemory *memory = gst_buffer_peek_memory(buffer, 0U);
				GstStructure* caps_struct = gst_caps_get_structure(caps, 0U);
				gint caps_width = 0, caps_height = 0;
				const gchar* caps_format = NULL;
				GstMapInfo memory_info;
				if(!memory)
					ROS_ERROR("%s publisher failed to get a pointer to the sample buffer memory", name);
				else if(!caps_struct)
					ROS_ERROR("%s publisher failed to get the internal caps structure", name);
				else if(!gst_structure_get_int(caps_struct, "width", &caps_width) || !gst_structure_get_int(caps_struct, "height", &caps_height) || caps_width < 1 || caps_height < 1)
					ROS_ERROR("%s publisher failed to determine the sample image dimensions from the caps", name);
				else if(type != PIT_JPEG && !(caps_format = gst_structure_get_string(caps_struct, "format")))
					ROS_ERROR("%s publisher failed to determine the sample image format from the caps", name);
				else if(!gst_memory_map(memory, &memory_info, GST_MAP_READ))
					ROS_ERROR("%s publisher failed to extract the raw data from the sample buffer memory", name);
				else
				{
					// Retrieve the raw data pointer and size
					gsize& data_size = memory_info.size;
					guint8*& data_ptr = memory_info.data;
					ros::Time data_stamp((m_pipeline->base_time + buffer->pts) * 1e-9 + m_pipeline_time_offset.toSec() + m_config.time_offset);

					// Publish a camera info message
					publish_camera_info(data_stamp);

					// Different publishing behaviour depending on the image type
					if(type == PIT_JPEG)
					{
						// Display info about the data that the ROS publisher is receiving
						static gint cur_caps_width = 0, cur_caps_height = 0;
						if(cur_caps_width != caps_width || cur_caps_height != caps_height)
						{
							cur_caps_width = caps_width;
							cur_caps_height = caps_height;
							ROS_INFO("JPEG publisher is receiving %dx%d JPEG images (~%" G_GSIZE_FORMAT " bytes)", cur_caps_width, cur_caps_height, 1000U * ((data_size + 500U) / 1000U));
						}

						// Publish an image message
						sensor_msgs::CompressedImagePtr image(new sensor_msgs::CompressedImage());
						image->header.frame_id = m_config.camera_frame;
						image->header.stamp = data_stamp;
						image->format = "jpeg";
						image->data.insert(image->data.end(), data_ptr, data_ptr + data_size);
						m_pub_jpeg.publish(image);

						// Signal that the data flow is okay
						flow = GST_FLOW_OK;
					}
					else if(type == PIT_YUV)
					{
						// Display info about the data that the ROS publisher is receiving
						static gint cur_caps_width = 0, cur_caps_height = 0;
						static std::string cur_caps_format;
						static gsize cur_data_size = 0U;
						if(cur_caps_width != caps_width || cur_caps_height != caps_height || cur_caps_format != caps_format || cur_data_size != data_size)
						{
							cur_caps_width = caps_width;
							cur_caps_height = caps_height;
							cur_caps_format = caps_format;
							cur_data_size = data_size;
							ROS_INFO("YUV publisher is receiving %dx%d %s images (%" G_GSIZE_FORMAT " bytes)", cur_caps_width, cur_caps_height, cur_caps_format.c_str(), cur_data_size);
						}

						// Ensure the data size is as expected
						gsize cur_row_size = cur_caps_width + (cur_caps_width >> 1);
						gsize exp_data_size = cur_caps_height * cur_row_size;
						if(cur_data_size != exp_data_size)
							ROS_ERROR("YUV publisher received unexpected number of image bytes: %" G_GSIZE_FORMAT " (received) vs %" G_GSIZE_FORMAT " (expected)", cur_data_size, exp_data_size);
						else
						{
							// Publish an image message
							sensor_msgs::ImagePtr image(new sensor_msgs::Image());
							image->header.frame_id = m_config.camera_frame;
							image->header.stamp = data_stamp;
							image->width = cur_caps_width;
							image->height = cur_caps_height;
							image->encoding = "yuv_" + cur_caps_format;
							image->is_bigendian = false;
							image->step = cur_row_size;
							image->data.insert(image->data.end(), data_ptr, data_ptr + exp_data_size);
							m_pub_yuv.publish(image);

							// Signal that the data flow is okay
							flow = GST_FLOW_OK;
						}
					}
					else if(type == PIT_RGB)
					{
						// Display info about the data that the ROS publisher is receiving
						static gint cur_caps_width = 0, cur_caps_height = 0;
						static std::string cur_caps_format;
						static gsize cur_data_size = 0U;
						if(cur_caps_width != caps_width || cur_caps_height != caps_height || cur_caps_format != caps_format || cur_data_size != data_size)
						{
							cur_caps_width = caps_width;
							cur_caps_height = caps_height;
							cur_caps_format = caps_format;
							cur_data_size = data_size;
							ROS_INFO("%s publisher is receiving %dx%d %s images (%" G_GSIZE_FORMAT " bytes)", name, cur_caps_width, cur_caps_height, cur_caps_format.c_str(), cur_data_size);
						}

						// Ensure the data size is as expected
						gsize cur_row_size = cur_caps_width * 3U;
						gsize exp_data_size = cur_caps_height * cur_row_size;
						if(cur_data_size != exp_data_size)
							ROS_ERROR("%s publisher received unexpected number of image bytes: %" G_GSIZE_FORMAT " (received) vs %" G_GSIZE_FORMAT " (expected)", name, cur_data_size, exp_data_size);
						else
						{
							// Publish an image message
							sensor_msgs::ImagePtr image(new sensor_msgs::Image());
							image->header.frame_id = m_config.camera_frame;
							image->header.stamp = data_stamp;
							image->width = cur_caps_width;
							image->height = cur_caps_height;
							image->encoding = sensor_msgs::image_encodings::RGB8;
							image->is_bigendian = false;
							image->step = cur_row_size;
							image->data.insert(image->data.end(), data_ptr, data_ptr + exp_data_size);
							m_pub_rgb.publish(image);

							// Signal that the data flow is okay
							flow = GST_FLOW_OK;
						}
					}
					else
					{
						// This should never occur
						ROS_ERROR("Unrecognised %s publisher type: %d ", name, (int) type);
					}

					// Unmap the sample buffer memory
					gst_memory_unmap(memory, &memory_info);
				}
			}
		}

		// Free the sample
		gst_sample_unref(sample);
	}

	// Return the data flow state
	return flow;
}

// Inspect stream data callback
void PepperCamera::inspect_callback(GstElement* fakesink, GstBuffer* buffer, GstPad* pad, PepperCamera* pc)
{
	// Summarise the data being captured by the fake sink
	GstCaps* caps = gst_pad_get_current_caps(pad);
	if(!caps)
		ROS_ERROR("Stream inspector failed to obtain the data caps");
	else
	{
		guint num_memories = gst_buffer_n_memory(buffer);
		guint num_caps_structs = gst_caps_get_size(caps);
		if(num_memories < 1U)
			ROS_ERROR("Stream inspector received a buffer with no memory");
		else if(num_caps_structs < 1U)
			ROS_ERROR("Stream inspector received a caps with no structures");
		else
		{
			if(num_memories != 1U)
				ROS_WARN_ONCE("Stream inspector is receiving buffers with multiple memories in it => Always just taking first one");
			if(num_caps_structs != 1U)
				ROS_WARN_ONCE("Stream inspector is receiving caps with multiple structures in it => Always just taking first one");
			GstMemory *memory = gst_buffer_peek_memory(buffer, 0U);
			GstStructure* caps_struct = gst_caps_get_structure(caps, 0U);
			gint caps_width = 0, caps_height = 0;
			const gchar* caps_type = NULL;
			if(!memory)
				ROS_ERROR("Stream inspector failed to get a pointer to the buffer memory");
			else if(!caps_struct)
				ROS_ERROR("Stream inspector failed to get the internal caps structure");
			else if(!(caps_type = gst_structure_get_name(caps_struct)))
				ROS_ERROR("Stream inspector failed to determine the data format from the caps");
			else
			{
				// Retrieve selected information about the stream
				gst_structure_get_int(caps_struct, "width", &caps_width);
				gst_structure_get_int(caps_struct, "height", &caps_height);
				gsize data_size = gst_memory_get_sizes(memory, NULL, NULL);

				// Display the stream information
				static std::string cur_caps_type;
				static gint cur_caps_width = 0, cur_caps_height = 0;
				if(cur_caps_type != caps_type || cur_caps_width != caps_width || cur_caps_height != caps_height)
				{
					cur_caps_type = caps_type;
					cur_caps_width = caps_width;
					cur_caps_height = caps_height;
					std::ostringstream os;
					os << "Receiving '" << cur_caps_type << "' data";
					if(caps_width >= 1 && caps_height >= 1)
						os << " of resolution " << caps_width << 'x' << caps_height;
					os << " (~" << 1000U * ((data_size + 500U) / 1000U) << " bytes), but no output is configured so nothing to do...";
					ROS_INFO_STREAM(os.str());
				}
			}
		}
		gst_caps_unref(caps);
	}
}

//
// GStreamer utilities
//

// Configure a queue in the standard way
void PepperCamera::configure_queue(GstElement* queue)
{
	// Do nothing if no queue was provided
	if(!queue)
		return;

	// Enable only a byte limit on the queue
	g_object_set(queue, "max-size-buffers", (guint) 0U, "max-size-bytes", (guint) (m_config.queue_size_mb * 1048576U), "max-size-time", (guint64) 0LU, NULL);

	// Handle queue overrun
	g_signal_connect(queue, "overrun", G_CALLBACK(PepperCamera::queue_overrun_callback), this);
}

// Link a tee element to a particular queue (if tee_pad is not NULL after calling then you need to release and unref it, whether the return value is TRUE or FALSE)
gboolean PepperCamera::link_tee_queue(GstElement* tee, GstElement* queue, GstPad*& tee_pad)
{
	// Create a src request pad on the tee element
	tee_pad = gst_element_get_request_pad(tee, "src_%u");
	if(!tee_pad)
		return FALSE;

	// Retrieve the sink pad on the queue element
	GstPad* queue_pad = gst_element_get_static_pad(queue, "sink");
	if(!queue_pad)
		return FALSE;

	// Link the two pads
	gboolean link_success = (gst_pad_link(tee_pad, queue_pad) == GST_PAD_LINK_OK ? TRUE : FALSE);
	gst_object_unref(queue_pad);
	return link_success;
}

// Add a GStreamer element to a bin while keeping a reference to it
gboolean PepperCamera::gst_bin_add_ref(GstBin *bin, GstElement *element)
{
	// Generate a new reference then let the bin take your original one
	gst_object_ref(element);
	return gst_bin_add(bin, element);
}

// Add many GStreamer elements to a bin while keeping references to them
void PepperCamera::gst_bin_add_many_ref(GstBin *bin, GstElement *element1, ...)
{
	// Adapt default implementation from gstutils.c
	va_list args;
	g_return_if_fail(GST_IS_BIN(bin));
	g_return_if_fail(GST_IS_ELEMENT(element1));
	va_start(args, element1);
	while(element1)
	{
		gst_bin_add_ref(bin, element1);
		element1 = va_arg(args, GstElement*);
	}
	va_end(args);
}

// Drop-in replacement for gst_clear_object() for GStreamer <1.16 (except that casting to GstObject** is required)
void PepperCamera::gst_object_unref_safe(GstObject** object_ptr)
{
	// Clear the pointer and unref the object
	g_clear_pointer(object_ptr, gst_object_unref);
}

// Post a callback into the main loop to cancel it
void PepperCamera::post_cancel_main_loop()
{
	// Post the cancel callback with high priority
	g_idle_add_full(G_PRIORITY_HIGH, PC_G_SOURCE_FUNC(PepperCamera::cancel_main_loop_callback), this, NULL);
}

// Cancel main loop callback
gboolean PepperCamera::cancel_main_loop_callback(PepperCamera* pc)
{
	// Cancel the main loop
	pc->cancel_main_loop();

	// Remove this callback source
	return G_SOURCE_REMOVE;
}

// Cancel the main loop
void PepperCamera::cancel_main_loop()
{
	// Cancel the main loop if it is running
	if(m_pipeline != NULL && m_main_loop != NULL && g_main_loop_is_running(m_main_loop) && !m_pipeline_stalled)
	{
		GstState current_state = GST_STATE_NULL, pending_state = GST_STATE_NULL;
		gst_element_get_state(m_pipeline, &current_state, &pending_state, 0U);
		if(current_state == GST_STATE_PLAYING || pending_state == GST_STATE_PLAYING)
		{
			ROS_INFO("Cancelling GStreamer main loop...");
			if(gst_element_send_event(m_pipeline, gst_event_new_eos()))
				return;
		}
	}

	// Fall back to quitting the main loop
	quit_main_loop();
}

// Quit main loop
void PepperCamera::quit_main_loop()
{
	// Quit the main loop if it exists
	if(m_main_loop != NULL)
	{
		ROS_INFO("Quitting GStreamer main loop...");
		g_main_loop_quit(m_main_loop);
	}
}

//
// Misc utilities
//

// Ensure a string ends with a particular extension (if it is not empty)
bool PepperCamera::ensure_extension(std::string& str, const std::string& ext)
{
	// Append extension if required and return whether the extension was added
	if(str.empty() || (str.length() >= ext.length() && std::equal(ext.begin(), ext.end(), str.end() - ext.length())))
		return false;
	else
	{
		str.append(ext);
		return true;
	}
}
// EOF
