// Pepper camera class
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Includes
#include <pepper_camera/pepper_camera.h>
#include <glib-unix.h>
#include <algorithm>
#include <thread>
#include <chrono>

// Namespaces
using namespace pepper_camera;

// Constructor
PepperCamera::PepperCamera(const ros::NodeHandle& nh_interface, const ros::NodeHandle& nh_param) :
	m_nh_interface(nh_interface),
	m_nh_param(nh_param)
{
	// Reset the configuration variables
	reset_config();
}

// Run the Pepper camera loop
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
		if(m_auto_retry)
		{
			ROS_INFO("Retrying Pepper camera stream...");
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
		else
			break;
	}
	ROS_INFO("Pepper camera stream has exited");
}

// Reset the configuration variables
void PepperCamera::reset_config()
{
	// Reset all configuration variables to their default values
	m_port = 3016;                                   // Port to listen for UDP packets
	m_auto_retry = false;                            // Whether to auto-retry the camera pipeline if it exits
	m_publish_jpeg = false;                          // Whether to publish JPEG images
	m_publish_yuv = false;                           // Whether to publish YUV images
	m_publish_rgb = true;                            // Whether to publish RGB images
	m_record_jpegs.clear();                          // Format: path/to/filename%05d.jpg
	m_record_jpegs_max = 1000;                       // Maximum number of JPEGs to save before the oldest are deleted again (0 = Unlimited)
	m_record_mjpeg.clear();                          // Format: path/to/filename.mkv
	m_record_h264.clear();                           // Format: path/to/filename.mkv
	m_record_h264_bitrate = 1500;                    // Units: kbit/sec
	m_record_h264_speed = 5;                         // Allowed values: gst-inspect-1.0 x264enc | grep 'speed-preset' -A 15
	m_record_h264_profile = "constrained-baseline";  // Allowed values: gst-inspect-1.0 x264enc | grep 'profile:'
	m_preview = false;                               // Whether to show a preview window
}

// Configure the Pepper camera loop
bool PepperCamera::configure()
{
	// Reset the current configuration to default values
	reset_config();

	// TODO: Set configuration variables based on ROS params and the like
	// TODO: Integrate into whatever code comes into this function
	ensure_extension(m_record_jpegs, ".jpg");
	ensure_extension(m_record_mjpeg, ".mkv");
	ensure_extension(m_record_h264, ".mkv");

	// Return success
	return true;
}

// Initialise the Pepper camera stream
bool PepperCamera::init_stream()
{
	// Print that the stream is being initialised
	ROS_INFO("Initialising Pepper camera stream...");

	// Initialise the GStreamer library
	if(!gst_is_initialized())
	{
		ROS_INFO("Initialising GStreamer library...");
		gst_init(0, NULL);
		ROS_INFO("Loaded %s", gst_version_string());
	}

	// Flags whether certain parts of the pipeline are required
	bool record_jpegs = !m_record_jpegs.empty();
	bool record_mjpeg = !m_record_mjpeg.empty();
	bool record_h264 = !m_record_h264.empty();
	int tee_yuv_count = m_publish_yuv + m_publish_rgb + record_h264 + m_preview;
	bool have_tee_yuv = (tee_yuv_count >= 2);
	bool have_jpegdec = (tee_yuv_count >= 1);
	int tee_jpeg_count = m_publish_jpeg + record_jpegs + record_h264 + have_jpegdec;
	bool have_tee_jpeg = (tee_jpeg_count >= 2);

	// Check that at least one sink is configured
	if(tee_jpeg_count < 1)
	{
		ROS_WARN("No sinks have been configured for the streaming pipeline => Nothing to do");
		return false;
	}

	// Create the required pipeline elements
	bool any_elem_invalid = false;
	m_elem = new GSElements();
	any_elem_invalid |= !(m_elem->udpsrc = gst_element_factory_make("udpsrc", "udpsrc"));
	any_elem_invalid |= !(m_elem->rtpjpegdepay = gst_element_factory_make("rtpjpegdepay", "rtpjpegdepay"));
	if(have_tee_jpeg)
	{
		any_elem_invalid |= !(m_elem->tee_jpeg = gst_element_factory_make("tee", "tee_jpeg"));
		if(m_publish_jpeg)
			any_elem_invalid |= !(m_elem->publish_jpeg_queue = gst_element_factory_make("queue", "publish_jpeg_queue"));
		if(record_jpegs)
			any_elem_invalid |= !(m_elem->record_jpegs_queue = gst_element_factory_make("queue", "record_jpegs_queue"));
		if(record_mjpeg)
			any_elem_invalid |= !(m_elem->record_mjpeg_queue = gst_element_factory_make("queue", "record_mjpeg_queue"));
		if(have_jpegdec)
			any_elem_invalid |= !(m_elem->jpegdec_queue = gst_element_factory_make("queue", "jpegdec_queue"));
	}
	if(m_publish_jpeg)
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
		if(m_publish_yuv)
			any_elem_invalid |= !(m_elem->publish_yuv_queue = gst_element_factory_make("queue", "publish_yuv_queue"));
		if(m_publish_rgb)
			any_elem_invalid |= !(m_elem->publish_rgb_queue = gst_element_factory_make("queue", "publish_rgb_queue"));
		if(record_h264)
			any_elem_invalid |= !(m_elem->record_h264_queue = gst_element_factory_make("queue", "record_h264_queue"));
		if(m_preview)
			any_elem_invalid |= !(m_elem->preview_queue = gst_element_factory_make("queue", "preview_queue"));
	}
	if(m_publish_yuv)
		any_elem_invalid |= !(m_elem->publish_yuv = gst_element_factory_make("appsink", "publish_yuv"));
	if(m_publish_rgb)
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
	if(m_preview)
		any_elem_invalid |= !(m_elem->preview = gst_element_factory_make("fpsdisplaysink", "preview"));

	// Abort if not all pipeline elements were created successfully
	if(any_elem_invalid)
	{
		ROS_ERROR("Failed to create some of the required pipeline elements");
		return false;
	}

	// Create the pipeline
	m_pipeline = gst_pipeline_new("pipeline");
	GstBin* pipeline_bin = GST_BIN(m_pipeline);

	// Configure/add the UDP source
	ROS_INFO("Will listen to UDP on port %d", m_port);
	GstCaps* udpsrc_caps = gst_caps_new_simple("application/x-rtp", "encoding-name", G_TYPE_STRING, "JPEG", NULL);
	g_object_set(m_elem->udpsrc, "port", m_port, "caps", udpsrc_caps, NULL);
	gst_caps_unref(udpsrc_caps);
	gst_bin_add_many_ref(pipeline_bin, m_elem->udpsrc, m_elem->rtpjpegdepay, NULL);

	// Configure/add the JPEG tee
	if(m_elem->tee_jpeg)
		gst_bin_add_ref(pipeline_bin, m_elem->tee_jpeg);

	// Configure/add the JPEG publisher app sink
	if(m_elem->publish_jpeg)
	{
		GstCaps* publish_jpeg_caps = gst_caps_new_empty_simple("image/jpeg");
		g_object_set(m_elem->publish_jpeg, "caps", publish_jpeg_caps, "emit-signals", TRUE, "sync", FALSE, NULL);
		gst_caps_unref(publish_jpeg_caps);
		g_signal_connect(m_elem->publish_jpeg, "new-sample", G_CALLBACK(PepperCamera::publish_jpeg_callback), this);
		gst_bin_add_many_ref(pipeline_bin, m_elem->publish_jpeg, m_elem->publish_jpeg_queue, NULL);
	}

	// Configure/add the JPEGs recorder
	if(m_elem->record_jpegs)
	{
		g_object_set(m_elem->record_jpegs, "location", m_record_jpegs.c_str(), "max-files", (unsigned int) std::max(m_record_jpegs_max, 0), "sync", FALSE, NULL);
		gst_bin_add_many_ref(pipeline_bin, m_elem->record_jpegs, m_elem->record_jpegs_queue, NULL);
	}

	// Configure/add the MJPEG recorder
	if(m_elem->record_mjpeg)
	{
		g_object_set(m_elem->record_mjpeg, "location", m_record_mjpeg.c_str(), "sync", FALSE, NULL);
		gst_bin_add_many_ref(pipeline_bin, m_elem->record_mjpeg_mux, m_elem->record_mjpeg, m_elem->record_mjpeg_queue, NULL);
	}

	// Configure/add the JPEG decoder
	if(m_elem->jpegdec)
		gst_bin_add_many_ref(pipeline_bin, m_elem->jpegdec, m_elem->jpegdec_queue, NULL);

	// Configure/add the YUV tee
	if(m_elem->tee_yuv)
		gst_bin_add_ref(pipeline_bin, m_elem->tee_yuv);

	// Configure/add the YUV publisher app sink
	if(m_elem->publish_yuv)
	{
		GstCaps* publish_yuv_caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "I420", NULL);
		g_object_set(m_elem->publish_yuv, "caps", publish_yuv_caps, "emit-signals", TRUE, "sync", FALSE, NULL);
		gst_caps_unref(publish_yuv_caps);
		g_signal_connect(m_elem->publish_yuv, "new-sample", G_CALLBACK(PepperCamera::publish_yuv_callback), this);
		gst_bin_add_many_ref(pipeline_bin, m_elem->publish_yuv, m_elem->publish_yuv_queue, NULL);
	}

	// Configure/add the RGB publisher app sink
	if(m_elem->publish_rgb)
	{
		GstCaps* publish_rgb_caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "RGB", NULL);
		g_object_set(m_elem->publish_rgb, "caps", publish_rgb_caps, "emit-signals", TRUE, "sync", FALSE, NULL);
		gst_caps_unref(publish_rgb_caps);
		g_signal_connect(m_elem->publish_rgb, "new-sample", G_CALLBACK(PepperCamera::publish_rgb_callback), this);
		gst_bin_add_many_ref(pipeline_bin, m_elem->publish_rgb_convert, m_elem->publish_rgb, m_elem->publish_rgb_queue, NULL);
	}

	// Configure/add the H264 recorder
	if(m_elem->record_h264)
	{
		g_object_set(m_elem->record_h264_enc, "pass", 0, "bitrate", (unsigned int) std::max(m_record_h264_bitrate, 50), "speed-preset", m_record_h264_speed, NULL);  // Pass = 0 = Constant bitrate encoding (CBR)
		g_object_set(m_elem->record_h264, "location", m_record_h264.c_str(), "sync", FALSE, NULL);
		gst_bin_add_many_ref(pipeline_bin, m_elem->record_h264_enc, m_elem->record_h264_mux, m_elem->record_h264, m_elem->record_h264_queue, NULL);
	}

	// Configure/add the preview window sink
	if(m_elem->preview)
	{
		g_object_set(m_elem->preview, "text-overlay", true, "sync", TRUE, NULL);
		gst_bin_add_many_ref(pipeline_bin, m_elem->preview, m_elem->preview_queue, NULL);
	}







	// TODO: COMMENT
	int link_success = TRUE;
	link_success &= gst_element_link(m_elem->udpsrc, m_elem->rtpjpegdepay);

// 		GstCaps* record_h264_caps = gst_caps_new_simple("video/x-h264", "stream-format", G_TYPE_STRING, "avc", "profile", G_TYPE_STRING, m_record_h264_profile.c_str(), NULL);
// 		g_object_set(m_elem->record_h264_caps, "caps", record_h264_caps, NULL); // TODO: Use gst_element_link_filtered instead
// 		gst_caps_unref(record_h264_caps);







	// Abort if not all pipeline elements were linked successfully
	if(link_success != TRUE)
	{
		ROS_ERROR("Failed to link some of the required pipeline elements");
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
	g_unix_signal_add(SIGINT, G_SOURCE_FUNC(PepperCamera::quit_main_loop_callback), m_main_loop);

	// Enable and listen to error messages on the bus
	GstBus* bus = gst_element_get_bus(m_pipeline);
	gst_bus_add_signal_watch(bus);
	g_signal_connect(G_OBJECT(bus), "message::error", G_CALLBACK(PepperCamera::stream_error_callback), m_main_loop);
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
	// Print that the stream is being cleaned up
	ROS_INFO("Cleaning up Pepper camera stream...");

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
		m_elem->clear();
		delete m_elem;
		m_elem = NULL;
	}
}

// Clear GStreamer pipeline elements
void pepper_camera::PepperCamera::GSElements::clear()
{
	// Unref all elements if they exist and reset the pointers to NULL
	gst_clear_object(&udpsrc);
	gst_clear_object(&rtpjpegdepay);
	gst_clear_object(&tee_jpeg);
	gst_clear_object(&publish_jpeg_queue);
	gst_clear_object(&publish_jpeg);
	gst_clear_object(&record_jpegs_queue);
	gst_clear_object(&record_jpegs);
	gst_clear_object(&record_mjpeg_queue);
	gst_clear_object(&record_mjpeg_mux);
	gst_clear_object(&record_mjpeg);
	gst_clear_object(&jpegdec_queue);
	gst_clear_object(&jpegdec);
	gst_clear_object(&tee_yuv);
	gst_clear_object(&publish_yuv_queue);
	gst_clear_object(&publish_yuv);
	gst_clear_object(&publish_rgb_queue);
	gst_clear_object(&publish_rgb_convert);
	gst_clear_object(&publish_rgb);
	gst_clear_object(&record_h264_queue);
	gst_clear_object(&record_h264_enc);
	gst_clear_object(&record_h264_mux);
	gst_clear_object(&record_h264);
	gst_clear_object(&preview_queue);
	gst_clear_object(&preview);
}

// Quit main loop callback
bool PepperCamera::quit_main_loop_callback(GMainLoop* main_loop)
{
	// Quit the main loop
	ROS_INFO("Quitting main loop...");
	g_main_loop_quit(main_loop);

	// Keep the source of this callback
	return TRUE;
}

// GStreamer error callback
void PepperCamera::stream_error_callback(GstBus* bus, GstMessage* msg, GMainLoop* main_loop)
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

// New JPEG image sample callback
GstFlowReturn PepperCamera::publish_jpeg_callback(GstElement* appsink, PepperCamera* pc)
{
	return GST_FLOW_OK;  // TODO: TEMP
}

// New YUV image sample callback
GstFlowReturn PepperCamera::publish_yuv_callback(GstElement* appsink, PepperCamera* pc)
{
	return GST_FLOW_OK;  // TODO: TEMP
}

// New RGB image sample callback
GstFlowReturn PepperCamera::publish_rgb_callback(GstElement* appsink, PepperCamera* pc)
{
	// Retrieve the sample
	GstSample* sample;
	g_signal_emit_by_name(appsink, "pull-sample", &sample);
	if(!sample)
		return GST_FLOW_ERROR;

	// TODO
	GstBuffer* buffer = gst_sample_get_buffer(sample);
	if(buffer == NULL)
		return GST_FLOW_ERROR;

	// TODO: gst_sample_get_info() What is in there?

	GstMemory *memory = gst_buffer_get_memory(buffer, 0);
	GstMapInfo info;
	gst_memory_map(memory, &info, GST_MAP_READ);
	gsize &buf_size = info.size;
	guint8* &buf_data = info.data;
	GstClockTime bt = gst_element_get_base_time(pc->m_pipeline);  // TODO: What time does this give? Any relevance?
	ROS_INFO("Got %u bytes at address %p: PTS %.9f", (unsigned int) buf_size, (void *) buf_data, buffer->pts * 1e-9);

	// Free the sample
	gst_sample_unref(sample);

	// Return that the data flow is okay
	return GST_FLOW_OK;
}

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

// Add a GStreamer element to a bin while keeping a reference to it
gboolean PepperCamera::gst_bin_add_ref(GstBin *bin, GstElement *element)
{
	// Generate a new reference then let the bin take your original one
	g_object_ref(element);
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
// EOF
