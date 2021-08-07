#ifndef RASPI_ENCAMODE_CONFIGS_HPP__
#define RASPI_ENCAMODE_CONFIGS_HPP__

#include <string>
#include <unordered_map>

#include "raspicam_camcontrol.hpp"


// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Port configuration for the splitter component
#define SPLITTER_OUTPUT_PORT 0
#define SPLITTER_PREVIEW_PORT 1

// Video format information
// 0 implies variable
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

 // Max bitrate we allow for recording
const int MAX_BITRATE_MJPEG = 25000000; // 25Mbits/s
const int MAX_BITRATE_LEVEL4 = 25000000; // 25Mbits/s
const int MAX_BITRATE_LEVEL42 = 62500000; // 62.5Mbits/s

/// Interval at which we check for an failure abort during capture
const int ABORT_INTERVAL = 100; // ms

//============= RaspiCamControl.h
/// Annotate bitmask options
/// Supplied by user on command line
#define ANNOTATE_USER_TEXT          1
/// Supplied by app using this module
#define ANNOTATE_APP_TEXT           2
/// Insert current date
#define ANNOTATE_DATE_TEXT          4
// Insert current time
#define ANNOTATE_TIME_TEXT          8

#define ANNOTATE_SHUTTER_SETTINGS   16
#define ANNOTATE_CAF_SETTINGS       32
#define ANNOTATE_GAIN_SETTINGS      64
#define ANNOTATE_LENS_SETTINGS      128
#define ANNOTATE_MOTION_SETTINGS    256
#define ANNOTATE_FRAME_NUMBER       512
#define ANNOTATE_BLACK_BACKGROUND   1024

/// Capture/Pause switch method
/// Simply capture for time specified
enum class WaitMethod
{
   NONE,       /// Simply capture for time specified
   TIMED,      /// Cycle between capture and pause for times specified
   KEYPRESS,   /// Switch between capture and pause on keypress
   WM_SIGNAL,     /// Switch between capture and pause on signal
   FOREVER     /// Run/record forever
};

/** Possible raw output formats
 */
enum class RawOutputFmt
{
   ROF_YUV = 0,
   ROF_RGB,
   ROF_GRAY,
};


/** Structure containing all state information for the current run
 */
struct RaspiEncamodeConfig
{
   RaspiCamControl::CamConfig camera_parameters; /// Camera setup parameters
	
	// ========== Common settings ==========
	char camera_name[MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN]; // Name of the camera sensor
	int width;                      /// Requested width of image
	int height;                     /// requested height of image
	std::string filename;           /// filename of output file
	int cameraNum;                  /// Camera number
	int sensor_mode;                /// Sensor mode. 0=auto. Check docs/forum for modes selected by other values.
	int verbose;                    /// !0 if want detailed run information
	int gps;                        /// Add real-time gpsd output to output
	
	// ========== Encode settings ==========
	int timeout;                    /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
	MMAL_FOURCC_T encoding;         /// Requested codec video encoding (MJPEG or H264)
	int bitrate;                    /// Requested bitrate
	int framerate;                  /// Requested frame rate (fps)
	int intraperiod;                /// Intra-refresh period (key frame rate)
	int quantisationParameter;      /// Quantisation parameter - quality. Set bitrate 0 and set this for variable bitrate
	int bInlineHeaders;             /// Insert inline headers to stream (SPS, PPS)
	
	int immutableInput;             /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
	/// the camera output or the encoder output (with compression artifacts)
	MMAL_VIDEO_PROFILE_T profile;                    /// H264 profile to use for encoding
	MMAL_VIDEO_LEVEL_T level;                      /// H264 level to use for encoding
	WaitMethod waitMethod;          /// Method for switching between pause and capture

	int onTime;                     /// In timed cycle mode, the amount of time the capture is on per cycle
	int offTime;                    /// In timed cycle mode, the amount of time the capture is off per cycle

	int segmentSize;                /// Segment mode In timed cycle mode, the amount of time the capture is off per cycle
	int segmentWrap;                /// Point at which to wrap segment counter
	int segmentNumber;              /// Current segment counter
	int splitWait;                  /// Switch if user wants splited files
	
	// ========== Preview parameters ==========
	int wantPreview;                /// Display a preview
	int wantFullScreenPreview;      /// 0 is use previewRect, non-zero to use full screen
	int opacity;                    /// Opacity of window - 0 = transparent, 255 = opaque
	MMAL_RECT_T previewWindow;      /// Destination rectangle for the preview window.

	bool bStartRunning;
	int bCircularBuffer;            /// Whether we are writing to a circular buffer
	bool flush_buffers;

	int inlineMotionVectors;		/// Encoder outputs inline Motion Vectors
	std::string imv_filename;		/// filename of inline Motion Vectors output
	int raw_output;                 /// Output raw video from camera as well
	RawOutputFmt raw_output_fmt;  /// The raw video format
	std::string raw_filename;       /// Filename for raw video output
	MMAL_VIDEO_INTRA_REFRESH_T intra_refresh_type;         /// What intra refresh type to use. -1 to not set.
	
	std::string pts_filename;
	int save_pts;
	

	bool netListen;
	MMAL_BOOL_T addSPSTiming;
	int slices;
   
public: 
	void SetDefault()
	{
		this->camera_parameters.SetDefault();
		
		// ========== Common settings ==========
		strncpy(this->camera_name, "(Unknown)", MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN);
		// We dont set width and height since these will be specific to the app being built.
		this->width = 0;
		this->height = 0;
		this->filename.clear();
		this->verbose = 0;
		this->cameraNum = 0;
		this->sensor_mode = 0;
		this->gps = 0;
		
		// ========== Encode settings ==========
		this->timeout = -1; // replaced with 5000ms later if unset
		this->width = 1920;       // Default to 1080p
		this->height = 1080;
		this->encoding = MMAL_ENCODING_H264;
		this->bitrate = 17000000; // This is a decent default bitrate for 1080p
		this->framerate = VIDEO_FRAME_RATE_NUM;
		this->intraperiod = -1;    // Not set
		this->quantisationParameter = 0;

		this->immutableInput = 1;
		this->profile = MMAL_VIDEO_PROFILE_H264_HIGH;
		this->level = MMAL_VIDEO_LEVEL_H264_4;
		this->waitMethod = WaitMethod::NONE;
		this->onTime = 5000;
		this->offTime = 5000;
		this->bInlineHeaders = 0;
		this->segmentSize = 0;  // 0 = not segmenting the file.
		this->segmentNumber = 1;
		this->segmentWrap = 0; // Point at which to wrap segment number back to 1. 0 = no wrap
		
		this->splitWait = 0;
		this->bStartRunning = true;
		this->bCircularBuffer = 0;
		this->flush_buffers = false;
		this->inlineMotionVectors = 0;
		this->intra_refresh_type = (MMAL_VIDEO_INTRA_REFRESH_T)0xFFFFFFFF;
		this->save_pts = 0;
		this->netListen = false;
		this->addSPSTiming = MMAL_FALSE;
		this->slices = 1;
		
		// ========== Preview parameters ==========
		this->wantPreview = 1;
		this->wantFullScreenPreview = 1;
		this->opacity = 255;
		this->previewWindow.x = 0;
		this->previewWindow.y = 0;
		this->previewWindow.width = 1024;
		this->previewWindow.height = 768;
	};
};


/// Structure to cross reference H264 profile strings against the MMAL parameter equivalent
static std::unordered_map<std::string, MMAL_VIDEO_PROFILE_T> profile_map =
{
   {"baseline",     MMAL_VIDEO_PROFILE_H264_BASELINE},
   {"main",         MMAL_VIDEO_PROFILE_H264_MAIN},
   {"high",         MMAL_VIDEO_PROFILE_H264_HIGH},
//   {"constrained",  MMAL_VIDEO_PROFILE_H264_CONSTRAINED_BASELINE} // Does anyone need this?
};

/// Structure to cross reference H264 level strings against the MMAL parameter equivalent
static std::unordered_map<std::string,MMAL_VIDEO_LEVEL_T> level_map =
{
   {"4",           MMAL_VIDEO_LEVEL_H264_4},
   {"4.1",         MMAL_VIDEO_LEVEL_H264_41},
   {"4.2",         MMAL_VIDEO_LEVEL_H264_42},
};


static std::unordered_map<std::string, bool> initial_map =
{
   {"record",     true},
   {"pause",      false},
};


static std::unordered_map<std::string, MMAL_VIDEO_INTRA_REFRESH_T>  intra_refresh_map =
{
   
   {"cyclic",       MMAL_VIDEO_INTRA_REFRESH_CYCLIC},
   {"adaptive",     MMAL_VIDEO_INTRA_REFRESH_ADAPTIVE},
   {"both",         MMAL_VIDEO_INTRA_REFRESH_BOTH},
   {"cyclicrows",   MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS},
//   {"random",       MMAL_VIDEO_INTRA_REFRESH_PSEUDO_RAND} Cannot use random, crashes the encoder. No idea why.
};


static std::unordered_map<std::string, RawOutputFmt> raw_output_fmt_map =
{
   {"yuv",  RawOutputFmt::ROF_YUV},
   {"rgb",  RawOutputFmt::ROF_RGB},
   {"gray", RawOutputFmt::ROF_GRAY},
};



static std::unordered_map<std::string, MMAL_STEREOSCOPIC_MODE_T> stereo_mode_map =
{
   {"off",           MMAL_STEREOSCOPIC_MODE_NONE},
   {"sbs",           MMAL_STEREOSCOPIC_MODE_SIDE_BY_SIDE},
   {"tb",            MMAL_STEREOSCOPIC_MODE_TOP_BOTTOM},
};



#endif
