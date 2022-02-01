#ifndef RASPI_ENCAMODE_HPP__
#define RASPI_ENCAMODE_HPP__

#include <cstdint>
#include "mmal.h"
#include "mmal_parameters_video.h"
#include "util/mmal_connection.h"
#include "raspi_encamode_configs.hpp" 

class RaspiEncamode;   // forward declaration

/** Struct used to pass information in encoder port userdata to callback
 */
// TODO use c++ file
struct PORT_USERDATA
{
   FILE *file_handle;                   /// File handle to write buffer data to.
   RaspiEncamode * pact_obj;			/// Pointer to our current acting object in callback
   int abort;                           /// Set to 1 in callback if an error occurs to attempt to abort the capture
   char *cb_buff;                       /// Circular buffer
   int   cb_len;                        /// Length of buffer
   int   cb_wptr;                       /// Current write pointer
   int   cb_wrap;                       /// Has buffer wrapped at least once?
   int   cb_data;                       /// Valid bytes in buffer
#define IFRAME_BUFSIZE (60*1000)
   int   iframe_buff[IFRAME_BUFSIZE];   /// buffer of iframe pointers
   int   iframe_buff_wpos;
   int   iframe_buff_rpos;
   char  header_bytes[29];
   int  header_wptr;
   FILE *imv_file_handle;               /// File handle to write inline motion vectors to.
   FILE *raw_file_handle;               /// File handle to write raw data to.
   int  flush_buffers;
   FILE *pts_file_handle;               /// File timestamps
   FILE *raw_pts_file_handle;               /// File timestamps
};


class RaspiEncamode 
{
public:
	bool Init(const RaspiEncamodeConfig & conf);
	
	bool Run();
	
 	// Callbacks for MMAL, not for user use
	void EncoderBufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
	void SplitterBufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

private:
	RaspiEncamodeConfig _conf;
	
	// MMAL components
	MMAL_COMPONENT_T *_p_camera_component = NULL;    /// Pointer to the camera component
	MMAL_COMPONENT_T *_p_preview_component = NULL;   /// Pointer to the created preview display component
	MMAL_COMPONENT_T * _p_encoder_component = NULL;   /// Pointer to the encoder component
	MMAL_COMPONENT_T *_p_splitter_component = NULL;  /// Pointer to the splitter component
	
	MMAL_CONNECTION_T *_p_preview_connection; /// Pointer to the connection from camera or splitter to preview
	MMAL_CONNECTION_T *_p_splitter_connection; /// Pointer to the connection from camera to splitter
	MMAL_CONNECTION_T *_p_encoder_connection; /// Pointer to the connection from camera to encoder
	
	MMAL_POOL_T *_p_encoder_pool = NULL; /// Pointer to the pool of buffers used by encoder output port
	MMAL_POOL_T *_p_splitter_pool = NULL; /// Pointer to the pool of buffers used by splitter output port 0
	
	MMAL_PORT_T *_p_camera_preview_port = NULL;
	MMAL_PORT_T *_p_camera_video_port = NULL;
	MMAL_PORT_T *_p_camera_still_port = NULL;
	MMAL_PORT_T *_p_preview_input_port = NULL;
	MMAL_PORT_T *_p_encoder_input_port = NULL;
	MMAL_PORT_T *_p_encoder_output_port = NULL;
	MMAL_PORT_T *_p_splitter_input_port = NULL;
	MMAL_PORT_T *_p_splitter_output_port = NULL;
	MMAL_PORT_T *_p_splitter_preview_port = NULL;

	PORT_USERDATA _callback_data;        /// Used to move data to the encoder callback
	
	int _spl_frame_cntr;			// Splitter frame counter
	int64_t _spl_frame_steady_us;	// Latest splitter callback steady clock us
	
	int _enc_frame_cntr;	// Frame counter, only valid with PTS logging
	int64_t _enc_frame_steady_us;  // Latest encoder callback steady clock us
	int64_t _startpts;	// Starting GPU (encoder) timestamp
	int64_t _lastpts;	// Latest GPU (encoder) timestamp
	
	int _seg_num;		// current segment number 
	
	// State control
	bool _split_now;	// Split at next possible i-frame if true
	bool _is_capturing;                 /// State of capture/pause
	
	static void GetCameraDefaults(int camera_num, char *camera_name, 
								  int *width, int *height );
	static void CheckCameraModel(int cam_num);
	static void DumpConfig( const RaspiEncamodeConfig & state);
	
	// Initialize
	void BuildRevMap();
	bool InitComponents();
	bool ConnectComponents();
	bool ConfigureCallback();
	bool EnablePort();
	
	// Start
	int WaitForNextChange();
	
	void SelfDestruct(const MMAL_STATUS_T & status );
	
	
	// Components
	
	// Note conf.framerate might be changed to 0, TODO
	static MMAL_STATUS_T CreateCameraComponent(
				RaspiEncamodeConfig & conf, 
				MMAL_COMPONENT_T ** camera_component);
	static void DestroyCameraComponent(MMAL_COMPONENT_T ** camera_component);
	
	static MMAL_STATUS_T CreatePreview(
				const RaspiEncamodeConfig & conf, 
				MMAL_COMPONENT_T ** preview_component);
	static void DestroyPreview(
				MMAL_COMPONENT_T ** preview_component);
	
	static MMAL_STATUS_T CreateEncoderComponent(
				RaspiEncamodeConfig & conf, 
				MMAL_COMPONENT_T ** encoder_component, 
				MMAL_POOL_T ** encoder_pool,
				int * bitrate		);
	static void DestroyEncoderComponent(
				MMAL_COMPONENT_T ** encoder_component, 
				MMAL_POOL_T ** encoder_pool);
	
	static MMAL_STATUS_T CreateSplitterComponent(
				const RaspiEncamodeConfig &conf,
				const MMAL_COMPONENT_T * camera_component,
				MMAL_COMPONENT_T ** splitter_component,
				MMAL_POOL_T ** splitter_pool);
	static void DestroySplitterComponent(
				MMAL_COMPONENT_T ** splitter_component, 
				MMAL_POOL_T **splitter_pool );
	
	
	// Helpers
	MMAL_STATUS_T ConnectPorts(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection);
	void MmalstatusToMsg(const MMAL_STATUS_T & status);
	void DisablePort(MMAL_PORT_T **port);
	
	
	// Time
	static void WriteTimestampCsv(FILE* p, const uint64_t & pts,
									const int & cntr,
									const uint64_t & clk);
	static int64_t GetSteadyUs64();
	static int64_t GetSteadyMs64();
	static int64_t GetUtcUs();
	static int64_t GetUsSinceUtcToday();
	static int64_t GetMsSinceUtcToday();
};

void EncoderBufferCallbackWrapper(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
void SplitterBufferCallbackWrapper(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);



#endif
