/*
 * TODO Modifies this
Copyright (c) 2018, Raspberry Pi (Trading) Ltd.
Copyright (c) 2013, Broadcom Europe Ltd.
Copyright (c) 2013, James Hughes
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \file RaspiVid.c
 * Command line program to capture a camera video stream and encode it to file.
 * Also optionally display a preview/viewfinder of current camera input.
 *
 * Description
 *
 * 3 components are created; camera, preview and video encoder.
 * Camera component has three ports, preview, video and stills.
 * This program connects preview and video to the preview and video
 * encoder. Using mmal we don't need to worry about buffers between these
 * components, but we do need to handle buffers from the encoder, which
 * are simply written straight to the file in the requisite buffer callback.
 *
 * If raw option is selected, a video splitter component is connected between
 * camera and preview. This allows us to set up callback for raw camera data
 * (in YUV420 or RGB format) which might be useful for further image processing.
 *
 * We use the RaspiCamControl code to handle the specific camera settings.
 * We use the RaspiPreview code to handle the (generic) preview window
 */

// We use some GNU extensions (basename)
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <thread>

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <ctype.h>
//#include <memory.h>
//#include <sysexits.h>

//#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
//#include <time.h>

extern "C" {
//TODO	_VC_TVSERVICE_DEFS_H_
//#include "interface/vmcs_host/vc_tvservice_defs.h"
#include "bcm_host.h"
#include "interface/vcos/vcos.h"
}

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h" // support for basic file logging
#include "spdlog/sinks/stdout_color_sinks.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

//#include "raspicam_camcontrol.hpp"
//#include "RaspiCommonSettings.h"
//#include "RaspiCamControl.h"
//#include "RaspiPreview.h"
//#include "RaspiCLI.h"
//#include "RaspiHelpers.h"
//#include "RaspiGPS.h"

//#include <semaphore.h>

//#include <stdbool.h>

#include "raspi_encamode_configs.hpp"
#include "raspi_encamode.hpp"

// TODO EWING is this correct?
static std::shared_ptr<spdlog::logger> p_err_logger;

static int64_t GPUCPUMONOTONICOFFSET = 0;
static int timeStamp_GlobalCounter=0;

/// Layer that preview window should be displayed on
#define PREVIEW_LAYER      2

//TBD: EWING
// [] Replace vcos log and fprintf log
// [] vcos_assert
// [] Linux file 
// [O] vcos_sleep


std::unordered_map<WaitMethod, std::string> wait_method_description
{
   {WaitMethod::NONE,      "Simple capture",         },
   {WaitMethod::FOREVER,   "Capture forever",        },
   {WaitMethod::TIMED,     "Cycle on time",          },
   {WaitMethod::KEYPRESS,  "Cycle on keypress",      },
   {WaitMethod::WM_SIGNAL, "Cycle on signal",        }
};

// TODO static int wait_method_description_size = sizeof(wait_method_description) / sizeof(wait_method_description[0]);

bool RaspiEncamode::Init(const RaspiEncamodeConfig & conf)
{
	BuildRevMap();  // initialize member reverse maps
	RaspiCamControl::BuildRevMap();
	bcm_host_init();
	_conf = conf;
	_frame = 0;
	_startpts = 0;
	_lastpts = 0;
	
	_split_now = false;
	_is_capturing = false;
	
	// Register our application with the logging system
	//vcos_log_register("RaspiVid", VCOS_LOG_CATEGORY);
	p_err_logger = spdlog::stdout_color_mt("RPI_ENCAM");
	//TODO p_err_logger->sinks().push_back(spdlog::stdout_color_st("RPI_ENCAM_ERR"));
	
	
	// Setup for sensor specific parameters, only set W/H settings if zero on entry
	GetCameraDefaults(_conf.cameraNum, 
					  _conf.camera_name,
					  &_conf.width,
					  &_conf.height);
	
	if (_conf.verbose) {
		//ORIG print_app_details(stderr);
		DumpConfig(_conf);
	}
	
	CheckCameraModel(_conf.cameraNum);
	
	//   if (_conf.gps)
	//      if (raspi_gps_setup(_conf.verbose))
	//         _conf.gps = 0;

	// OK, we have a nice set of parameters. Now set up our components
	// We have three components. Camera, Preview and encoder.
	if( !InitComponents() )
	{
		p_err_logger->error("Component init error!");
		return false;
	}
	
	if( !ConnectComponents() )
	{
		p_err_logger->error("Component connect error!");
		return false;
	}
	
	if( !ConfigureCallback() )
	{
		p_err_logger->error("Callback configure error!");
		return false;
	}
	
	// TODO: maybe move the following to an independent function?
	if( !EnablePort() )
	{
		p_err_logger->error("Enable port error!");
		return false;
	}
	
	return true;
}
	
static std::unordered_map<decltype(raw_output_fmt_map)::mapped_type, decltype(raw_output_fmt_map)::key_type> raw_output_fmt_map_rev;
static std::unordered_map<decltype(initial_map)::mapped_type, decltype(initial_map)::key_type> initial_map_rev;
static std::unordered_map<decltype(profile_map)::mapped_type, decltype(profile_map)::key_type> profile_map_rev;
static std::unordered_map<decltype(level_map)::mapped_type, decltype(level_map)::key_type> level_map_rev;
static std::unordered_map<decltype(intra_refresh_map)::mapped_type, decltype(intra_refresh_map)::key_type> intra_refresh_map_rev;

	
void RaspiEncamode::BuildRevMap()
{
	for(auto it = raw_output_fmt_map.begin(); it != raw_output_fmt_map.end(); it++)
	{
		raw_output_fmt_map_rev.insert(
			std::make_pair(it->second, it->first)  	);
	}
	
	for(auto it = initial_map.begin(); it != initial_map.end(); it++)
	{
		initial_map_rev.insert(
			std::make_pair(it->second, it->first)  	);;
	}
	
	for(auto it = profile_map.begin(); it != profile_map.end(); it++)
	{
		profile_map_rev.insert(
			std::make_pair(it->second, it->first)  	);;
	}
	for(auto it = level_map.begin(); it != level_map.end(); it++)
	{
		level_map_rev.insert(
			std::make_pair(it->second, it->first)  	);;
	}
	for(auto it = intra_refresh_map.begin(); it != intra_refresh_map.end(); it++)
	{
		intra_refresh_map_rev.insert(
			std::make_pair(it->second, it->first)  	);;
	}
	
}

void RaspiEncamode::GetCameraDefaults(int camera_num, char *camera_name, 
					   int *width, int *height )
{
   MMAL_COMPONENT_T *camera_info;
   MMAL_STATUS_T status;

   // Default to the OV5647 setup
   strncpy(camera_name, "OV5647", MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN);

   // Try to get the camera name and maximum supported resolution
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA_INFO, &camera_info);
   if (status == MMAL_SUCCESS)
   {
      MMAL_PARAMETER_CAMERA_INFO_T param;
      param.hdr.id = MMAL_PARAMETER_CAMERA_INFO;
      param.hdr.size = sizeof(param)-4;  // Deliberately undersize to check firmware version
      status = mmal_port_parameter_get(camera_info->control, &param.hdr);

      if (status != MMAL_SUCCESS)
      {
         // Running on newer firmware
         param.hdr.size = sizeof(param);
         status = mmal_port_parameter_get(camera_info->control, &param.hdr);
         if (status == MMAL_SUCCESS && (int) param.num_cameras > camera_num)
         {
            // Take the parameters from the first camera listed.
            if (*width == 0)
               *width = param.cameras[camera_num].max_width;
            if (*height == 0)
               *height = param.cameras[camera_num].max_height;
            strncpy(camera_name, param.cameras[camera_num].camera_name, MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN);
            camera_name[MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN-1] = 0;
         }
         else
            p_err_logger->error("Cannot read camera info, keeping the defaults for OV5647");
      }
      else
      {
         // Older firmware
         // Nothing to do here, keep the defaults for OV5647
      }

      mmal_component_destroy(camera_info);
   }
   else
   {
      p_err_logger->error("Failed to create camera_info component");
   }

   // default to OV5647 if nothing detected..
   if (*width == 0)
      *width = 2592;
   if (*height == 0)
      *height = 1944;
}


void RaspiEncamode::CheckCameraModel(int cam_num)
{
	MMAL_COMPONENT_T *camera_info;
	MMAL_STATUS_T status;

	// Try to get the camera name
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA_INFO, &camera_info);
	if (status == MMAL_SUCCESS)
	{
		MMAL_PARAMETER_CAMERA_INFO_T param;
		param.hdr.id = MMAL_PARAMETER_CAMERA_INFO;
		param.hdr.size = sizeof(param)-4;  // Deliberately undersize to check firmware version
		status = mmal_port_parameter_get(camera_info->control, &param.hdr);

		if (status != MMAL_SUCCESS)
		{
			// Running on newer firmware
			param.hdr.size = sizeof(param);
			status = mmal_port_parameter_get(camera_info->control, &param.hdr);
			if (status == MMAL_SUCCESS && (int)param.num_cameras > cam_num)
			{
				if (!strncmp(param.cameras[cam_num].camera_name, "toshh2c", 7))
				{
				p_err_logger->error( "The driver for the TC358743 HDMI to CSI2 chip you are using is NOT supported.");
				p_err_logger->error( "They were written for a demo purposes only, and are in the firmware on an as-is");
				p_err_logger->error( "basis and therefore requests for support or changes will not be acted on.\n");
				}
			}
		}

		mmal_component_destroy(camera_info);
	}
}

/**
 * Dump image state parameters to stderr.
 *
 * @param state Pointer to state structure to assign defaults to
 */
void RaspiEncamode::DumpConfig(const RaspiEncamodeConfig & conf)
{
	//ORIG raspicommonsettings_dump_parameters(&conf.common_settings);
	{
		p_err_logger->error( "Camera Name %s", conf.camera_name);
		p_err_logger->error( "Width {}, Height {}, filename {}", 
							 conf.width, conf.height, conf.filename);
		p_err_logger->error( "Using camera {}, sensor mode {}\n", 
							 conf.cameraNum, conf.sensor_mode);
		p_err_logger->error( "GPS output {}\n", conf.gps ? "Enabled" : "Disabled");
	}

	p_err_logger->error( "bitrate {}, framerate {}, time delay {}", 
						 conf.bitrate, conf.framerate, conf.timeout);
	p_err_logger->error( "H264 Profile " + profile_map_rev.at(conf.profile));
	p_err_logger->error( "H264 Level " + level_map_rev.at(conf.level));
	p_err_logger->error( "H264 Quantisation level {}, Inline headers {}", conf.quantisationParameter, conf.bInlineHeaders ? "Yes" : "No");
	p_err_logger->error( "H264 Fill SPS Timings {}", conf.addSPSTiming ? "Yes" : "No");
	p_err_logger->error( "H264 Intra refresh type {}, period {}",
						 conf.intra_refresh_type != (MMAL_VIDEO_INTRA_REFRESH_T)0xFFFFFFFF?intra_refresh_map_rev.at(conf.intra_refresh_type): "unknown", conf.intraperiod);
	p_err_logger->error( "H264 Slices {}", conf.slices);

	// Not going to display segment data unless asked for it.
	if (conf.segmentSize)
		p_err_logger->error( "Segment size {}, segment wrap value {}, initial segment number {}", conf.segmentSize, conf.segmentWrap, conf.segmentNumber);

	if (conf.raw_output)
		p_err_logger->error( "Raw output enabled, format {}", raw_output_fmt_map_rev.at(conf.raw_output_fmt));

	p_err_logger->error( "Wait method: " + wait_method_description.at(conf.waitMethod));
	
	p_err_logger->error( "Initial conf '{}'", initial_map_rev.at(conf.bStartRunning));

	//ORIG raspipreview_dump_parameters(&conf.preview_parameters);
	{
		p_err_logger->error( "Preview {}, Full screen {}", 
							 conf.wantPreview ? "Yes" : "No",
							 conf.wantFullScreenPreview ? "Yes" : "No");
		p_err_logger->error( "Preview window {},{},{},{}\nOpacity {}",
							 conf.previewWindow.x,
							 conf.previewWindow.y, 
							 conf.previewWindow.width,
							 conf.previewWindow.height, conf.opacity);
	}
	
	RaspiCamControl::DumpParameters(conf.camera_parameters);
}


/**
 * Open a file based on the settings in state
 *
 * @param state Pointer to state
 */
// TODO: what about switching to c++ style file?
static bool open_filename(const RaspiEncamodeConfig &config,
						  const std::string &filename_in,
						  FILE ** filed)
{
	FILE *new_handle = NULL;  //TBD
	*filed = nullptr;
	std::string filename = filename_in;
	std::string tempname;

	if (config.segmentSize || config.splitWait)
	{
		// Create a new filename string

		//If %d/{:d} or any valid combination e.g. %04d is specified, assume segment number.
		//bool bSegmentNumber = false;
		//const char* pPercent = 
		size_t pos_percent = filename.find('%');
		size_t pos_d = filename.find('d', pos_percent);
		size_t pos_u = filename.find('u', pos_percent);
		size_t pos_end = pos_d < pos_u ? pos_d : pos_u;
		
		char* ttt;
		if (pos_percent!= filename.npos && pos_end != filename.npos)
		{
			//bSegmentNumber = true;
			asprintf(&ttt, filename.c_str(), config.segmentNumber);
		}
		else
		{
			char temp_ts_str[100];
			time_t t = time(NULL);
			struct tm *tm = localtime(&t);
			strftime(temp_ts_str, 100, filename.c_str(), tm);
			asprintf(&ttt, "%s", temp_ts_str);
		}
		tempname = std::string(ttt);
		free( ttt);

		filename = tempname;
	}

	if( !filename.empty() )
	{
		bool bNetwork = false;
		int sfd = -1, socktype;

		if(!filename.compare(0,6,"tcp://"))
		{
			bNetwork = true;
			socktype = SOCK_STREAM;
		}
		else if(!filename.compare(0,6,"udp://"))
		{
			if (config.netListen)
			{
				p_err_logger->error( "No support for listening in UDP mode");
				return false;
			}
			bNetwork = true;
			socktype = SOCK_DGRAM;
		}

		if(bNetwork)
		{
			std::string ip = filename;
			ip.erase(0,6);
			unsigned short port;
			//char *colon;
			unsigned int colon_pos = 0;
			colon_pos = ip.find(':');
 			//if(NULL == (colon = strchr(ip.c_str(), ':')))
			if(colon_pos == ip.npos)
			{
				p_err_logger->error( "{} is not a valid IPv4:port, use something like tcp://1.2.3.4:1234 or udp://1.2.3.4:1234",
						ip);
				return false;
			}
			ip.erase(0,colon_pos+1);
			//if(1 != sscanf(colon + 1, "%hu", &port))
			if(1 != sscanf(ip.c_str(), "%hu", &port))
			{
				p_err_logger->error(
						"Port parse failed. %s is not a valid network file name, use something like tcp://1.2.3.4:1234 or udp://1.2.3.4:1234",
						ip);
				return false;
			}
			//char chTmp = *colon;
			//*colon = 0;
			char chTmp = ip[0];
			ip[0] = 0;

			struct sockaddr_in saddr= {};
			saddr.sin_family = AF_INET;
			saddr.sin_port = htons(port);
			if(0 == inet_aton(ip.c_str(), &saddr.sin_addr))
			{
				p_err_logger->error( "inet_aton failed. {} is not a valid IPv4 address",
						ip);
				return false;
			}
			//*colon = chTmp;
			ip[0] = chTmp;

			if (config.netListen)
			{
				int sockListen = socket(AF_INET, SOCK_STREAM, 0);
				if (sockListen >= 0)
				{
					int iTmp = 1;
					setsockopt(sockListen, SOL_SOCKET, SO_REUSEADDR, &iTmp, sizeof(int));//no error handling, just go on
					if (bind(sockListen, (struct sockaddr *) &saddr, sizeof(saddr)) >= 0)
					{
						while ((-1 == (iTmp = listen(sockListen, 0))) && (EINTR == errno))
							;
						if (-1 != iTmp)
						{
							p_err_logger->error( "Waiting for a TCP connection on {}:{}...",
									inet_ntoa(saddr.sin_addr), ntohs(saddr.sin_port));
							struct sockaddr_in cli_addr;
							socklen_t clilen = sizeof(cli_addr);
							while ((-1 == (sfd = accept(sockListen, (struct sockaddr *) &cli_addr, &clilen))) && (EINTR == errno))
								;
							if (sfd >= 0)
								p_err_logger->error( "Client connected from {}:{}", inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));
							else
								p_err_logger->error( "Error on accept: {}", strerror(errno));
						}
						else//if (-1 != iTmp)
						{
							p_err_logger->error( "Error trying to listen on a socket: {}", strerror(errno));
						}
					}
					else//if (bind(sockListen, (struct sockaddr *) &saddr, sizeof(saddr)) >= 0)
					{
						p_err_logger->error( "Error on binding socket: {}", strerror(errno));
					}
				}
				else//if (sockListen >= 0)
				{
					p_err_logger->error( "Error creating socket: {}", strerror(errno));
				}

				if (sockListen >= 0)//regardless success or error
					close(sockListen);//do not listen on a given port anymore
			}
			else//if (config.netListen)
			{
				if(0 <= (sfd = socket(AF_INET, socktype, 0)))
				{
					p_err_logger->error( "Connecting to {}:{}...", inet_ntoa(saddr.sin_addr), port);

					int iTmp = 1;
					while ((-1 == (iTmp = connect(sfd, (struct sockaddr *) &saddr, sizeof(struct sockaddr_in)))) && (EINTR == errno))
						;
					if (iTmp < 0)
						p_err_logger->error( "error: {}", strerror(errno));
					else
						p_err_logger->error( "connected, sending video...");
				}
				else
					p_err_logger->error( "Error creating socket: {}", strerror(errno));
			}

			if (sfd >= 0)
				new_handle = fdopen(sfd, "w");
		}
		else
		{
			new_handle = fopen(filename.c_str(), "wb");
		}
	}

	if (config.verbose)
	{
		if (new_handle)
			p_err_logger->info( "Opening output file \"{}\"", filename);
		else
			p_err_logger->info( "Failed to open new file \"{}\"", filename);
	}

	(*filed) = new_handle;
	return true;
}

/**
 * Update any annotation data specific to the video.
 * This simply passes on the setting from cli, or
 * if application defined annotate requested, updates
 * with the H264 parameters
 *
 * @param state Pointer to state control struct
 *
 *//*
static void update_annotation_data(RaspiEncamodeConfig *state)
{
   // So, if we have asked for a application supplied string, set it to the H264 or GPS parameters
   if (state->camera_parameters.enable_annotate & ANNOTATE_APP_TEXT)
   {
      char *text;

      if (state->gps)
      {
         text = raspi_gps_location_string();
      }
      else
      {
         const char *refresh = raspicli_unmap_xref(state->intra_refresh_type, intra_refresh_map, intra_refresh_map_size);

         asprintf(&text,  "%dk,%df,%s,%d,%s,%s",
                  state->bitrate / 1000,  state->framerate,
                  refresh ? refresh : "(none)",
                  state->intraperiod,
                  raspicli_unmap_xref(state->profile, profile_map, profile_map_size),
                  raspicli_unmap_xref(state->level, level_map, level_map_size));
      }

      raspicamcontrol_set_annotate(_p_camera_component, state->camera_parameters.enable_annotate, text,
                                   state->camera_parameters.annotate_text_size,
                                   state->camera_parameters.annotate_text_colour,
                                   state->camera_parameters.annotate_bg_colour,
                                   state->camera_parameters.annotate_justify,
                                   state->camera_parameters.annotate_x,
                                   state->camera_parameters.annotate_y
                                  );

      free(text);
   }
   else
   {
      raspicamcontrol_set_annotate(_p_camera_component, state->camera_parameters.enable_annotate, state->camera_parameters.annotate_string,
                                   state->camera_parameters.annotate_text_size,
                                   state->camera_parameters.annotate_text_colour,
                                   state->camera_parameters.annotate_bg_colour,
                                   state->camera_parameters.annotate_justify,
                                   state->camera_parameters.annotate_x,
                                   state->camera_parameters.annotate_y
                                  );
   }
}*/

bool RaspiEncamode::InitComponents()
{
	MMAL_STATUS_T status = MMAL_SUCCESS;
	if ((status = CreateCameraComponent(_conf, &_p_camera_component)) != MMAL_SUCCESS)
	{
		p_err_logger->error( "{}: Failed to create camera component", __func__);
		return false;
	}
	else if( (status = CreatePreview(_conf, &_p_preview_component)) != MMAL_SUCCESS)
	{
		p_err_logger->error( "{}: Failed to create preview component", __func__);
		DestroyCameraComponent( &_p_camera_component );
		return false;
	}
	else if( (status = CreateEncoderComponent(_conf, &_p_encoder_component, &_p_encoder_pool, &(_conf.bitrate))) != MMAL_SUCCESS)
	{
		p_err_logger->error( "{}: Failed to create encode component", __func__);
		DestroyPreview(&_p_preview_component);
		DestroyCameraComponent( &_p_camera_component );
		return false;
	}
	else if( _conf.raw_output && 
			(status = CreateSplitterComponent(
				_conf, _p_camera_component, 
				&_p_splitter_component, &_p_splitter_pool) ) != MMAL_SUCCESS)
	{
		p_err_logger->error( "{}: Failed to create splitter component", __func__);
		DestroyPreview(&_p_preview_component);
		DestroyCameraComponent( &_p_camera_component );
		DestroyEncoderComponent(&_p_encoder_component, &_p_encoder_pool);
		return false;
	}
	return true;
}


bool RaspiEncamode::ConnectComponents()
{
	MMAL_STATUS_T status = MMAL_SUCCESS;;
	if (_conf.verbose)
		p_err_logger->info( "Starting component connection stage");

	_p_camera_preview_port = _p_camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
	_p_camera_video_port   = _p_camera_component->output[MMAL_CAMERA_VIDEO_PORT];
	_p_camera_still_port   = _p_camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
	_p_preview_input_port  = _p_preview_component->input[0];
	_p_encoder_input_port  = _p_encoder_component->input[0];
	_p_encoder_output_port = _p_encoder_component->output[0];

	if (_conf.raw_output)
	{
		_p_splitter_input_port = _p_splitter_component->input[0];
		_p_splitter_output_port = _p_splitter_component->output[SPLITTER_OUTPUT_PORT];
		_p_splitter_preview_port = _p_splitter_component->output[SPLITTER_PREVIEW_PORT];
	}

	if( _conf.wantPreview )
	{
		if (_conf.raw_output)
		{
			if (_conf.verbose)
			p_err_logger->info( "Connecting camera preview port to splitter input port");

			// Connect camera to splitter
			status = ConnectPorts(_p_camera_preview_port, _p_splitter_input_port, &_p_splitter_connection);

			if (status != MMAL_SUCCESS)
			{
				_p_splitter_component = NULL;
				p_err_logger->error("{}: Failed to connect camera preview port to splitter input", __func__);
				SelfDestruct(status);
				return false;
			}

			if (_conf.verbose)
			{
				p_err_logger->info( "Connecting splitter preview port to preview input port");
				p_err_logger->info( "Starting video preview");
			}

			// Connect splitter to preview
			status = ConnectPorts(_p_splitter_preview_port, _p_preview_input_port, &_p_preview_connection);
		}
		else
		{
			if (_conf.verbose)
			{
				p_err_logger->info( "Connecting camera preview port to preview input port");
				p_err_logger->info( "Starting video preview");
			}

			// Connect camera to preview
			status = ConnectPorts(_p_camera_preview_port, _p_preview_input_port, &_p_preview_connection);
		}

		if (status != MMAL_SUCCESS)
		{
			_p_preview_connection = NULL;
			p_err_logger->error("{}: Failed to connect camera preview port to camera input", __func__);
			SelfDestruct(status);
		}
	}
	else
	{
		if (_conf.raw_output)
		{
			if (_conf.verbose)
			p_err_logger->info( "Connecting camera preview port to splitter input port");

			// Connect camera to splitter
			status = ConnectPorts(_p_camera_preview_port, _p_splitter_input_port, &_p_splitter_connection);

			if (status != MMAL_SUCCESS)
			{
				_p_splitter_connection = NULL;
				p_err_logger->error("{}: Failed to connect camera preview port to splitter input", __func__);
				SelfDestruct(status);
				return false;
			}
		}
		else
		{
			status = MMAL_SUCCESS;
		}
	}

	if (status == MMAL_SUCCESS)
	{
		if (_conf.verbose)
			p_err_logger->info( "Connecting camera video port to encoder input port");

		// Now connect the camera to the encoder
		status = ConnectPorts(_p_camera_video_port, _p_encoder_input_port, &_p_encoder_connection);

		if (status != MMAL_SUCCESS)
		{
			_p_encoder_connection = NULL;
			p_err_logger->error("{}: Failed to connect camera video port to encoder input", __func__);
			SelfDestruct(status);
			return false;
		}
	}
	return true;
}


void RaspiEncamode::SelfDestruct(const MMAL_STATUS_T & status )
{
	MmalstatusToMsg(status);
	
	if (_conf.verbose)
		p_err_logger->error( "Closing down");

	// Disable all our ports that are not handled by connections
	DisablePort(&_p_camera_still_port);
	DisablePort(&_p_encoder_output_port);
	DisablePort(&_p_splitter_output_port);

	if (_conf.wantPreview && _p_preview_connection)
		mmal_connection_destroy(_p_preview_connection);

	if (_p_encoder_connection)
		mmal_connection_destroy(_p_encoder_connection);

	if (_p_splitter_connection)
		mmal_connection_destroy(_p_splitter_connection);
	
	_p_preview_connection = NULL;
	_p_encoder_connection = NULL;
	_p_splitter_connection = NULL;

	// Can now close our file. Note disabling ports may flush buffers which causes
	// problems if we have already closed the file!
	// TODO: switch to c++ std files
	if (_callback_data.file_handle && _callback_data.file_handle != stdout)
		fclose(_callback_data.file_handle);
	if (_callback_data.imv_file_handle && _callback_data.imv_file_handle != stdout)
		fclose(_callback_data.imv_file_handle);
	if (_callback_data.pts_file_handle && _callback_data.pts_file_handle != stdout)
		fclose(_callback_data.pts_file_handle);
	if (_callback_data.raw_file_handle && _callback_data.raw_file_handle != stdout)
		fclose(_callback_data.raw_file_handle);

	/* Disable components */
	if (_p_encoder_component)
		mmal_component_disable(_p_encoder_component);

	if (_p_preview_component)
		mmal_component_disable(_p_preview_component);

	if (_p_splitter_component)
		mmal_component_disable(_p_splitter_component);

	if (_p_camera_component)
		mmal_component_disable(_p_camera_component);
	
	DestroyEncoderComponent(&_p_encoder_component, &_p_encoder_pool);
	DestroyPreview(&_p_preview_component);
	DestroySplitterComponent(&_p_splitter_component, &_p_splitter_pool);
	DestroyCameraComponent( &_p_camera_component );

	if (_conf.verbose)
		p_err_logger->info( "Close down completed, all components disconnected, disabled and destroyed");
	return;
}


// Set up our userdata - this is passed though to the callback where we need the information.
bool RaspiEncamode::ConfigureCallback()
{
	_callback_data.pconfig = &_conf;
	_callback_data.abort = 0;
	_callback_data.pact_obj = this;
	_callback_data.flush_buffers = _conf.flush_buffers;

	MMAL_STATUS_T status;
	if (_conf.raw_output)
	{
		_p_splitter_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&_callback_data;

		if (_conf.verbose)
			p_err_logger->info( "Enabling splitter output port");

		// Enable the splitter output port and tell it its callback function
		status = mmal_port_enable(
					_p_splitter_output_port,
					SplitterBufferCallbackWrapper);
					//&RaspiEncamode::SplitterBufferCallback  );

		if (status != MMAL_SUCCESS)
		{
			p_err_logger->error("{}: Failed to setup splitter output port", __func__);
			SelfDestruct(status);
			return false;
		}
	}

	_callback_data.file_handle = NULL;

	if( !_conf.filename.empty() )
	{
		if (_conf.filename[0] == '-')
		{
			_callback_data.file_handle = stdout;
		}
		else
		{
			if( !open_filename(_conf, _conf.filename, &_callback_data.file_handle) )
			{
				p_err_logger->error("File opening failed with filename: {}", _conf.filename);
				return false;
			}
		}

		if (!_callback_data.file_handle)
		{
			// Notify user, carry on but discarding encoded output buffers
			p_err_logger->error("{}: Error opening output file: %s\nNo output file will be generated\n", __func__, _conf.filename);
		}
	}

	_callback_data.imv_file_handle = NULL;

	if( !_conf.imv_filename.empty() )
	{
		if (_conf.imv_filename[0] == '-')
		{
			_callback_data.imv_file_handle = stdout;
		}
		else
		{
			if( !open_filename(_conf, _conf.imv_filename, &_callback_data.imv_file_handle))
			{
				p_err_logger->error("File opening failed with filename: {}", _conf.imv_filename);
				return false;
			}
		}

		if (!_callback_data.imv_file_handle)
		{
			// Notify user, carry on but discarding encoded output buffers
			p_err_logger->error( "Error opening output file: {}\nNo output file will be generated",_conf.imv_filename);
			_conf.inlineMotionVectors=0;
		}
	}

	_callback_data.pts_file_handle = NULL;

	if( !_conf.pts_filename.empty() )
	{
		if (_conf.pts_filename[0] == '-')
		{
			_callback_data.pts_file_handle = stdout;
		}
		else
		{
			if( !open_filename(_conf, _conf.pts_filename, &_callback_data.pts_file_handle))
			{
				p_err_logger->error( "Error opening pts output file: {}",_conf.pts_filename);
			}
			if (_callback_data.pts_file_handle) /* save header for mkvmerge */
				fprintf(_callback_data.pts_file_handle, "# timecode format v2");
		}

		if (!_callback_data.pts_file_handle)
		{
			// Notify user, carry on but discarding encoded output 	buffers
			p_err_logger->error( "Error opening output file: {}\nNo output file will be generated",_conf.pts_filename);
			_conf.save_pts=0;
		}
	}

	_callback_data.raw_file_handle = NULL;

	if( !_conf.raw_filename.empty() )
	{
		if (_conf.raw_filename[0] == '-')
		{
			_callback_data.raw_file_handle = stdout;
		}
		else
		{
			if( !open_filename(_conf, _conf.raw_filename, &_callback_data.raw_file_handle) )
			{
				p_err_logger->error( "Error opening raw output file: {}", _conf.raw_filename);
			}
		}

		if (!_callback_data.raw_file_handle)
		{
			// Notify user, carry on but discarding encoded output buffers
			p_err_logger->error( "Error opening output file: {}\nNo output file will be generated", _conf.raw_filename);
			_conf.raw_output = 0;
		}
	}

	if(_conf.bCircularBuffer)
	{
		if(_conf.bitrate == 0)
		{
			p_err_logger->error("{}: Error circular buffer requires constant bitrate and small intra period", __func__);
			SelfDestruct(status);
			return false;
		}
		else if(_conf.timeout == 0)
		{
			p_err_logger->error("{}: Error, circular buffer size is based on timeout must be greater than zero", __func__);
			SelfDestruct(status);
			return false;
		}
		else if(_conf.waitMethod != WaitMethod::KEYPRESS && _conf.waitMethod != WaitMethod::WM_SIGNAL)
		{
			p_err_logger->error("{}: Error, Circular buffer mode requires either keypress (-k) or signal (-s) triggering\n", __func__);
			SelfDestruct(status);
			return false;
		}
		else if(!_callback_data.file_handle)
		{
			p_err_logger->error("{}: Error require output file (or stdout) for Circular buffer mode", __func__);
			SelfDestruct(status);
			return false;
		}
		else
		{
			int count = _conf.bitrate * (_conf.timeout / 1000) / 8;
	
			_callback_data.cb_buff = (char *) malloc(count);
			if(_callback_data.cb_buff == NULL)
			{
				p_err_logger->error("{}: Unable to allocate circular buffer for {} seconds at %.1f Mbits", __func__, _conf.timeout / 1000, (double)_conf.bitrate/1000000.0);
				SelfDestruct(status);
				return false;
			}
			else
			{
				_callback_data.cb_len = count;
				_callback_data.cb_wptr = 0;
				_callback_data.cb_wrap = 0;
				_callback_data.cb_data = 0;
				_callback_data.iframe_buff_wpos = 0;
				_callback_data.iframe_buff_rpos = 0;
				_callback_data.header_wptr = 0;
			}
		}
	}
	return true;
}

bool RaspiEncamode::EnablePort()
{
	MMAL_PARAMETER_INT64_T param;
	param.hdr.id = MMAL_PARAMETER_SYSTEM_TIME;
	param.hdr.size = sizeof(param);
	param.value = -1;
	mmal_port_parameter_get(_p_encoder_output_port, &param.hdr);//time in microseconds
	uint64_t pts=param.value/1000;
	//uint64_t cpumonotonic=GetSteadyMs64();   // do this?
	uint64_t cpumonotonic=GetMsSinceUtcToday();
	//cpumonotonic=GPUCPUMONOTONICOFFSET+pts;
	GPUCPUMONOTONICOFFSET=cpumonotonic-pts;
	p_err_logger->info( "(ms) GPU: {}, CPU: {}, OFFSET= {}", pts ,cpumonotonic, GPUCPUMONOTONICOFFSET);

	// Set up our userdata - this is passed though to the callback where we need the information.
	_p_encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&_callback_data;

	if (_conf.verbose)
		p_err_logger->info( "Enabling encoder output port");

	// Enable the encoder output port and tell it its callback function
	MMAL_STATUS_T status = mmal_port_enable(
		_p_encoder_output_port,EncoderBufferCallbackWrapper);
		//std::bind( &RaspiEncamode::EncoderBufferCallback, 
		//		   this, 
		//		   std::placeholders::_2, std::placeholders::_3) );

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("Failed to setup encoder output");
		SelfDestruct(status);
		return false;
	}
	return true;
}

bool RaspiEncamode::Run()
{
	// Send all the buffers to the encoder output port
	if (_callback_data.file_handle)
	{
		int num = mmal_queue_length(_p_encoder_pool->queue);
		int q;
		for (q=0; q<num; q++)
		{
			MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(_p_encoder_pool->queue);

			if (!buffer)
				vcos_log_error("Unable to get a required buffer %d from pool queue", q);

			if (mmal_port_send_buffer(_p_encoder_output_port, buffer)!= MMAL_SUCCESS)
				vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
		}
	}

	// Send all the buffers to the splitter output port
	if (_callback_data.raw_file_handle)
	{
		int num = mmal_queue_length(_p_splitter_pool->queue);
		int q;
		for (q = 0; q < num; q++)
		{
			MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(_p_splitter_pool->queue);

			if (!buffer)
				vcos_log_error("Unable to get a required buffer %d from pool queue", q);

			if (mmal_port_send_buffer(_p_splitter_output_port, buffer)!= MMAL_SUCCESS)
				vcos_log_error("Unable to send a buffer to splitter output port (%d)", q);
		}
	}
	
	// Only encode stuff if we have a filename and it opened
	// Note we use the copy in the callback, as the call back MIGHT change the file handle
	if( _callback_data.file_handle || 
		_callback_data.raw_file_handle  )
	{
		int running = 1;
		int initialCapturing = _conf.bStartRunning;
		
		if( !_conf.bStartRunning)
			_is_capturing = true; // So it switches to stop
		
		while (running)
		{
			// Toggle state
			_is_capturing = !_is_capturing;

			if( mmal_port_parameter_set_boolean(_p_camera_video_port, MMAL_PARAMETER_CAPTURE, _is_capturing) != MMAL_SUCCESS)
			{
				// How to handle?
			}

			// In circular buffer mode, exit and save the buffer (make sure we do this after having paused the capture
			if(_conf.bCircularBuffer && !_is_capturing)
			{
				break;
			}

			if( _conf.verbose )
			{
				if (_is_capturing)
					p_err_logger->info( "Starting video capture");
				else
					p_err_logger->info( "Pausing video capture");
			}

			if( _conf.splitWait )
			{
				if(_is_capturing)
				{
					if( mmal_port_parameter_set_boolean(_p_encoder_output_port, MMAL_PARAMETER_VIDEO_REQUEST_I_FRAME, 1) != MMAL_SUCCESS)
					{
						p_err_logger->error("failed to request I-FRAME");
					}
				}
				else
				{
					if(initialCapturing)
						_split_now = true;
				}
				initialCapturing = true;
			}
			running = WaitForNextChange();
		}

		if (_conf.verbose)
			p_err_logger->error( "Finished capture");
	}
	else
	{
		if (_conf.timeout)
			std::this_thread::sleep_for(
				std::chrono::milliseconds(_conf.timeout));
		else
		{
			// timeout = 0 so run forever
			while(1)
				std::this_thread::sleep_for( std::chrono::milliseconds(ABORT_INTERVAL) );
		}
	}
	
	
	if(_conf.bCircularBuffer)
	{
		int copy_from_end, copy_from_start;

		copy_from_end = _callback_data.cb_len - _callback_data.iframe_buff[_callback_data.iframe_buff_rpos];
		copy_from_start = _callback_data.cb_len - copy_from_end;
		copy_from_start = _callback_data.cb_wptr < copy_from_start ? _callback_data.cb_wptr : copy_from_start;
		if(!_callback_data.cb_wrap)
		{
			copy_from_start = _callback_data.cb_wptr;
			copy_from_end = 0;
		}

		fwrite(_callback_data.header_bytes, 1, _callback_data.header_wptr, _callback_data.file_handle);
		// Save circular buffer
		fwrite(_callback_data.cb_buff + _callback_data.iframe_buff[_callback_data.iframe_buff_rpos], 1, copy_from_end, _callback_data.file_handle);
		fwrite(_callback_data.cb_buff, 1, copy_from_start, _callback_data.file_handle);
		if(_callback_data.flush_buffers) 
			fflush(_callback_data.file_handle);
	}
	return true;
}


void writeTimeStamp(FILE*p,uint64_t time){
    if(p==NULL)return;
    fprintf(p,"%llu\t%i\n",time,timeStamp_GlobalCounter++);
    fflush(p); // pmedina
}

int64_t RaspiEncamode::GetSteadyUs64()
{
	auto steady_dur = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch());
	return steady_dur.count();
}
int64_t RaspiEncamode::GetSteadyMs64()
{
	auto steady_dur = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch());
	return steady_dur.count();
}

int64_t RaspiEncamode::GetMsSinceUtcToday()
{
	/* original Replace this C function
	auto getClockMs = []()
	{
		struct timeval tv;
		gettimeofday(&tv, NULL);
		return (unsigned long long)(tv.tv_sec) * 1000 +(unsigned long long)(tv.tv_usec) / 1000;
	};*/
	typedef std::chrono::duration<int, std::ratio<86400>> Days;
	
	auto clk = std::chrono::high_resolution_clock::now();
	Days today = std::chrono::duration_cast<Days>(clk.time_since_epoch());
	auto time_since_today = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(clk.time_since_epoch() - today);
	return std::chrono::duration_cast<std::chrono::milliseconds>(time_since_today).count();
}


void EncoderBufferCallbackWrapper(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
	pData->pact_obj->EncoderBufferCallback(port, buffer);
}
void SplitterBufferCallbackWrapper(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
	pData->pact_obj->SplitterBufferCallback(port, buffer);
}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
void RaspiEncamode::EncoderBufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	MMAL_BUFFER_HEADER_T *new_buffer;
	static int64_t base_time =  -1;
	static int64_t last_second = -1;

	// All our segment times based on the receipt of the first encoder callback
	if (base_time == -1)
		base_time = GetSteadyMs64();
	
	// We pass our file handle and other stuff in via the userdata field.

	PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

	if (pData)
	{
		int bytes_written = buffer->length;
		int64_t current_time = GetSteadyMs64();

		vcos_assert(pData->file_handle);
		if(pData->pconfig->inlineMotionVectors) vcos_assert(pData->imv_file_handle);

		if (pData->cb_buff)
		{
			int space_in_buff = pData->cb_len - pData->cb_wptr;
			int copy_to_end = space_in_buff > (int)buffer->length ? buffer->length : space_in_buff;
			int copy_to_start = buffer->length - copy_to_end;

			if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG)
			{
				if(pData->header_wptr + buffer->length > sizeof(pData->header_bytes))
				{
					p_err_logger->error("Error in header bytes\n");
				}
				else
				{
					// These are the header bytes, save them for final output
					mmal_buffer_header_mem_lock(buffer);
					memcpy(pData->header_bytes + pData->header_wptr, buffer->data, buffer->length);
					mmal_buffer_header_mem_unlock(buffer);
					pData->header_wptr += buffer->length;
				}
			}
			else if((buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO))
			{
				// Do something with the inline motion vectors...
			}
			else
			{
				static int frame_start = -1;
				int i;

				if(frame_start == -1)
					frame_start = pData->cb_wptr;

				if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_KEYFRAME)
				{
					pData->iframe_buff[pData->iframe_buff_wpos] = frame_start;
					pData->iframe_buff_wpos = (pData->iframe_buff_wpos + 1) % IFRAME_BUFSIZE;
				}

				if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END)
				frame_start = -1;

				// If we overtake the iframe rptr then move the rptr along
				if((pData->iframe_buff_rpos + 1) % IFRAME_BUFSIZE != pData->iframe_buff_wpos)
				{
					while(
						(
							pData->cb_wptr <= pData->iframe_buff[pData->iframe_buff_rpos] &&
							(int)(pData->cb_wptr + buffer->length) > pData->iframe_buff[pData->iframe_buff_rpos]
						) ||
						(
							(pData->cb_wptr > pData->iframe_buff[pData->iframe_buff_rpos]) &&
							(int)(pData->cb_wptr + buffer->length) > (pData->iframe_buff[pData->iframe_buff_rpos] + pData->cb_len)
						)
					)
						pData->iframe_buff_rpos = (pData->iframe_buff_rpos + 1) % IFRAME_BUFSIZE;
				}

				mmal_buffer_header_mem_lock(buffer);
				// We are pushing data into a circular buffer
				memcpy(pData->cb_buff + pData->cb_wptr, buffer->data, copy_to_end);
				memcpy(pData->cb_buff, buffer->data + copy_to_end, copy_to_start);
				mmal_buffer_header_mem_unlock(buffer);

				if((int)(pData->cb_wptr + buffer->length) > pData->cb_len)
				pData->cb_wrap = 1;

				pData->cb_wptr = (pData->cb_wptr + buffer->length) % pData->cb_len;

				for(i = pData->iframe_buff_rpos; i != pData->iframe_buff_wpos; i = (i + 1) % IFRAME_BUFSIZE)
				{
					int p = pData->iframe_buff[i];
					if(pData->cb_buff[p] != 0 || pData->cb_buff[p+1] != 0 || pData->cb_buff[p+2] != 0 || pData->cb_buff[p+3] != 1)
					{
						p_err_logger->error("Error in iframe list\n");
					}
				}
			}
		}
		else
		{
			// For segmented record mode, we need to see if we have exceeded our time/size,
			// but also since we have inline headers turned on we need to break when we get one to
			// ensure that the new stream has the header in it. If we break on an I-frame, the
			// SPS/PPS header is actually in the previous chunk.
			if ((buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) &&
				((pData->pconfig->segmentSize && current_time > base_time + pData->pconfig->segmentSize) ||
					(pData->pconfig->splitWait && _split_now)))
			{
				FILE *new_handle;

				base_time = current_time;

				_split_now = false;
				pData->pconfig->segmentNumber++;

				// Only wrap if we have a wrap point set
				if (pData->pconfig->segmentWrap && pData->pconfig->segmentNumber > pData->pconfig->segmentWrap)
				pData->pconfig->segmentNumber = 1;

				if(!pData->pconfig->filename.empty() && pData->pconfig->filename[0] != '-')
				{
					 bool suc = open_filename( *(pData->pconfig), pData->pconfig->filename, &new_handle);

					if(suc && new_handle)
					{
						fclose(pData->file_handle);
						pData->file_handle = new_handle;
					}
				}

				if( !pData->pconfig->imv_filename.empty() && pData->pconfig->imv_filename[0] != '-')
				{
					bool suc = open_filename(*(pData->pconfig), pData->pconfig->imv_filename, &new_handle);

					if (suc && new_handle)
					{
						fclose(pData->imv_file_handle);
						pData->imv_file_handle = new_handle;
					}
				}

				if( !pData->pconfig->pts_filename.empty() && pData->pconfig->pts_filename[0] != '-')
				{
					bool suc = open_filename( *(pData->pconfig), pData->pconfig->pts_filename, &new_handle);

					if(suc && new_handle)
					{
						fclose(pData->pts_file_handle);
						pData->pts_file_handle = new_handle;
					}
				}
			}

			if (buffer->length)
			{
				mmal_buffer_header_mem_lock(buffer);
				if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO)
				{
					if(pData->pconfig->inlineMotionVectors)
					{
						bytes_written = fwrite(buffer->data, 1, buffer->length, pData->imv_file_handle);
						if(pData->flush_buffers) fflush(pData->imv_file_handle);
					}
					else
					{
						//We do not want to save inlineMotionVectors...
						bytes_written = buffer->length;
					}
				}
				else
				{
					bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);
					if(pData->flush_buffers)
					{
						fflush(pData->file_handle);
						fdatasync(fileno(pData->file_handle));
					}

					if (pData->pconfig->save_pts &&
							!(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) &&
							buffer->pts != MMAL_TIME_UNKNOWN &&
							buffer->pts != _lastpts)
					{
						int64_t pts;
						if (_frame == 0)
							_startpts = buffer->pts;
						
						_lastpts = buffer->pts;
						pts = buffer->pts - _startpts;
						// TODO offset to UTC?
						writeTimeStamp(pData->pts_file_handle,(buffer->pts/1000)+GPUCPUMONOTONICOFFSET);
						_frame++;
						(void) pts; //TODO
					}
				}

				mmal_buffer_header_mem_unlock(buffer);

				if (bytes_written != (int)buffer->length)
				{
					p_err_logger->error("Failed to write buffer data ({} from {})- aborting", bytes_written, buffer->length);
					pData->abort = 1; //TODO ewing: what is this?
				}
			}
		}

		// See if the second count has changed and we need to update any annotation
		if (current_time/1000 != last_second)
		{
		//   update_annotation_data(pData->pconfig);
			last_second = current_time/1000;
		}
	}
	else
	{
		p_err_logger->error("Received a encoder buffer callback with no state");
	}

	// release buffer back to the pool
	mmal_buffer_header_release(buffer);

	// and send one back to the port (if still open)
	if (port->is_enabled)
	{
		MMAL_STATUS_T status;

		new_buffer = mmal_queue_get(_p_encoder_pool->queue);

		if (new_buffer)
			status = mmal_port_send_buffer(port, new_buffer);

		if (!new_buffer || status != MMAL_SUCCESS)
			p_err_logger->error("Unable to return a buffer to the encoder port");
	}
}

/**
 *  buffer header callback function for splitter
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
void RaspiEncamode::SplitterBufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	//  printf("SplitterBufferCallback\n");
	MMAL_BUFFER_HEADER_T *new_buffer;
	PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

	if (pData)
	{
		int bytes_written = 0;
		int bytes_to_write = buffer->length;

		/* Write only luma component to get grayscale image: */
		if( buffer->length && 
			pData->pconfig->raw_output_fmt == RawOutputFmt::ROF_GRAY )
			bytes_to_write = port->format->es->video.width * port->format->es->video.height;

		vcos_assert(pData->raw_file_handle);

		if( bytes_to_write )
		{
			mmal_buffer_header_mem_lock(buffer);
			bytes_written = fwrite(buffer->data, 1, bytes_to_write, pData->raw_file_handle);
			mmal_buffer_header_mem_unlock(buffer);

			if (bytes_written != bytes_to_write)
			{
				p_err_logger->error("Failed to write raw buffer data ({} from {})- aborting", bytes_written, bytes_to_write);
				pData->abort = 1;
			}
		}
	}
	else
	{
		p_err_logger->error("Received a camera buffer callback with no state");
	}

	// release buffer back to the pool
	mmal_buffer_header_release(buffer);

	// and send one back to the port (if still open)
	if (port->is_enabled)
	{
		MMAL_STATUS_T status;
		new_buffer = mmal_queue_get(_p_splitter_pool->queue);

		if (new_buffer)
			status = mmal_port_send_buffer(port, new_buffer);

		if (!new_buffer || status != MMAL_SUCCESS)
			p_err_logger->error("Unable to return a buffer to the splitter port");
	}
}

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *  Note conf.framerate might be changed to 0
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
#define CAM_CREATE_ERROR \
	if(camera) \
		mmal_component_destroy(camera); \
	return status;
MMAL_STATUS_T RaspiEncamode::CreateCameraComponent(RaspiEncamodeConfig & conf, 
										   MMAL_COMPONENT_T ** camera_component)
{
	MMAL_COMPONENT_T *camera = 0;
	MMAL_ES_FORMAT_T *format;
	MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
	MMAL_STATUS_T status;
	
	/* Create the component */
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("Failed to create camera component");
		CAM_CREATE_ERROR;
	}

	using namespace RaspiCamControl;
	int status_int;
	status_int = SetStereoMode(camera->output[0], conf.camera_parameters.stereo_mode);
	status_int += SetStereoMode(camera->output[1], conf.camera_parameters.stereo_mode);
	status_int += SetStereoMode(camera->output[2], conf.camera_parameters.stereo_mode);

	if (status_int != 0)
	{
		p_err_logger->error("Could not set stereo mode : error {}", status_int);
		CAM_CREATE_ERROR;
	}

	MMAL_PARAMETER_INT32_T camera_num =
	{{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, conf.cameraNum};

	status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("Could not select camera : error {}", status);
		CAM_CREATE_ERROR;
	}

	if (!camera->output_num)
	{
		status = MMAL_ENOSYS;
		p_err_logger->error("Camera doesn't have output ports");
		CAM_CREATE_ERROR;
	}

	status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, conf.sensor_mode);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("Could not set sensor mode : error {}", status);
		CAM_CREATE_ERROR;
	}

	preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
	video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
	still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

	// Enable the camera, and tell it its control callback function
	status = mmal_port_enable(camera->control, RaspiCamControl::DefaultCallback);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("Unable to enable control port : error {}", status);
		CAM_CREATE_ERROR;
	}

	//  set up the camera configuration
	{
		MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
		{
			{ MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
			(uint32_t) conf.width, 	// .max_stills_w
			(uint32_t)  conf.height,	//max_stills_h
			(uint32_t) 0,				//stills_yuv422
			(uint32_t) 0,				//one_shot_stills
			(uint32_t) conf.width,	//max_preview_video_w
			(uint32_t) conf.height,	//max_preview_video_h
			(uint32_t) 3 + vcos_max(0, (conf.framerate-30)/10),//num_preview_video_frames
			(uint32_t) 0,				//stills_capture_circular_buffer_height
			(uint32_t) 0,				//fast_preview_resume
			MMAL_PARAM_TIMESTAMP_MODE_RAW_STC	//use_stc_timestamp
		};
		mmal_port_parameter_set(camera->control, &cam_config.hdr);
	}

	// Now set up the port formats

	// Set the encode format on the Preview port
	// HW limitations mean we need the preview to be the same size as the required recorded output

	format = preview_port->format;

	format->encoding = MMAL_ENCODING_OPAQUE;
	format->encoding_variant = MMAL_ENCODING_I420;

	if(conf.camera_parameters.shutter_speed > 6000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
			{ 50, 1000 }, {166, 1000}
		};
		mmal_port_parameter_set(preview_port, &fps_range.hdr);
	}
	else if(conf.camera_parameters.shutter_speed > 1000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
			{ 166, 1000 }, {999, 1000}
		};
		mmal_port_parameter_set(preview_port, &fps_range.hdr);
	}

	//enable dynamic framerate if necessary
	if (conf.camera_parameters.shutter_speed)
	{
		if (conf.framerate > 1000000./conf.camera_parameters.shutter_speed)
		{
			conf.framerate=0;
			if (conf.verbose)
				p_err_logger->error( "Enable dynamic frame rate to fulfil shutter speed requirement");
		}
	}

	format->encoding = MMAL_ENCODING_OPAQUE;
	format->es->video.width = VCOS_ALIGN_UP(conf.width, 32);
	format->es->video.height = VCOS_ALIGN_UP(conf.height, 16);
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = conf.width;
	format->es->video.crop.height = conf.height;
	format->es->video.frame_rate.num = conf.framerate;
	format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

	status = mmal_port_format_commit(preview_port);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("camera viewfinder format couldn't be set");
		CAM_CREATE_ERROR;
	}

	// Set the encode format on the video  port

	format = video_port->format;
	format->encoding_variant = MMAL_ENCODING_I420;

	if(conf.camera_parameters.shutter_speed > 6000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
			{ 50, 1000 }, {166, 1000}
		};
		mmal_port_parameter_set(video_port, &fps_range.hdr);
	}
	else if(conf.camera_parameters.shutter_speed > 1000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
			{ 167, 1000 }, {999, 1000}
		};
		mmal_port_parameter_set(video_port, &fps_range.hdr);
	}

	format->encoding = MMAL_ENCODING_OPAQUE;
	format->es->video.width = VCOS_ALIGN_UP(conf.width, 32);
	format->es->video.height = VCOS_ALIGN_UP(conf.height, 16);
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = conf.width;
	format->es->video.crop.height = conf.height;
	format->es->video.frame_rate.num = conf.framerate;
	format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

	status = mmal_port_format_commit(video_port);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("camera video format couldn't be set");
		CAM_CREATE_ERROR;
	}

	// Ensure there are enough buffers to avoid dropping frames
	if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
		video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;


	// Set the encode format on the still  port

	format = still_port->format;

	format->encoding = MMAL_ENCODING_OPAQUE;
	format->encoding_variant = MMAL_ENCODING_I420;

	format->es->video.width = VCOS_ALIGN_UP(conf.width, 32);
	format->es->video.height = VCOS_ALIGN_UP(conf.height, 16);
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = conf.width;
	format->es->video.crop.height = conf.height;
	format->es->video.frame_rate.num = 0;
	format->es->video.frame_rate.den = 1;

	status = mmal_port_format_commit(still_port);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("camera still format couldn't be set");
		CAM_CREATE_ERROR;
	}

	/* Ensure there are enough buffers to avoid dropping frames */
	if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
		still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

	/* Enable component */
	status = mmal_component_enable(camera);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("camera component couldn't be enabled");
		CAM_CREATE_ERROR;
	}

	// Note: this sets lots of parameters that were not individually addressed before.
	// TODO
	RaspiCamControl::SetAllParameters(camera, &conf.camera_parameters);

	(*camera_component) = camera;

	// update_annotation_data(state);

	if (conf.verbose)
		p_err_logger->error( "Camera component done");

	return status;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
void RaspiEncamode::DestroyCameraComponent(MMAL_COMPONENT_T ** camera_component)
{
   if( (*camera_component) )
   {
      mmal_component_destroy( *camera_component );
      *camera_component = NULL;
   }
}

/**
 * Create the splitter component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
#define SPLITTER_CREATE_ERROR \
	if (splitter)\
		mmal_component_destroy(splitter);\
	return status;
MMAL_STATUS_T RaspiEncamode::CreateSplitterComponent(
			const RaspiEncamodeConfig &conf, 
			const MMAL_COMPONENT_T * camera_component, 
			MMAL_COMPONENT_T ** splitter_component, 
			MMAL_POOL_T ** splitter_pool)
{
	MMAL_COMPONENT_T *splitter = 0;
	MMAL_PORT_T *splitter_output = NULL;
	MMAL_ES_FORMAT_T *format;
	MMAL_STATUS_T status;
	MMAL_POOL_T *pool;

	if (camera_component == NULL)
	{
		status = MMAL_ENOSYS;
		p_err_logger->error("Camera component must be created before splitter");
		SPLITTER_CREATE_ERROR;
	}

	/* Create the component */
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_SPLITTER, &splitter);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("Failed to create splitter component");
		SPLITTER_CREATE_ERROR;
	}

	if (!splitter->input_num)
	{
		status = MMAL_ENOSYS;
		p_err_logger->error("Splitter doesn't have any input port");
		SPLITTER_CREATE_ERROR;
	}

	if (splitter->output_num < 2)
	{
		status = MMAL_ENOSYS;
		p_err_logger->error("Splitter doesn't have enough output ports");
		SPLITTER_CREATE_ERROR;
	}

	/* Ensure there are enough buffers to avoid dropping frames: */
	mmal_format_copy(splitter->input[0]->format, camera_component->output[MMAL_CAMERA_PREVIEW_PORT]->format);

	if (splitter->input[0]->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
		splitter->input[0]->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

	status = mmal_port_format_commit(splitter->input[0]);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("Unable to set format on splitter input port");
		SPLITTER_CREATE_ERROR;
	}

	/* Splitter can do format conversions, configure format for its output port: */
	for (unsigned int i = 0; i < splitter->output_num; i++)
	{
		mmal_format_copy(splitter->output[i]->format, splitter->input[0]->format);

		if (i == SPLITTER_OUTPUT_PORT)
		{
			format = splitter->output[i]->format;

			switch (conf.raw_output_fmt)
			{
			case RawOutputFmt::ROF_YUV:
			case RawOutputFmt::ROF_GRAY: /* Grayscale image contains only luma (Y) component */
				format->encoding = MMAL_ENCODING_I420;
				format->encoding_variant = MMAL_ENCODING_I420;
				break;
			case RawOutputFmt::ROF_RGB:
				if (mmal_util_rgb_order_fixed(camera_component->output[MMAL_CAMERA_CAPTURE_PORT]))
				format->encoding = MMAL_ENCODING_RGB24;
				else
				format->encoding = MMAL_ENCODING_BGR24;
				format->encoding_variant = 0;  /* Irrelevant when not in opaque mode */
				break;
			default:
				status = MMAL_EINVAL;
				p_err_logger->error("unknown raw output format");
				SPLITTER_CREATE_ERROR;
			}
		}

		status = mmal_port_format_commit(splitter->output[i]);

		if (status != MMAL_SUCCESS)
		{
			p_err_logger->error("Unable to set format on splitter output port {}", i);
			SPLITTER_CREATE_ERROR;
		}
	}

	/* Enable component */
	status = mmal_component_enable(splitter);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("splitter component couldn't be enabled");
		SPLITTER_CREATE_ERROR;
	}

	/* Create pool of buffer headers for the output port to consume */
	splitter_output = splitter->output[SPLITTER_OUTPUT_PORT];
	pool = mmal_port_pool_create(splitter_output, splitter_output->buffer_num, splitter_output->buffer_size);

	if (!pool)
	{
		p_err_logger->error("Failed to create buffer header pool for splitter output port %s", splitter_output->name);
	}

	(*splitter_pool) = pool;
	(*splitter_component) = splitter;

	if (conf.verbose)
		p_err_logger->error( "Splitter component done");

	return status;
}

/**
 * Destroy the splitter component
 *
 * @param state Pointer to state control struct
 *
 */
void RaspiEncamode::DestroySplitterComponent(MMAL_COMPONENT_T ** splitter_component, MMAL_POOL_T **splitter_pool)
{
   // Get rid of any port buffers first
   if(*splitter_pool)
   {
      mmal_port_pool_destroy( (*splitter_component)->output[SPLITTER_OUTPUT_PORT], (*splitter_pool) );
   }

   if (*splitter_component)
   {
      mmal_component_destroy(*splitter_component);
      (*splitter_component) = NULL;
   }
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */

#define ENC_CREATE_ERROR \
	if (encoder)\
		mmal_component_destroy(encoder);\
	(*encoder_component) = NULL;\
	return status;\
	
MMAL_STATUS_T RaspiEncamode::CreateEncoderComponent(
			RaspiEncamodeConfig & conf, 
			MMAL_COMPONENT_T ** encoder_component, 
			MMAL_POOL_T ** encoder_pool, 
			int * bitrate)
{
	MMAL_COMPONENT_T *encoder = 0;
	MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
	MMAL_STATUS_T status;
	MMAL_POOL_T *pool;

	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("Unable to create video encoder component");
		ENC_CREATE_ERROR;
	}

	if (!encoder->input_num || !encoder->output_num)
	{
		status = MMAL_ENOSYS;
		p_err_logger->error("Video encoder doesn't have input/output ports");
		ENC_CREATE_ERROR;
	}

	encoder_input = encoder->input[0];
	encoder_output = encoder->output[0];

	// We want same format on input and output
	mmal_format_copy(encoder_output->format, encoder_input->format);

	// Only supporting H264 at the moment
	encoder_output->format->encoding = conf.encoding;

	if(conf.encoding == MMAL_ENCODING_H264)
	{
		if(conf.level == MMAL_VIDEO_LEVEL_H264_4)
		{
			if((*bitrate) > MAX_BITRATE_LEVEL4)
			{
				p_err_logger->error( "Bitrate too high: Reducing to 25MBit/s");
				(*bitrate) = MAX_BITRATE_LEVEL4;
			}
		}
		else
		{
			if((*bitrate) > MAX_BITRATE_LEVEL42)
			{
				p_err_logger->error( "Bitrate too high: Reducing to 62.5MBit/s");
				(*bitrate) = MAX_BITRATE_LEVEL42;
			}
		}
	}
	else if(conf.encoding == MMAL_ENCODING_MJPEG)
	{
		if((*bitrate) > MAX_BITRATE_MJPEG)
		{
			p_err_logger->error( "Bitrate too high: Reducing to 25MBit/s");
			(*bitrate) = MAX_BITRATE_MJPEG;
		}
	}

	encoder_output->format->bitrate = (*bitrate);

	if (conf.encoding == MMAL_ENCODING_H264)
		encoder_output->buffer_size = encoder_output->buffer_size_recommended;
	else
		encoder_output->buffer_size = 256<<10;


	if (encoder_output->buffer_size < encoder_output->buffer_size_min)
		encoder_output->buffer_size = encoder_output->buffer_size_min;

	encoder_output->buffer_num = encoder_output->buffer_num_recommended;

	if (encoder_output->buffer_num < encoder_output->buffer_num_min)
		encoder_output->buffer_num = encoder_output->buffer_num_min;

	// We need to set the frame rate on output to 0, to ensure it gets
	// updated correctly from the input framerate when port connected
	encoder_output->format->es->video.frame_rate.num = 0;
	encoder_output->format->es->video.frame_rate.den = 1;

	// Commit the port changes to the output port
	status = mmal_port_format_commit(encoder_output);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("Unable to set format on video encoder output port");
		ENC_CREATE_ERROR;
	}

	// Set the rate control parameter
	if (0)
	{
		MMAL_PARAMETER_VIDEO_RATECONTROL_T param = {{ MMAL_PARAMETER_RATECONTROL, sizeof(param)}, MMAL_VIDEO_RATECONTROL_DEFAULT};
		status = mmal_port_parameter_set(encoder_output, &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			p_err_logger->error("Unable to set ratecontrol");
			ENC_CREATE_ERROR;
		}

	}

	if (conf.encoding == MMAL_ENCODING_H264 &&
			conf.intraperiod != -1)
	{
		MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, (uint32_t)conf.intraperiod};
		status = mmal_port_parameter_set(encoder_output, &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			p_err_logger->error("Unable to set intraperiod");
			ENC_CREATE_ERROR;
		}
	}

	if (conf.encoding == MMAL_ENCODING_H264 && conf.slices > 1 && conf.width <= 1280)
	{
		int frame_mb_rows = VCOS_ALIGN_UP(conf.height, 16) >> 4;

		if (conf.slices > frame_mb_rows) //warn user if too many slices selected
		{
			p_err_logger->error("H264 Slice count ({}) exceeds number of macroblock rows ({}). Setting slices to {}.", conf.slices, frame_mb_rows, frame_mb_rows);
			// Continue rather than abort..
		}
		int slice_row_mb = frame_mb_rows/conf.slices;
		if (frame_mb_rows - conf.slices*slice_row_mb)
			slice_row_mb++; //must round up to avoid extra slice if not evenly divided

		status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_MB_ROWS_PER_SLICE, slice_row_mb);
		if (status != MMAL_SUCCESS)
		{
			p_err_logger->error("Unable to set number of slices");
			ENC_CREATE_ERROR;
		}
	}

	if (conf.encoding == MMAL_ENCODING_H264 &&
		conf.quantisationParameter)
	{
		MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, (uint32_t)conf.quantisationParameter};
		status = mmal_port_parameter_set(encoder_output, &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			p_err_logger->error("Unable to set initial QP");
			ENC_CREATE_ERROR;
		}

		MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param)}, (uint32_t)conf.quantisationParameter};
		status = mmal_port_parameter_set(encoder_output, &param2.hdr);
		if (status != MMAL_SUCCESS)
		{
			p_err_logger->error("Unable to set min QP");
			ENC_CREATE_ERROR;
		}

		MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param)}, (uint32_t)conf.quantisationParameter};
		status = mmal_port_parameter_set(encoder_output, &param3.hdr);
		if (status != MMAL_SUCCESS)
		{
			p_err_logger->error("Unable to set max QP");
			ENC_CREATE_ERROR;
		}
	}

	if (conf.encoding == MMAL_ENCODING_H264)
	{
		MMAL_PARAMETER_VIDEO_PROFILE_T  param;
		param.hdr.id = MMAL_PARAMETER_PROFILE;
		param.hdr.size = sizeof(param);

		param.profile[0].profile = conf.profile;

		if((VCOS_ALIGN_UP(conf.width,16) >> 4) * (VCOS_ALIGN_UP(conf.height,16) >> 4) * conf.framerate > 245760)
		{
			if((VCOS_ALIGN_UP(conf.width,16) >> 4) * (VCOS_ALIGN_UP(conf.height,16) >> 4) * conf.framerate <= 522240)
			{
				p_err_logger->error( "Too many macroblocks/s: Increasing H264 Level to 4.2");
				conf.level=MMAL_VIDEO_LEVEL_H264_42;//TODO
			}
			else
			{
				p_err_logger->error("Too many macroblocks/s requested");
				status = MMAL_EINVAL;
				ENC_CREATE_ERROR;
			}
		}

		param.profile[0].level = conf.level;

		status = mmal_port_parameter_set(encoder_output, &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			p_err_logger->error("Unable to set H264 profile");
			ENC_CREATE_ERROR;
		}
	}

	if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, conf.immutableInput) != MMAL_SUCCESS)
	{
		p_err_logger->error("Unable to set immutable input flag");
		// Continue rather than abort..
	}

	if (conf.encoding == MMAL_ENCODING_H264)
	{
		//set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
		if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, conf.bInlineHeaders) != MMAL_SUCCESS)
		{
			p_err_logger->error("failed to set INLINE HEADER FLAG parameters");
			// Continue rather than abort..
		}

		//set flag for add SPS TIMING
		if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_SPS_TIMING, conf.addSPSTiming) != MMAL_SUCCESS)
		{
			p_err_logger->error("failed to set SPS TIMINGS FLAG parameters");
			// Continue rather than abort..
		}

		//set INLINE VECTORS flag to request motion vector estimates
		if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, conf.inlineMotionVectors) != MMAL_SUCCESS)
		{
			p_err_logger->error("failed to set INLINE VECTORS parameters");
			// Continue rather than abort..
		}

		// Adaptive intra refresh settings
		if ( conf.intra_refresh_type != -1)
		{
			MMAL_PARAMETER_VIDEO_INTRA_REFRESH_T  param;
			param.hdr.id = MMAL_PARAMETER_VIDEO_INTRA_REFRESH;
			param.hdr.size = sizeof(param);

			// Get first so we don't overwrite anything unexpectedly
			status = mmal_port_parameter_get(encoder_output, &param.hdr);
			if (status != MMAL_SUCCESS)
			{
				vcos_log_warn("Unable to get existing H264 intra-refresh values. Please update your firmware");
				// Set some defaults, don't just pass random stack data
				param.air_mbs = param.air_ref = param.cir_mbs = param.pir_mbs = 0;
			}

			param.refresh_mode = (MMAL_VIDEO_INTRA_REFRESH_T)conf.intra_refresh_type;

			//if (conf.intra_refresh_type == MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS)
			//   param.cir_mbs = 10;

			status = mmal_port_parameter_set(encoder_output, &param.hdr);
			if (status != MMAL_SUCCESS)
			{
				p_err_logger->error("Unable to set H264 intra-refresh values");
				ENC_CREATE_ERROR;
			}
		}
	}

	//  Enable component
	status = mmal_component_enable(encoder);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("Unable to enable video encoder component");
		ENC_CREATE_ERROR;
	}

	/* Create pool of buffer headers for the output port to consume */
	pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

	if (!pool)
	{
		p_err_logger->error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
	}

	(*encoder_pool) = pool;
	(*encoder_component) = encoder;

	if (conf.verbose)
		p_err_logger->info( "Encoder component done");

	return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
void RaspiEncamode::DestroyEncoderComponent(MMAL_COMPONENT_T ** encoder_component, MMAL_POOL_T ** encoder_pool)
{
	// Get rid of any port buffers first
	if ((*encoder_pool))
	{
		mmal_port_pool_destroy( (*encoder_component)->output[0], (*encoder_pool) );
	}

	if ((*encoder_component))
	{
		mmal_component_destroy((*encoder_component));
		(*encoder_component) = NULL;
	}
}


/**
 * Pause for specified time, but return early if detect an abort request
 *
 * @param state Pointer to state control struct
 * @param pause Time in ms to pause
 * @param callback Struct contain an abort flag tested for early termination
 **/
static int PauseAndTestAbort(int *abort_tgt, int pause)
{
	if( pause == 0)
		return 0;

	p_err_logger ->info("PauseAdnTestAbort for {}",pause);
	// Going to check every ABORT_INTERVAL milliseconds
	for(int wait = 0; wait < pause; wait+= ABORT_INTERVAL)
	{
		std::this_thread::sleep_for(
			std::chrono::milliseconds(ABORT_INTERVAL) );
		if( *abort_tgt )
			return 1;
	}
	return 0;
}


/**
 * Function to wait in various ways (depending on settings)
 *
 * @param state Pointer to the state data
 *
 * @return !0 if to continue, 0 if reached end of run
 */
int RaspiEncamode::WaitForNextChange()
{
	int keep_running = 1;
	static int64_t complete_time = -1;

	// Have we actually exceeded our timeout?
	int64_t current_time = GetSteadyMs64();

	if (complete_time == -1)
		complete_time =  current_time + _conf.timeout;

	// if we have run out of time, flag we need to exit
	if (current_time >= complete_time && _conf.timeout != 0)
		keep_running = 0;

	switch(_conf.waitMethod)
	{
	case WaitMethod::NONE:
	{
		p_err_logger->info("no wait method");
		(void)PauseAndTestAbort(&_callback_data.abort, _conf.timeout);
		return 0;
	}

	case WaitMethod::FOREVER:
	{
		// We never return from this. Expect a ctrl-c to exit or abort.
		while (!_callback_data.abort)
			// Have a sleep so we don't hog the CPU.
			std::this_thread::sleep_for( std::chrono::milliseconds(ABORT_INTERVAL) );
		return 0;
	}

	case WaitMethod::TIMED:
	{
		int abort;

		if( _is_capturing )
			abort = PauseAndTestAbort(&_callback_data.abort, _conf.onTime);
		else
			abort = PauseAndTestAbort(&_callback_data.abort, _conf.offTime);

		if (abort)
			return 0;
		else
			return keep_running;
	}

	// TODO: do we need this within the library?
	case WaitMethod::KEYPRESS:
	{
		if (_conf.verbose)
			p_err_logger->error( "Press Enter to {}, X then ENTER to exit, [i,o,r] then ENTER to change zoom", _is_capturing ? "pause" : "capture");
		
		char ch = getchar();
		if (ch == 'x' || ch == 'X')
		{
			return 0;
		}
		else if (ch == 'i' || ch == 'I')
		{
			if (_conf.verbose)
				p_err_logger->error( "Starting zoom in");

			RaspiCamControl::ZoomInOut(_p_camera_component, RaspiCamControl::ZOOM_COMMAND_T::ZOOM_IN, &(_conf.camera_parameters.roi));

			if (_conf.verbose)
				DumpConfig(_conf);
		}
		else if (ch == 'o' || ch == 'O')
		{
			if (_conf.verbose)
				p_err_logger->error( "Starting zoom out");

			RaspiCamControl::ZoomInOut(_p_camera_component, RaspiCamControl::ZOOM_COMMAND_T::ZOOM_OUT, &(_conf.camera_parameters.roi));

			if (_conf.verbose)
				DumpConfig(_conf);
		}
		else if (ch == 'r' || ch == 'R')
		{
			if (_conf.verbose)
				p_err_logger->error( "starting reset zoom");

			RaspiCamControl::ZoomInOut(_p_camera_component, RaspiCamControl::ZOOM_COMMAND_T::ZOOM_RESET, &(_conf.camera_parameters.roi));

			if (_conf.verbose)
				DumpConfig(_conf);
		}
		
		return keep_running;
	}

	case WaitMethod::WM_SIGNAL:
	{
		// Need to wait for a SIGUSR1 signal
		sigset_t waitset;
		int sig;
		int result = 0;

		//TBD
		sigemptyset( &waitset );
		sigaddset( &waitset, SIGUSR1 );

		// We are multi threaded because we use mmal, so need to use the pthread
		// variant of procmask to block SIGUSR1 so we can wait on it.
		pthread_sigmask( SIG_BLOCK, &waitset, NULL );

		if (_conf.verbose)
		{
			p_err_logger->error( "Waiting for SIGUSR1 to {}", _is_capturing ? "pause" : "capture");
		}

		result = sigwait( &waitset, &sig );

		if (_conf.verbose && result != 0)
			p_err_logger->error( "Bad signal received - error {}", errno);

		return keep_running;
	}

	} // switch

	return keep_running;
}



// RaspiPreview

/**
 * Create the preview component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */

#define PREVIEW_CREATE_ERROR \
	if (preview) \
		mmal_component_destroy(preview); \
	return status;
MMAL_STATUS_T RaspiEncamode::CreatePreview( const RaspiEncamodeConfig & conf, 
							 MMAL_COMPONENT_T ** preview_component)
{
	MMAL_COMPONENT_T *preview = 0;
	MMAL_PORT_T *preview_port = NULL;
	MMAL_STATUS_T status;

	if (!conf.wantPreview)
	{
		// No preview required, so create a null sink component to take its place
		status = mmal_component_create("vc.null_sink", &preview);

		if (status != MMAL_SUCCESS)
		{
			p_err_logger->error("Unable to create null sink component");
			PREVIEW_CREATE_ERROR;
		}
	}
	else
	{
		status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER,
										&preview);

		if (status != MMAL_SUCCESS)
		{
			p_err_logger->error("Unable to create preview component");
			PREVIEW_CREATE_ERROR;
		}

		if (!preview->input_num)
		{
			status = MMAL_ENOSYS;
			p_err_logger->error("No input ports found on component");
			PREVIEW_CREATE_ERROR;
		}

		preview_port = preview->input[0];

		MMAL_DISPLAYREGION_T param;
		param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
		param.hdr.size = sizeof(MMAL_DISPLAYREGION_T);

		param.set = MMAL_DISPLAY_SET_LAYER;
		param.layer = PREVIEW_LAYER;

		param.set |= MMAL_DISPLAY_SET_ALPHA;
		param.alpha = conf.opacity;

		if (conf.wantFullScreenPreview)
		{
			param.set |= MMAL_DISPLAY_SET_FULLSCREEN;
			param.fullscreen = 1;
		}
		else
		{
			param.set |= (MMAL_DISPLAY_SET_DEST_RECT | MMAL_DISPLAY_SET_FULLSCREEN);
			param.fullscreen = 0;
			param.dest_rect = conf.previewWindow;
		}

		status = mmal_port_parameter_set(preview_port, &param.hdr);

		if (status != MMAL_SUCCESS && status != MMAL_ENOSYS)
		{
			p_err_logger->error("unable to set preview port parameters ({:d})", status);
			PREVIEW_CREATE_ERROR;
		}
	}

	/* Enable component */
	status = mmal_component_enable(preview);

	if (status != MMAL_SUCCESS)
	{
		p_err_logger->error("Unable to enable preview/null sink component ({:d})", status);
		PREVIEW_CREATE_ERROR;
	}

	(*preview_component) = preview;

	return status;
}


/**
 * Destroy the preview component
 *
 * @param state Pointer to state control struct
 *
 */
void RaspiEncamode::DestroyPreview(MMAL_COMPONENT_T ** preview_component)
{
   if(*preview_component)
   {
      mmal_component_destroy((*preview_component));
      (*preview_component) = NULL;
   }
}



/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
MMAL_STATUS_T RaspiEncamode::ConnectPorts(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
	MMAL_STATUS_T status;

	status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

	if (status == MMAL_SUCCESS)
	{
		status =  mmal_connection_enable(*connection);
		if (status != MMAL_SUCCESS)
			mmal_connection_destroy(*connection);
	}

	return status;
}

/**
 * Convert a MMAL status return value to a simple boolean of success
 * ALso displays a fault if code is not success
 *
 * @param status The error code to convert
 * @return 0 if status is success, 1 otherwise
 */
void RaspiEncamode::MmalstatusToMsg(const MMAL_STATUS_T & status)
{
	std::unordered_map<MMAL_STATUS_T, std::string> map_mmal_status_to_string {
		{ MMAL_SUCCESS, std::string("Success")},
		{ MMAL_ENOMEM, std::string("Out of memory") },
		{ MMAL_ENOSPC, std::string("Out of resources (other than memory)") },
		{ MMAL_EINVAL, std::string("Argument is invalid") },
		{ MMAL_ENOSYS, std::string("Function not implemented") },
		{ MMAL_ENOENT, std::string("No such file or directory") },
		{ MMAL_ENXIO, std::string("No such device or address") },
		{ MMAL_EIO, std::string("I/O error") },
		{ MMAL_ESPIPE, std::string("Illegal seek") },
		{ MMAL_ECORRUPT, std::string("Data is corrupt \attention FIXME: not POSIX") },
		{ MMAL_ENOTREADY, std::string("Component is not ready \attention FIXME: not POSIX") },
		{ MMAL_ECONFIG, std::string("Component is not configured \attention FIXME: not POSIX") },
		{ MMAL_EISCONN, std::string("Port is already connected ") },
		{ MMAL_ENOTCONN, std::string("Port is disconnected") },
		{ MMAL_EAGAIN, std::string("Resource temporarily unavailable. Try again later") },
		{ MMAL_EFAULT, std::string("Bad address") }
	};  // end of map
	
	if (status == MMAL_SUCCESS) 
	{
		p_err_logger->info( "Success" );
		return;
	}
	try {
		p_err_logger->error( map_mmal_status_to_string.at(status) );
	}
	catch(...) {
		p_err_logger->error("Unknown status error: {}", status);
			return;
	}
	
	return;
}


/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
void RaspiEncamode::DisablePort(MMAL_PORT_T **port)
{
	if (*port && (*port)->is_enabled)
		mmal_port_disable(*port);
	*port = NULL;
}
