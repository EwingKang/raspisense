/*
Copyright (c) 2013, Broadcom Europe Ltd
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

#include <iostream>
//#include <stdio.h>
#include <memory.h>
#include <ctype.h>

//#include "interface/vcos/vcos.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h" // support for basic file logging
#include "spdlog/sinks/stdout_color_sinks.h"

#include "interface/vmcs_host/vc_vchi_gencmd.h"
#include "interface/mmal/mmal.h"
//#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"


#include "raspicam_camcontrol.hpp"

// TODO: 1151 strncat warning

#define parameter_reset -99999

#define zoom_full_16P16 ((unsigned int)(65536 * 0.15))
#define zoom_increment_16P16 (65536UL / 10)
namespace RaspiCamControl
{
static std::shared_ptr<spdlog::logger> p_err_logger = spdlog::stdout_color_mt("RPI_CAMCTRL");

// prototypes
static int MmalstatusToMsg(const MMAL_STATUS_T & status);
	

// ===== Type trait =====
// A system for generic use of getting parameters
template <class T>
struct MmalCamParamTrait {
	static const int id;
	static const char* name;
};
// Specializations
template <>
struct MmalCamParamTrait<MMAL_PARAMETER_EXPOSUREMETERINGMODE_T> {
	static const int id = MMAL_PARAMETER_EXP_METERING_MODE;
	static const char* name;
};
const char* MmalCamParamTrait<MMAL_PARAMETER_EXPOSUREMETERINGMODE_T>::name = "Exposure metering";
template <>
struct MmalCamParamTrait<MMAL_PARAMETER_EXPOSUREMODE_T> {
	static const int id = MMAL_PARAMETER_EXPOSURE_MODE;
	static const char* name;
};
const char* MmalCamParamTrait<MMAL_PARAMETER_EXPOSUREMODE_T>::name = "Exposure mode";
template <>
struct MmalCamParamTrait<MMAL_PARAMETER_FLICKERAVOID_T> {
	static const int id = MMAL_PARAMETER_FLICKER_AVOID;
	static const char* name;
};
const char* MmalCamParamTrait<MMAL_PARAMETER_FLICKERAVOID_T>::name = "Flicker avoid";
template <>
struct MmalCamParamTrait<MMAL_PARAMETER_AWBMODE_T> {
	static const int id = MMAL_PARAMETER_AWB_MODE;
	static const char* name;
};
const char* MmalCamParamTrait<MMAL_PARAMETER_AWBMODE_T>::name = "AWB mode";
template <>
struct MmalCamParamTrait<MMAL_PARAMETER_IMAGEFX_T> {
	static const int id = MMAL_PARAMETER_IMAGE_EFFECT;
	static const char* name;
};
const char* MmalCamParamTrait<MMAL_PARAMETER_IMAGEFX_T>::name = "Image FX";

// Following type trait deduced "enclosing type" from actual "mmal parameter type"
// The types are defined accroding to definition in  mmal_parameters_camera.h
template <class T>
struct MmalEnumBase {
	typedef void basetype; // dummy
};
template <>
struct MmalEnumBase<MMAL_PARAM_EXPOSUREMETERINGMODE_T> {
	typedef MMAL_PARAMETER_EXPOSUREMETERINGMODE_T basetype;
};
template <>
struct MmalEnumBase<MMAL_PARAM_EXPOSUREMODE_T> {
	typedef MMAL_PARAMETER_EXPOSUREMODE_T basetype;
};
template <>
struct MmalEnumBase<MMAL_PARAM_FLICKERAVOID_T> {
	typedef MMAL_PARAMETER_FLICKERAVOID_T basetype;
};
template <>
struct MmalEnumBase<MMAL_PARAM_AWBMODE_T> {
	typedef MMAL_PARAMETER_AWBMODE_T basetype;
};
template <>
struct MmalEnumBase<MMAL_PARAM_IMAGEFX_T> {
	typedef MMAL_PARAMETER_IMAGEFX_T basetype;
};


// ===== Members =====
void CamConfig::SetDefault() 
{
	this->sharpness = 0;
	this->contrast = 0;
	this->brightness = 50;
	this->saturation = 0;
	this->ISO = 0;                    // 0 = auto
	this->videoStabilisation = 0;
	this->exposureCompensation = 0;
	this->exposureMode = MMAL_PARAM_EXPOSUREMODE_AUTO;
	this->flickerAvoidMode = MMAL_PARAM_FLICKERAVOID_OFF;
	this->exposureMeterMode = MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
	this->awbMode = MMAL_PARAM_AWBMODE_AUTO;
	this->imageEffect = MMAL_PARAM_IMAGEFX_NONE;
	this->colourEffects.enable = 0;
	this->colourEffects.u = 128;
	this->colourEffects.v = 128;
	this->rotation = 0;
	this->hflip = this->vflip = 0;
	this->roi.x = this->roi.y = 0.0;
	this->roi.w = this->roi.h = 1.0;
	this->shutter_speed = 0;          // 0 = auto
	this->awb_gains_r = 0;      // Only have any function if AWB OFF is used.
	this->awb_gains_b = 0;
	this->drc_level = MMAL_PARAMETER_DRC_STRENGTH_OFF;
	this->stats_pass = MMAL_FALSE;
	this->enable_annotate = 0;
	this->annotate_string[0] = '\0';
	this->annotate_text_size = 0;	//Use firmware default
	this->annotate_text_colour = -1;   //Use firmware default
	this->annotate_bg_colour = -1;     //Use firmware default
	this->stereo_mode.mode = MMAL_STEREOSCOPIC_MODE_NONE;
	this->stereo_mode.decimate = MMAL_FALSE;
	this->stereo_mode.swap_eyes = MMAL_FALSE;
}
	
	
	
/**
 * Update the passed in parameter according to the rest of the parameters
 * passed in.
 *
 *
 * @return 0 if reached end of cycle for this parameter, !0 otherwise
 */
/*TBD
static int update_cycle_parameter(int *option, int min, int max, int increment)
{
   vcos_assert(option);
   if (!option)
      return 0;

   if (*option == parameter_reset)
      *option = min - increment;

   *option += increment;

   if (*option > max)
   {
      *option = parameter_reset;
      return 0;
   }
   else
      return 1;
}*/


/**
 * Test/Demo code to cycle through a bunch of camera settings
 * This code is pretty hacky so please don't complain!!
 * It only does stuff that should have a visual impact (hence demo!)
 * This will override any user supplied parameters
 *
 * Each call of this function will move on to the next setting
 *
 * @param camera Pointer to the camera to change settings on.
 * @return 0 if reached end of complete sequence, !0 otherwise
 */
// TBD
/*int raspicamcontrol_cycle_test(MMAL_COMPONENT_T *camera)
{
   static int parameter = 0;
   static int parameter_option = parameter_reset; // which value the parameter currently has

   vcos_assert(camera);

   // We are going to cycle through all the relevant entries in the parameter block
   // and send options to the camera.
   if (parameter == 0)
   {
      // sharpness
      if (update_cycle_parameter(&parameter_option, -100, 100, 10))
         SetSharpness(camera, parameter_option);
      else
      {
         SetSharpness(camera, 0);
         parameter++;
      }
   }
   else if (parameter == 1)
   {
      // contrast
      if (update_cycle_parameter(&parameter_option, -100, 100, 10))
         SetContrast(camera, parameter_option);
      else
      {
         SetContrast(camera, 0);
         parameter++;
      }
   }
   else if (parameter == 2)
   {
      // brightness
      if (update_cycle_parameter(&parameter_option, 0, 100, 10))
         SetBrightness(camera, parameter_option);
      else
      {
         SetBrightness(camera, 50);
         parameter++;
      }
   }
   else if (parameter == 3)
   {
      // contrast
      if (update_cycle_parameter(&parameter_option, -100, 100, 10))
         SetSaturation(camera, parameter_option);
      else
      {
         parameter++;
         SetSaturation(camera, 0);
      }
   }
   else if (parameter == 4)
   {
      // EV
      if (update_cycle_parameter(&parameter_option, -10, 10, 4))
         SetExposureCompensation(camera, parameter_option);
      else
      {
         SetExposureCompensation(camera, 0);
         parameter++;
      }
   }
   else if (parameter == 5)
   {
      // MMAL_PARAM_EXPOSUREMODE_T
      if (update_cycle_parameter(&parameter_option, 0, exposure_map_size, 1))
         SetExposureMode(camera, exposure_map[parameter_option].mmal_mode);
      else
      {
         SetExposureMode(camera, MMAL_PARAM_EXPOSUREMODE_AUTO);
         parameter++;
      }
   }
   else if (parameter == 6)
   {
      // MMAL_PARAM_AWB_T
      if (update_cycle_parameter(&parameter_option, 0, awb_map_size, 1))
         SetAwbMode(camera, awb_map[parameter_option].mmal_mode);
      else
      {
         SetAwbMode(camera, MMAL_PARAM_AWBMODE_AUTO);
         parameter++;
      }
   }
   if (parameter == 7)
   {
      // MMAL_PARAM_IMAGEFX_T
      if (update_cycle_parameter(&parameter_option, 0, imagefx_map_size, 1))
         SetImageFX(camera, imagefx_map[parameter_option].mmal_mode);
      else
      {
         SetImageFX(camera, MMAL_PARAM_IMAGEFX_NONE);
         parameter++;
      }
   }
   if (parameter == 8)
   {
      MMAL_PARAM_COLOURFX_T colfx = {0,0,0};// TODO: change to MMAL_PARAMETER_COLOURFX_T
      switch (parameter_option)
      {
      case parameter_reset :
         parameter_option = 1;
         colfx.u = 128;
         colfx.v = 128;
         break;
      case 1 :
         parameter_option = 2;
         colfx.u = 100;
         colfx.v = 200;
         break;
      case 2 :
         parameter_option = parameter_reset;
         colfx.enable = 0;
         parameter++;
         break;
      }
      SetColourFX(camera, &colfx);
   }

   // Orientation
   if (parameter == 9)
   {
      switch (parameter_option)
      {
      case parameter_reset:
         SetRotation(camera, 90);
         parameter_option = 1;
         break;

      case 1 :
         SetRotation(camera, 180);
         parameter_option = 2;
         break;

      case 2 :
         SetRotation(camera, 270);
         parameter_option = 3;
         break;

      case 3 :
      {
         SetRotation(camera, 0);
         SetFlips(camera, 1,0);
         parameter_option = 4;
         break;
      }
      case 4 :
      {
         SetFlips(camera, 0,1);
         parameter_option = 5;
         break;
      }
      case 5 :
      {
         SetFlips(camera, 1, 1);
         parameter_option = 6;
         break;
      }
      case 6 :
      {
         SetFlips(camera, 0, 0);
         parameter_option = parameter_reset;
         parameter++;
         break;
      }
      }
   }

   if (parameter == 10)
   {
      parameter = 1;
      return 0;
   }

   return 1;
}*/


/**
 * Display help for command line options
 */
void DisplayHelp()
{
	std::cout << "\nImage parameter commands\n\n";
	//raspicli_display_help(cmdline_commands, cmdline_commands_size);

	std::cout << "\n\nNotes\n\nExposure mode options :\n" << exposure_map.begin()->first;
	for( auto it = exposure_map.begin()++; it!= exposure_map.end(); ++it ) {
		std::cout << ", " << it->first;
	}

	std::cout << "\n\nFlicker avoid mode options :\n" << flicker_avoid_map.begin()->first;
	for( auto it = flicker_avoid_map.begin()++; it!= flicker_avoid_map.end(); ++it ) {
		std::cout << ", " << it->first;
	}


	std::cout << "\n\nAWB mode options :\n" << awb_map.begin()->first;
	for( auto it = awb_map.begin()++; it!= awb_map.end(); ++it ) {
		std::cout << ", " << it->first;
	}

	std::cout << "\n\nImage Effect mode options :\n", imagefx_map.begin()->first ;
	for( auto it = imagefx_map.begin()++; it!= imagefx_map.end(); ++it ) {
		std::cout << ", " << it->first;
	}
   
	std::cout << "\n\nMetering Mode options :\n", metering_mode_map.begin()->first ;
	for( auto it = metering_mode_map.begin()++; it!= metering_mode_map.end(); ++it ) {
		std::cout << ", " << it->first;
	}

	std::cout << "\n\nDynamic Range Compression (DRC) options :\n", drc_mode_map.begin()->first ;
	for( auto it = drc_mode_map.begin()++; it!= drc_mode_map.end(); ++it ) {
		std::cout << ", " << it->first;
	}
	std::cout << std::endl;
}


static std::unordered_map< decltype(exposure_map)::mapped_type,
						   decltype(exposure_map)::key_type> exposure_map_rev;
						   
static std::unordered_map< decltype(flicker_avoid_map)::mapped_type,
						   decltype(flicker_avoid_map)::key_type> flicker_avoid_map_rev;
						   
static std::unordered_map< decltype(awb_map)::mapped_type,
						   decltype(awb_map)::key_type> awb_map_rev;
						   
static std::unordered_map<decltype(imagefx_map)::mapped_type,
						  decltype(imagefx_map)::key_type> imagefx_map_rev;
						  
static std::unordered_map<decltype(metering_mode_map)::mapped_type, 
						  decltype(metering_mode_map)::key_type> metering_mode_map_rev;
						  
static std::unordered_map<decltype(drc_mode_map)::mapped_type, 
						  decltype(drc_mode_map)::key_type> drc_mode_map_rev;
static bool is_revmap_built = false;
void BuildRevMap()
{
	for(auto it = exposure_map.begin(); it != exposure_map.end(); it++)
	{
		exposure_map_rev.insert(
			std::make_pair(it->second, it->first)  	);
	}
	for(auto it = flicker_avoid_map.begin(); it != flicker_avoid_map.end(); it++)
	{
		flicker_avoid_map_rev.insert(
			std::make_pair(it->second, it->first)  	);
	}
	for(auto it = awb_map.begin(); it != awb_map.end(); it++)
	{
		awb_map_rev.insert(
			std::make_pair(it->second, it->first)  	);
	}
	
	for(auto it = imagefx_map.begin(); it != imagefx_map.end(); it++)
	{
		imagefx_map_rev.insert(
			std::make_pair(it->second, it->first)  	);
	}
	
	for(auto it = metering_mode_map.begin(); it != metering_mode_map.end(); it++)
	{
		metering_mode_map_rev.insert(
			std::make_pair(it->second, it->first)  	);
	}
	
	for(auto it = drc_mode_map.begin(); it != drc_mode_map.end(); it++)
	{
		drc_mode_map_rev.insert(
			std::make_pair(it->second, it->first)  	);
	}
	is_revmap_built = true;
}

/**
 * Dump contents of camera parameter structure to stderr for debugging/verbose logging
 *
 * @param params Const pointer to parameters structure to dump
 */
void DumpParameters(const CamConfig &params)
{
	if(!is_revmap_built) 
		BuildRevMap();
	
	std::string exp_mode = exposure_map_rev.at(params.exposureMode);
	std::string fl_mode = flicker_avoid_map_rev.at(params.flickerAvoidMode);
	std::string awb_mode = awb_map_rev.at(params.awbMode);
	std::string image_effect = imagefx_map_rev.at(params.imageEffect);
	std::string metering_mode = metering_mode_map_rev.at(params.exposureMeterMode);
	std::string drc_mode = drc_mode_map_rev.at(params.drc_level);

	p_err_logger->error( "Sharpness {}, Contrast {}, Brightness {}", params.sharpness, params.contrast, params.brightness);
	p_err_logger->error( "Saturation {}, ISO {}, Video Stabilisation {}, Exposure compensation {}", params.saturation, params.ISO, params.videoStabilisation ? "Yes": "No", params.exposureCompensation);
	p_err_logger->error( "Exposure Mode '{}', AWB Mode '{}', Image Effect '{}'", exp_mode, awb_mode, image_effect);
	p_err_logger->error( "Flicker Avoid Mode '{}'", fl_mode);
	p_err_logger->error( "Metering Mode '{}', Colour Effect Enabled {} with U = {}, V = {}", metering_mode, params.colourEffects.enable ? "Yes":"No", params.colourEffects.u, params.colourEffects.v);
	p_err_logger->error( "Dynamic Range Level '{}", drc_mode);
	p_err_logger->error( "Rotation {}, hflip {}, vflip {}", params.rotation, params.hflip ? "Yes":"No",params.vflip ? "Yes":"No");
	p_err_logger->error( "ROI x {}, y {}, w {} h {}", params.roi.x, params.roi.y, params.roi.w, params.roi.h);
}



/**
 * Give the supplied parameter block a set of default values
 * @params Pointer to parameter block
 */
void SetDefaults(CamConfig *params)
{
   vcos_assert(params);

   params->sharpness = 0;
   params->contrast = 0;
   params->brightness = 50;
   params->saturation = 0;
   params->ISO = 0;                    // 0 = auto
   params->videoStabilisation = 0;
   params->exposureCompensation = 0;
   params->exposureMode = MMAL_PARAM_EXPOSUREMODE_AUTO;
   params->flickerAvoidMode = MMAL_PARAM_FLICKERAVOID_OFF;
   params->exposureMeterMode = MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
   params->awbMode = MMAL_PARAM_AWBMODE_AUTO;
   params->imageEffect = MMAL_PARAM_IMAGEFX_NONE;
   params->colourEffects.enable = 0;
   params->colourEffects.u = 128;
   params->colourEffects.v = 128;
   params->rotation = 0;
   params->hflip = params->vflip = 0;
   params->roi.x = params->roi.y = 0.0;
   params->roi.w = params->roi.h = 1.0;
   params->shutter_speed = 0;          // 0 = auto
   params->awb_gains_r = 0;      // Only have any function if AWB OFF is used.
   params->awb_gains_b = 0;
   params->drc_level = MMAL_PARAMETER_DRC_STRENGTH_OFF;
   params->stats_pass = MMAL_FALSE;
   params->enable_annotate = 0;
   params->annotate_string[0] = '\0';
   params->annotate_text_size = 0;	//Use firmware default
   params->annotate_text_colour = -1;   //Use firmware default
   params->annotate_bg_colour = -1;     //Use firmware default
   params->stereo_mode.mode = MMAL_STEREOSCOPIC_MODE_NONE;
   params->stereo_mode.decimate = MMAL_FALSE;
   params->stereo_mode.swap_eyes = MMAL_FALSE;
}

/**
 * Get all the current camera parameters from specified camera component
 * @param camera Pointer to camera component
 * @param params Pointer to parameter block to accept settings
 * @return 0 if successful, non-zero if unsuccessful
 */
int GetAllParameters(MMAL_COMPONENT_T *camera, CamConfig *params)
{
   vcos_assert(camera);
   vcos_assert(params);

   if (!camera || !params)
      return 1;

	int result = 0;
	result += GetSharpness(camera, &params->sharpness);
	result += GetContrast(camera, &params->contrast);
	result += GetBrightness(camera, &params->brightness);
	result += GetSaturation(camera, &params->saturation);
	result += GetISO(camera, &params->ISO);
	result += GetMeteringMode(camera, &params->exposureMeterMode);
	result += GetVideoStabilisation(camera, &params->videoStabilisation);
	result += GetExposureCompensation(camera, &params->exposureCompensation);
	result += GetExposureMode(camera, &params->exposureMode);
	result += GetFlickerAvoidMode(camera, &params->flickerAvoidMode);
	result += GetAwbMode(camera, &params->awbMode);
	result += GetImageFX(camera, &params->imageEffect);
	result += GetColourFX(camera, &params->colourEffects);
	// TODO not implemented
	// result += GetThumbnailConfig(camera, &params->thumbnailConfig);
	
	if( result )
		p_err_logger->error("Errors while retrieveing parameters from MMAL: {}", result);
   return result;
}

/**
 * Set the specified camera to all the specified settings
 * @param camera Pointer to camera component
 * @param params Pointer to parameter block containing parameters
 * @return 0 if successful, none-zero if unsuccessful.
 */
int SetAllParameters(MMAL_COMPONENT_T *camera, const CamConfig *params)
{
   int result;

   result  = SetSaturation(camera, params->saturation);
   result += SetSharpness(camera, params->sharpness);
   result += SetContrast(camera, params->contrast);
   result += SetBrightness(camera, params->brightness);
   result += SetISO(camera, params->ISO);
   result += SetVideoStabilisation(camera, params->videoStabilisation);
   result += SetExposureCompensation(camera, params->exposureCompensation);
   result += SetExposureMode(camera, params->exposureMode);
   result += SetFlickerAvoidMode(camera, params->flickerAvoidMode);
   result += SetMeteringMode(camera, params->exposureMeterMode);
   result += SetAwbMode(camera, params->awbMode);
   result += SetAwbGains(camera, params->awb_gains_r, params->awb_gains_b);
   result += SetImageFX(camera, params->imageEffect);
   result += SetColourFX(camera, &params->colourEffects);
   //result += SetThumbnailParameters(camera, &params->thumbnailConfig);  TODO Not working for some reason
   result += SetRotation(camera, params->rotation);
   result += SetFlips(camera, params->hflip, params->vflip);
   result += SetRoi(camera, params->roi);
   result += SetShutterSpeed(camera, params->shutter_speed);
   result += SetDrc(camera, params->drc_level);
   result += SetStatsPass(camera, params->stats_pass);
   result += SetAnnotate(camera, params->enable_annotate, params->annotate_string,
                                          params->annotate_text_size,
                                          params->annotate_text_colour,
                                          params->annotate_bg_colour,
                                          params->annotate_justify,
                                          params->annotate_x,
                                          params->annotate_y);
   result += SetGains(camera, params->analog_gain, params->digital_gain);

   if (params->settings)
   {
      MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T change_event_request =
      {
         {MMAL_PARAMETER_CHANGE_EVENT_REQUEST, sizeof(MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T)},
         MMAL_PARAMETER_CAMERA_SETTINGS, 1
      };

      MMAL_STATUS_T status = mmal_port_parameter_set(camera->control, &change_event_request.hdr);
      if ( status != MMAL_SUCCESS )
      {
         p_err_logger->error("No camera settings events");
      }

      result += status;
   }

   return result;
}


int GetRational(MMAL_COMPONENT_T *camera, uint32_t id, int *num, int *den)
{
	if( !camera )
		return 1;
	if( mmal_param_names.find(id) == mmal_param_names.end() )
		return 1;

	int ret = 0;
	MMAL_RATIONAL_T v_rat = {0, 100};
	ret = MmalstatusToMsg(mmal_port_parameter_get_rational(camera->control, id, &v_rat));
   
	if( ret == 0 )
	{
		(*num) = v_rat.num;
		(*den) = v_rat.den;
	}
	else
	{
		p_err_logger->error("Invalid param " + mmal_param_names.at(id) + " for rational value");
		ret = 1;
	}
	return ret;
}

int GetUint32(MMAL_COMPONENT_T *camera, uint32_t id, uint32_t *value)
{
	if( !camera )
		return 1;
	if( mmal_param_names.find(id) == mmal_param_names.end() )
		return 1;

	int ret = 0;
	uint32_t v_uint32 = 0;
	ret = MmalstatusToMsg(mmal_port_parameter_get_uint32(camera->control, id, &v_uint32));
   
	if( ret == 0 )
	{
		(*value) = v_uint32;
	}
	else
	{
		p_err_logger->error("Invalid param " + mmal_param_names.at(id) + " for uint32 value");
		ret = 1;
	}
	return ret;
}

int GetInt32(MMAL_COMPONENT_T *camera, uint32_t id, int32_t *value)
{
	if( !camera )
		return 1;
	if( mmal_param_names.find(id) == mmal_param_names.end() )
		return 1;

	int ret = 0;
	int32_t v_int32 = 0;
	ret = MmalstatusToMsg(mmal_port_parameter_get_int32(camera->control, id, &v_int32));
   
	if( ret == 0 )
	{
		(*value) = v_int32;
	}
	else
	{
		p_err_logger->error("Invalid param " + mmal_param_names.at(id) + " for int32 value");
		ret = 1;
	}
	return ret;
}

int GetBoolean(MMAL_COMPONENT_T *camera, uint32_t id, bool *value)
{
	if( !camera )
		return 1;
	if( mmal_param_names.find(id) == mmal_param_names.end() )
		return 1;

	int ret = 0;
	MMAL_BOOL_T v_bool = 0;
	ret = MmalstatusToMsg(mmal_port_parameter_get_boolean(camera->control, id, &v_bool));
   
	if( ret == 0 )
	{
		(*value) = v_bool;
	}
	else
	{
		p_err_logger->error("Invalid param " + mmal_param_names.at(id) + " for boolean value");
		ret = 1;
	}
	return ret;
}

template <class MMAL_TYPE>
int GetEnumedParam(MMAL_COMPONENT_T *camera, MMAL_TYPE* param)
{
	if (!camera)
		return 1;
	param->hdr.id = MmalCamParamTrait<MMAL_TYPE>::id;
	param->hdr.size = sizeof(MMAL_TYPE);
	
	
	MMAL_STATUS_T get_ret = mmal_port_parameter_get(camera->control, &param->hdr);
	
	if( get_ret != 0)
	{
		p_err_logger->error(std::string("Unable to get ") + MmalCamParamTrait<MMAL_TYPE>::name + "setting");
	}
	return MmalstatusToMsg(get_ret);
}

template <typename MMAL_TYPE>
int GetValuedParam(MMAL_COMPONENT_T *camera, MMAL_TYPE* param)
{
	if (!camera)
		return 1;
	typename MmalEnumBase<MMAL_TYPE>::basetype mmal_parameter;
	
	int ret = GetEnumedParam(camera, &mmal_parameter);
	
	if( ret )
		return ret;
	(*param) = mmal_parameter.value;
	
	return 0;
}

/**
 * Adjust the saturation level for images
 * @param camera Pointer to camera component
 * @param saturation Value to adjust, -100 to 100
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetSaturation(MMAL_COMPONENT_T *camera, int saturation)
{
   int ret = 0;

   if (!camera)
      return 1;

   if (saturation >= -100 && saturation <= 100)
   {
      MMAL_RATIONAL_T value = {saturation, 100};
      ret = MmalstatusToMsg(mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_SATURATION, value));
   }
   else
   {
      p_err_logger->error("Invalid saturation value");
      ret = 1;
   }

   return ret;
}
int GetSaturation( MMAL_COMPONENT_T *camera, int *saturation)
{
	int den;
	return GetRational(camera, MMAL_PARAMETER_SATURATION, saturation, &den);
}

/**
 * Set the sharpness of the image
 * @param camera Pointer to camera component
 * @param sharpness Sharpness adjustment -100 to 100
 */
int SetSharpness(MMAL_COMPONENT_T *camera, int sharpness)
{
   int ret = 0;

   if (!camera)
      return 1;

   if (sharpness >= -100 && sharpness <= 100)
   {
      MMAL_RATIONAL_T value = {sharpness, 100};
      ret = MmalstatusToMsg(mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_SHARPNESS, value));
   }
   else
   {
      p_err_logger->error("Invalid sharpness value");
      ret = 1;
   }

   return ret;
}
int GetSharpness(MMAL_COMPONENT_T *camera, int *sharpness)
{
	int den;
	return GetRational(camera, MMAL_PARAMETER_SHARPNESS, sharpness, &den);
}

/**
 * Set the contrast adjustment for the image
 * @param camera Pointer to camera component
 * @param contrast Contrast adjustment -100 to  100
 * @return
 */
int SetContrast(MMAL_COMPONENT_T *camera, int contrast)
{
   int ret = 0;

   if (!camera)
      return 1;

   if (contrast >= -100 && contrast <= 100)
   {
      MMAL_RATIONAL_T value = {contrast, 100};
      ret = MmalstatusToMsg(mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_CONTRAST, value));
   }
   else
   {
      p_err_logger->error("Invalid contrast value");
      ret = 1;
   }

   return ret;
}
int GetContrast(MMAL_COMPONENT_T *camera, int *contrast)
{
	int den;
	return GetRational(camera, MMAL_PARAMETER_CONTRAST, contrast, &den);
}



/**
 * Adjust the brightness level for images
 * @param camera Pointer to camera component
 * @param brightness Value to adjust, 0 to 100
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetBrightness(MMAL_COMPONENT_T *camera, int brightness)
{
   int ret = 0;

   if (!camera)
      return 1;

   if (brightness >= 0 && brightness <= 100)
   {
      MMAL_RATIONAL_T value = {brightness, 100};
      ret = MmalstatusToMsg(mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_BRIGHTNESS, value));
   }
   else
   {
      p_err_logger->error("Invalid brightness value");
      ret = 1;
   }

   return ret;
}
int GetBrightness(MMAL_COMPONENT_T *camera, int *brightness)
{
	int den;
	return GetRational(camera, MMAL_PARAMETER_BRIGHTNESS, brightness, &den);
}

/**
 * Adjust the ISO used for images
 * @param camera Pointer to camera component
 * @param ISO Value to set TODO :
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetISO(MMAL_COMPONENT_T *camera, int ISO)
{
   if (!camera)
      return 1;

   return MmalstatusToMsg(mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_ISO, ISO));
}
int GetISO(MMAL_COMPONENT_T *camera, int* ISO)
{
   if (!camera)
      return 1;
   return GetUint32(camera, MMAL_PARAMETER_ISO, (uint32_t*)ISO);
}

/**
 * Adjust the metering mode for images
 * @param camera Pointer to camera component
 * @param saturation Value from following
 *   - MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE,
 *   - MMAL_PARAM_EXPOSUREMETERINGMODE_SPOT,
 *   - MMAL_PARAM_EXPOSUREMETERINGMODE_BACKLIT,
 *   - MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetMeteringMode(MMAL_COMPONENT_T *camera, MMAL_PARAM_EXPOSUREMETERINGMODE_T m_mode )
{
   MMAL_PARAMETER_EXPOSUREMETERINGMODE_T meter_mode = {{MMAL_PARAMETER_EXP_METERING_MODE,sizeof(meter_mode)},
      m_mode
   };
   if (!camera)
      return 1;

   return MmalstatusToMsg(mmal_port_parameter_set(camera->control, &meter_mode.hdr));
}

int GetMeteringMode(MMAL_COMPONENT_T* camera, MMAL_PARAM_EXPOSUREMETERINGMODE_T* m_mode )
{
	return GetValuedParam(camera, m_mode);
}


/**
 * Set the video stabilisation flag. Only used in video mode
 * @param camera Pointer to camera component
 * @param saturation Flag 0 off 1 on
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetVideoStabilisation(MMAL_COMPONENT_T *camera, int vstabilisation)
{
   if (!camera)
      return 1;

   return MmalstatusToMsg(mmal_port_parameter_set_boolean(camera->control, MMAL_PARAMETER_VIDEO_STABILISATION, vstabilisation));
}
int GetVideoStabilisation(MMAL_COMPONENT_T *camera, int* vstabilisation)
{
   if (!camera)
      return 1;
   return GetBoolean(camera, MMAL_PARAMETER_VIDEO_STABILISATION, (bool*)vstabilisation);
}

/**
 * Adjust the exposure compensation for images (EV)
 * @param camera Pointer to camera component
 * @param exp_comp Value to adjust, -10 to +10
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetExposureCompensation(MMAL_COMPONENT_T *camera, int exp_comp)
{
   if (!camera)
      return 1;

   return MmalstatusToMsg(mmal_port_parameter_set_int32(camera->control, MMAL_PARAMETER_EXPOSURE_COMP, exp_comp));
}
int GetExposureCompensation(MMAL_COMPONENT_T *camera, int * exp_comp)
{
   if (!camera)
      return 1;
   return GetInt32(camera, MMAL_PARAMETER_EXPOSURE_COMP, exp_comp);
}

/**
 * Set exposure mode for images
 * @param camera Pointer to camera component
 * @param mode Exposure mode to set from
 *   - MMAL_PARAM_EXPOSUREMODE_OFF,
 *   - MMAL_PARAM_EXPOSUREMODE_AUTO,
 *   - MMAL_PARAM_EXPOSUREMODE_NIGHT,
 *   - MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW,
 *   - MMAL_PARAM_EXPOSUREMODE_BACKLIGHT,
 *   - MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT,
 *   - MMAL_PARAM_EXPOSUREMODE_SPORTS,
 *   - MMAL_PARAM_EXPOSUREMODE_SNOW,
 *   - MMAL_PARAM_EXPOSUREMODE_BEACH,
 *   - MMAL_PARAM_EXPOSUREMODE_VERYLONG,
 *   - MMAL_PARAM_EXPOSUREMODE_FIXEDFPS,
 *   - MMAL_PARAM_EXPOSUREMODE_ANTISHAKE,
 *   - MMAL_PARAM_EXPOSUREMODE_FIREWORKS,
 *
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetExposureMode(MMAL_COMPONENT_T *camera, MMAL_PARAM_EXPOSUREMODE_T mode)
{
   MMAL_PARAMETER_EXPOSUREMODE_T exp_mode = {{MMAL_PARAMETER_EXPOSURE_MODE,sizeof(exp_mode)}, mode};

   if (!camera)
      return 1;

   return MmalstatusToMsg(mmal_port_parameter_set(camera->control, &exp_mode.hdr));
}
int GetExposureMode(MMAL_COMPONENT_T* camera, MMAL_PARAM_EXPOSUREMODE_T* e_mode )
{
	return GetValuedParam(camera, e_mode);
}


/**
 * Set flicker avoid mode for images
 * @param camera Pointer to camera component
 * @param mode Exposure mode to set from
 *   - MMAL_PARAM_FLICKERAVOID_OFF,
 *   - MMAL_PARAM_FLICKERAVOID_AUTO,
 *   - MMAL_PARAM_FLICKERAVOID_50HZ,
 *   - MMAL_PARAM_FLICKERAVOID_60HZ,
 *
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetFlickerAvoidMode(MMAL_COMPONENT_T *camera, MMAL_PARAM_FLICKERAVOID_T mode)
{
   MMAL_PARAMETER_FLICKERAVOID_T fl_mode = {{MMAL_PARAMETER_FLICKER_AVOID,sizeof(fl_mode)}, mode};

   if (!camera)
      return 1;

   return MmalstatusToMsg(mmal_port_parameter_set(camera->control, &fl_mode.hdr));
}
int GetFlickerAvoidMode(MMAL_COMPONENT_T* camera, MMAL_PARAM_FLICKERAVOID_T* flk_mode )
{
	return GetValuedParam(camera, flk_mode);
}

/**
 * Set the aWB (auto white balance) mode for images
 * @param camera Pointer to camera component
 * @param awb_mode Value to set from
 *   - MMAL_PARAM_AWBMODE_OFF,
 *   - MMAL_PARAM_AWBMODE_AUTO,
 *   - MMAL_PARAM_AWBMODE_SUNLIGHT,
 *   - MMAL_PARAM_AWBMODE_CLOUDY,
 *   - MMAL_PARAM_AWBMODE_SHADE,
 *   - MMAL_PARAM_AWBMODE_TUNGSTEN,
 *   - MMAL_PARAM_AWBMODE_FLUORESCENT,
 *   - MMAL_PARAM_AWBMODE_INCANDESCENT,
 *   - MMAL_PARAM_AWBMODE_FLASH,
 *   - MMAL_PARAM_AWBMODE_HORIZON,
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetAwbMode(MMAL_COMPONENT_T *camera, MMAL_PARAM_AWBMODE_T awb_mode)
{
   MMAL_PARAMETER_AWBMODE_T param = {{MMAL_PARAMETER_AWB_MODE,sizeof(param)}, awb_mode};

   if (!camera)
      return 1;

   return MmalstatusToMsg(mmal_port_parameter_set(camera->control, &param.hdr));
}
int GetAwbMode(MMAL_COMPONENT_T* camera, MMAL_PARAM_AWBMODE_T* awb_mode )
{
	return GetValuedParam(camera, awb_mode);
}

int SetAwbGains(MMAL_COMPONENT_T *camera, float r_gain, float b_gain)
{
   MMAL_PARAMETER_AWB_GAINS_T param = {{MMAL_PARAMETER_CUSTOM_AWB_GAINS,sizeof(param)}, {0,0}, {0,0}};

   if (!camera)
      return 1;

   if (!r_gain || !b_gain)
      return 0;

   param.r_gain.num = (unsigned int)(r_gain * 65536);
   param.b_gain.num = (unsigned int)(b_gain * 65536);
   param.r_gain.den = param.b_gain.den = 65536;
   return MmalstatusToMsg(mmal_port_parameter_set(camera->control, &param.hdr));
}
int GetAwbGains(MMAL_COMPONENT_T *camera, float *r_gain, float *b_gain)
{
	if (!camera)
		return 1;
   
	MMAL_PARAMETER_AWB_GAINS_T awbgain = {
		{
			MMAL_PARAMETER_CUSTOM_AWB_GAINS,
			sizeof(awbgain)
		}, 
		{0,65536}, {0,65536}
	};
	
	MMAL_STATUS_T get_ret = mmal_port_parameter_get(camera->control, &awbgain.hdr);
	if( get_ret == 0)
	{
		*r_gain = (float)awbgain.r_gain.num / (float)awbgain.r_gain.den;
		*b_gain = (float)awbgain.b_gain.num / (float)awbgain.b_gain.den;
	}
	else
	{
		*r_gain = *b_gain = -1;
		p_err_logger->error("Unable to get color effect settings");
	}
	return MmalstatusToMsg(get_ret);
}

/**
 * Set the image effect for the images
 * @param camera Pointer to camera component
 * @param imageFX Value from
 *   - MMAL_PARAM_IMAGEFX_NONE,
 *   - MMAL_PARAM_IMAGEFX_NEGATIVE,
 *   - MMAL_PARAM_IMAGEFX_SOLARIZE,
 *   - MMAL_PARAM_IMAGEFX_POSTERIZE,
 *   - MMAL_PARAM_IMAGEFX_WHITEBOARD,
 *   - MMAL_PARAM_IMAGEFX_BLACKBOARD,
 *   - MMAL_PARAM_IMAGEFX_SKETCH,
 *   - MMAL_PARAM_IMAGEFX_DENOISE,
 *   - MMAL_PARAM_IMAGEFX_EMBOSS,
 *   - MMAL_PARAM_IMAGEFX_OILPAINT,
 *   - MMAL_PARAM_IMAGEFX_HATCH,
 *   - MMAL_PARAM_IMAGEFX_GPEN,
 *   - MMAL_PARAM_IMAGEFX_PASTEL,
 *   - MMAL_PARAM_IMAGEFX_WATERCOLOUR,
 *   - MMAL_PARAM_IMAGEFX_FILM,
 *   - MMAL_PARAM_IMAGEFX_BLUR,
 *   - MMAL_PARAM_IMAGEFX_SATURATION,
 *   - MMAL_PARAM_IMAGEFX_COLOURSWAP,
 *   - MMAL_PARAM_IMAGEFX_WASHEDOUT,
 *   - MMAL_PARAM_IMAGEFX_POSTERISE,
 *   - MMAL_PARAM_IMAGEFX_COLOURPOINT,
 *   - MMAL_PARAM_IMAGEFX_COLOURBALANCE,
 *   - MMAL_PARAM_IMAGEFX_CARTOON,
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetImageFX(MMAL_COMPONENT_T *camera, MMAL_PARAM_IMAGEFX_T imageFX)
{
   MMAL_PARAMETER_IMAGEFX_T imgFX = {{MMAL_PARAMETER_IMAGE_EFFECT,sizeof(imgFX)}, imageFX};

   if (!camera)
      return 1;

   return MmalstatusToMsg(mmal_port_parameter_set(camera->control, &imgFX.hdr));
}
int GetImageFX(MMAL_COMPONENT_T* camera, MMAL_PARAM_IMAGEFX_T* imageFX )
{
	return GetValuedParam(camera, imageFX);
}


/**
 * Set the colour effect  for images (Set UV component)
 * @param camera Pointer to camera component
 * @param colourFX  Contains enable state and U and V numbers to set (e.g. 128,128 = Black and white)
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetColourFX(MMAL_COMPONENT_T *camera, const MMAL_PARAMETER_COLOURFX_T *colourFX)
{
   MMAL_PARAMETER_COLOURFX_T colfx = {{MMAL_PARAMETER_COLOUR_EFFECT,sizeof(colfx)}, 0, 0, 0};

   if (!camera)
      return 1;

   colfx.enable = colourFX->enable;
   colfx.u = colourFX->u;
   colfx.v = colourFX->v;

   return MmalstatusToMsg(mmal_port_parameter_set(camera->control, &colfx.hdr));
}
int GetColourFX(MMAL_COMPONENT_T* camera, MMAL_PARAMETER_COLOURFX_T* colourFX )
{
	if (!camera)
		return 1;
	MMAL_PARAMETER_COLOURFX_T colfx = 
	{
		{
			MMAL_PARAMETER_COLOUR_EFFECT,
			sizeof(colfx)
		},
		0, 0, 0
	};
	
	MMAL_STATUS_T get_ret = mmal_port_parameter_get(camera->control, &colfx.hdr);
	if( get_ret == 0)
	{
		colourFX->enable = colfx.enable;
		colourFX->u = colfx.u;
		colourFX->v = colfx.v;
	}
	else
	{
		p_err_logger->error("Unable to get color effect settings");
	}
	return MmalstatusToMsg(get_ret);
}

/**
 * Set the rotation of the image
 * @param camera Pointer to camera component
 * @param rotation Degree of rotation (any number, but will be converted to 0,90,180 or 270 only)
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetRotation(MMAL_COMPONENT_T *camera, int rotation)
{
   MMAL_STATUS_T ret;
   int my_rotation = ((rotation % 360 ) / 90) * 90;

   ret = mmal_port_parameter_set_int32(camera->output[0], MMAL_PARAMETER_ROTATION, my_rotation);
   mmal_port_parameter_set_int32(camera->output[1], MMAL_PARAMETER_ROTATION, my_rotation);
   mmal_port_parameter_set_int32(camera->output[2], MMAL_PARAMETER_ROTATION, my_rotation);

   return MmalstatusToMsg(ret);
}
int GetRotation(MMAL_COMPONENT_T *camera, unsigned int output_ch, int *rotation)
{
	if( !camera )
		return 1;
	if(output_ch > 2)
		return -1;
	
	int ret = MmalstatusToMsg(mmal_port_parameter_get_int32(camera->output[output_ch], MMAL_PARAMETER_ROTATION, rotation));
	if( ret != 0 )
	{
		ret = 1;
		*rotation = 0;
	}
	return ret;
}

/**
 * Set the flips state of the image
 * @param camera Pointer to camera component
 * @param hflip If true, horizontally flip the image
 * @param vflip If true, vertically flip the image
 *
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetFlips(MMAL_COMPONENT_T *camera, int hflip, int vflip)
{
   MMAL_PARAMETER_MIRROR_T mirror = {{MMAL_PARAMETER_MIRROR, sizeof(MMAL_PARAMETER_MIRROR_T)}, MMAL_PARAM_MIRROR_NONE};

   if (hflip && vflip)
      mirror.value = MMAL_PARAM_MIRROR_BOTH;
   else if (hflip)
      mirror.value = MMAL_PARAM_MIRROR_HORIZONTAL;
   else if (vflip)
      mirror.value = MMAL_PARAM_MIRROR_VERTICAL;

   mmal_port_parameter_set(camera->output[0], &mirror.hdr);
   mmal_port_parameter_set(camera->output[1], &mirror.hdr);
   return MmalstatusToMsg(mmal_port_parameter_set(camera->output[2], &mirror.hdr));
}
int GetFlips(MMAL_COMPONENT_T *camera, const unsigned int & output_ch,
			 int *hflip, int *vflip)
{
	if( !camera ) return 1;
	if( output_ch>2 ) return -1;
	
	MMAL_PARAMETER_MIRROR_T mirror = 
	{
	   { MMAL_PARAMETER_MIRROR, sizeof(MMAL_PARAMETER_MIRROR_T)},
	   MMAL_PARAM_MIRROR_NONE
	};

	int ret = MmalstatusToMsg(mmal_port_parameter_get(camera->output[output_ch], &mirror.hdr));
	if( ret != 0 )
	{
		ret = 1;
		*hflip = *vflip = 0;
	}
	else
	{
		if(mirror.value == MMAL_PARAM_MIRROR_NONE) {
			*hflip = *vflip = 0;
		} else if(mirror.value == MMAL_PARAM_MIRROR_BOTH) {
			*hflip = *vflip = 1;
		} else if( mirror.value == MMAL_PARAM_MIRROR_HORIZONTAL ) {
			*hflip = 1;
			*vflip = 0;
		} else if( mirror.value == MMAL_PARAM_MIRROR_VERTICAL ) {
			*hflip = 0;
			*vflip = 1;
		} else {
			p_err_logger->error("Unknown mirror settings");
		}
	}
	return ret;
}

/**
 * Set the ROI of the sensor to use for captures/preview
 * @param camera Pointer to camera component
 * @param rect   Normalised coordinates of ROI rectangle
 *
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetRoi(MMAL_COMPONENT_T *camera, FloatRect rect)
{
   MMAL_PARAMETER_INPUT_CROP_T crop = {{MMAL_PARAMETER_INPUT_CROP, sizeof(MMAL_PARAMETER_INPUT_CROP_T)},{0,0,0,0}};

   crop.rect.x = (65536 * rect.x);
   crop.rect.y = (65536 * rect.y);
   crop.rect.width = (65536 * rect.w);
   crop.rect.height = (65536 * rect.h);

   return MmalstatusToMsg(mmal_port_parameter_set(camera->control, &crop.hdr));
}
int GetRoi(MMAL_COMPONENT_T *camera, FloatRect *rect)
{
	if( !camera ) return 1;
	MMAL_PARAMETER_INPUT_CROP_T crop = {
		{MMAL_PARAMETER_INPUT_CROP, sizeof(MMAL_PARAMETER_INPUT_CROP_T)},
		{0,0,0,0}
	};

	int ret = MmalstatusToMsg(mmal_port_parameter_get(camera->control, &crop.hdr));
	if( ret != 0 )
	{
		ret = 1;
		rect->x = rect->y = rect->w = rect->h = 0;
	}
	else
	{
		rect->x = (double)crop.rect.x / 65536 ;
		rect->y = (double)crop.rect.y / 65536 ;
		rect->w = (double)crop.rect.width / 65536;
		rect->h = (double)crop.rect.height / 65536;
	}
	return ret;
}

/**
 * Zoom in and Zoom out by changing ROI
 * @param camera Pointer to camera component
 * @param zoom_command zoom command enum
 * @return 0 if successful, non-zero otherwise
 */
int ZoomInOut(MMAL_COMPONENT_T *camera, ZOOM_COMMAND_T zoom_command, FloatRect *roi)
{
   MMAL_PARAMETER_INPUT_CROP_T crop;
   crop.hdr.id = MMAL_PARAMETER_INPUT_CROP;
   crop.hdr.size = sizeof(crop);

   if (mmal_port_parameter_get(camera->control, &crop.hdr) != MMAL_SUCCESS)
   {
      p_err_logger->error("mmal_port_parameter_get(camera->control, &crop.hdr) failed, skip it");
      return 0;
   }

   if (zoom_command == ZOOM_COMMAND_T::ZOOM_IN)
   {
      if ((unsigned)crop.rect.width <= (zoom_full_16P16 + zoom_increment_16P16))
      {
         crop.rect.width = zoom_full_16P16;
         crop.rect.height = zoom_full_16P16;
      }
      else
      {
         crop.rect.width -= zoom_increment_16P16;
         crop.rect.height -= zoom_increment_16P16;
      }
   }
   else if (zoom_command == ZOOM_COMMAND_T::ZOOM_OUT)
   {
      unsigned int increased_size = crop.rect.width + zoom_increment_16P16;
      if (increased_size < (unsigned)crop.rect.width) //overflow
      {
         crop.rect.width = 65536;
         crop.rect.height = 65536;
      }
      else
      {
         crop.rect.width = increased_size;
         crop.rect.height = increased_size;
      }
   }

   if (zoom_command == ZOOM_COMMAND_T::ZOOM_RESET)
   {
      crop.rect.x = 0;
      crop.rect.y = 0;
      crop.rect.width = 65536;
      crop.rect.height = 65536;
   }
   else
   {
      unsigned int centered_top_coordinate = (65536 - crop.rect.width) / 2;
      crop.rect.x = centered_top_coordinate;
      crop.rect.y = centered_top_coordinate;
   }

   int ret = MmalstatusToMsg(mmal_port_parameter_set(camera->control, &crop.hdr));

   if (ret == 0)
   {
      roi->x = roi->y = (double)crop.rect.x/65536;
      roi->w = roi->h = (double)crop.rect.width/65536;
   }
   else
   {
      p_err_logger->error("Failed to set crop values, x/y: {}, w/h: {}", crop.rect.x, crop.rect.width);
      ret = 1;
   }

   return ret;
}

/**
 * Adjust the exposure time used for images
 * @param camera Pointer to camera component
 * @param shutter speed in microseconds
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetShutterSpeed(MMAL_COMPONENT_T *camera, int speed)
{
   if (!camera)
      return 1;

   return MmalstatusToMsg(mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_SHUTTER_SPEED, speed));
}
int GetShutterSpeed(MMAL_COMPONENT_T *camera, uint32_t *speed_ms)
{
	if (!camera)
		return 1;
	return GetUint32(camera, MMAL_PARAMETER_SHUTTER_SPEED, speed_ms);
}

/**
 * Adjust the Dynamic range compression level
 * @param camera Pointer to camera component
 * @param strength Strength of DRC to apply
 *        MMAL_PARAMETER_DRC_STRENGTH_OFF
 *        MMAL_PARAMETER_DRC_STRENGTH_LOW
 *        MMAL_PARAMETER_DRC_STRENGTH_MEDIUM
 *        MMAL_PARAMETER_DRC_STRENGTH_HIGH
 *
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetDrc(MMAL_COMPONENT_T *camera, MMAL_PARAMETER_DRC_STRENGTH_T strength)
{
	MMAL_PARAMETER_DRC_T drc = {{MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(MMAL_PARAMETER_DRC_T)}, strength};

	if (!camera)
		return 1;

	return MmalstatusToMsg(mmal_port_parameter_set(camera->control, &drc.hdr));
}
int GetDrc(MMAL_COMPONENT_T *camera, MMAL_PARAMETER_DRC_STRENGTH_T *strength)
{
	if (!camera) return 1;
	MMAL_PARAMETER_DRC_T drc = {
		{MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(MMAL_PARAMETER_DRC_T)},
		MMAL_PARAMETER_DRC_STRENGTH_OFF
	};
	
	int ret = MmalstatusToMsg(mmal_port_parameter_get(camera->control, &drc.hdr));
	if( ret != 0 )
	{
		ret = 1;
	}
	else
	{
		*strength = drc.strength;
	}
	return ret;
}

int SetStatsPass(MMAL_COMPONENT_T *camera, int stats_pass)
{
   if (!camera)
      return 1;

   return MmalstatusToMsg(mmal_port_parameter_set_boolean(camera->control, MMAL_PARAMETER_CAPTURE_STATS_PASS, stats_pass));
}
int GetStatsPass(MMAL_COMPONENT_T *camera, bool *stats_pass)
{
   if (!camera)
      return 1;
   return GetBoolean( camera, MMAL_PARAMETER_CAPTURE_STATS_PASS, stats_pass );
}


/**
 * Set the annotate data
 * @param camera Pointer to camera component
 * @param Bitmask of required annotation data. 0 for off.
 * @param If set, a pointer to text string to use instead of bitmask, max length 32 characters
 *
 * @return 0 if successful, non-zero if any parameters out of range
 */
int SetAnnotate(MMAL_COMPONENT_T *camera, const int settings, const char *string,
                                 const int text_size, const int text_colour, const int bg_colour,
                                 const unsigned int justify, const unsigned int x, const unsigned int y)
{
   //MMAL_PARAMETER_CAMERA_ANNOTATE_V4_T annotate =
   //{{MMAL_PARAMETER_ANNOTATE, sizeof(MMAL_PARAMETER_CAMERA_ANNOTATE_V4_T)}};
   MMAL_PARAMETER_CAMERA_ANNOTATE_V4_T annotate;
   annotate.hdr = {MMAL_PARAMETER_ANNOTATE, sizeof(MMAL_PARAMETER_CAMERA_ANNOTATE_V4_T)};

   if (settings)
   {
      time_t t = time(NULL);
      struct tm tm = *localtime(&t);
      char tmp[MMAL_CAMERA_ANNOTATE_MAX_TEXT_LEN_V4];
      int process_datetime = 1;

      annotate.enable = 1;

      if (settings & (ANNOTATE_APP_TEXT | ANNOTATE_USER_TEXT))
      {
         if ((settings & (ANNOTATE_TIME_TEXT | ANNOTATE_DATE_TEXT)) && strchr(string,'%') != NULL)
         {
            //string contains strftime parameter?
            strftime(annotate.text, MMAL_CAMERA_ANNOTATE_MAX_TEXT_LEN_V3, string, &tm );
            process_datetime = 0;
         }
         else
         {
            strncpy(annotate.text, string, MMAL_CAMERA_ANNOTATE_MAX_TEXT_LEN_V3);
         }
         annotate.text[MMAL_CAMERA_ANNOTATE_MAX_TEXT_LEN_V3-1] = '\0';
      }

      if (process_datetime && (settings & ANNOTATE_TIME_TEXT))
      {
         if(strlen(annotate.text))
         {
            strftime(tmp, 32, " %X", &tm );
         }
         else
         {
            strftime(tmp, 32, "%X", &tm );
         }
         strncat(annotate.text, tmp, MMAL_CAMERA_ANNOTATE_MAX_TEXT_LEN_V3 - strlen(annotate.text) - 1);
      }

      if (process_datetime && (settings & ANNOTATE_DATE_TEXT))
      {
         if(strlen(annotate.text))
         {
            strftime(tmp, 32, " %x", &tm );
         }
         else
         {
            strftime(tmp, 32, "%x", &tm );
         }
         strncat(annotate.text, tmp, MMAL_CAMERA_ANNOTATE_MAX_TEXT_LEN_V3 - strlen(annotate.text) - 1);
      }

      if (settings & ANNOTATE_SHUTTER_SETTINGS)
         annotate.show_shutter = MMAL_TRUE;

      if (settings & ANNOTATE_GAIN_SETTINGS)
         annotate.show_analog_gain = MMAL_TRUE;

      if (settings & ANNOTATE_LENS_SETTINGS)
         annotate.show_lens = MMAL_TRUE;

      if (settings & ANNOTATE_CAF_SETTINGS)
         annotate.show_caf = MMAL_TRUE;

      if (settings & ANNOTATE_MOTION_SETTINGS)
         annotate.show_motion = MMAL_TRUE;

      if (settings & ANNOTATE_FRAME_NUMBER)
         annotate.show_frame_num = MMAL_TRUE;

      if (settings & ANNOTATE_BLACK_BACKGROUND)
         annotate.enable_text_background = MMAL_TRUE;

      annotate.text_size = text_size;

      if (text_colour != -1)
      {
         annotate.custom_text_colour = MMAL_TRUE;
         annotate.custom_text_Y = text_colour&0xff;
         annotate.custom_text_U = (text_colour>>8)&0xff;
         annotate.custom_text_V = (text_colour>>16)&0xff;
      }
      else
         annotate.custom_text_colour = MMAL_FALSE;

      if (bg_colour != -1)
      {
         annotate.custom_background_colour = MMAL_TRUE;
         annotate.custom_background_Y = bg_colour&0xff;
         annotate.custom_background_U = (bg_colour>>8)&0xff;
         annotate.custom_background_V = (bg_colour>>16)&0xff;
      }
      else
         annotate.custom_background_colour = MMAL_FALSE;

      annotate.justify = justify;
      annotate.x_offset = x;
      annotate.y_offset = y;
   }
   else
      annotate.enable = 0;

   return MmalstatusToMsg(mmal_port_parameter_set(camera->control, &annotate.hdr));
}

int SetStereoMode(MMAL_PORT_T *port, const MMAL_PARAMETER_STEREOSCOPIC_MODE_T & stereo_mode)
{
   MMAL_PARAMETER_STEREOSCOPIC_MODE_T stereo = { {MMAL_PARAMETER_STEREOSCOPIC_MODE, sizeof(stereo)},
      MMAL_STEREOSCOPIC_MODE_NONE, MMAL_FALSE, MMAL_FALSE
   };
   if (stereo_mode.mode != MMAL_STEREOSCOPIC_MODE_NONE)
   {
      stereo.mode = stereo_mode.mode;
      stereo.decimate = stereo_mode.decimate;
      stereo.swap_eyes = stereo_mode.swap_eyes;
   }
   return MmalstatusToMsg(mmal_port_parameter_set(port, &stereo.hdr));
}
int GetStereoMode(MMAL_PORT_T *port, MMAL_PARAMETER_STEREOSCOPIC_MODE_T *stereo_mode)
{
	if( !port ) return 1;
	
	MMAL_PARAMETER_STEREOSCOPIC_MODE_T stereo = 
	{
		{MMAL_PARAMETER_STEREOSCOPIC_MODE, sizeof(stereo)},
		MMAL_STEREOSCOPIC_MODE_NONE, MMAL_FALSE, MMAL_FALSE
	};
	int ret = MmalstatusToMsg(mmal_port_parameter_get(port, &stereo.hdr));
	if( ret != 0 )
	{
		ret = 1;
	}
	else
	{
		stereo_mode->mode = stereo.mode;
		stereo_mode->decimate = stereo.decimate;
		stereo_mode->swap_eyes = stereo.swap_eyes;
	}
	return ret;
}

int SetGains(MMAL_COMPONENT_T *camera, float analog, float digital)
{
   MMAL_RATIONAL_T rational = {0,65536};
   MMAL_STATUS_T status;

   if (!camera)
      return 1;

   rational.num = (unsigned int)(analog * 65536);
   status = mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_ANALOG_GAIN, rational);
   if (status != MMAL_SUCCESS)
      return MmalstatusToMsg(status);

   rational.num = (unsigned int)(digital * 65536);
   status = mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_DIGITAL_GAIN, rational);
   return MmalstatusToMsg(status);
}
int GetGains(MMAL_COMPONENT_T *camera, float *analog, float *digital)
{
	if (!camera) return 1;

	int num, den, ret;
	ret = GetRational(camera, MMAL_PARAMETER_ANALOG_GAIN, &num, &den);
	if( ret ) return 1;
	*analog = (float)num / (float)den;
	
	ret = GetRational(camera, MMAL_PARAMETER_DIGITAL_GAIN, &num, &den);
	if( ret ) return 1;
	*digital = (float)num / (float)den;
	
	return 0;
}

/**
 * Asked GPU how much memory it has allocated
 *
 * @return amount of memory in MB
 */
/*static int raspicamcontrol_get_mem_gpu(void)
{
   char response[80] = "";
   int gpu_mem = 0;
   if (vc_gencmd(response, sizeof response, "get_mem gpu") == 0)
      vc_gencmd_number_property(response, "gpu", &gpu_mem);
   return gpu_mem;
}*/

/**
 * Ask GPU about its camera abilities
 * @param supported None-zero if software supports the camera
 * @param detected  None-zero if a camera has been detected
 */
/*static void raspicamcontrol_get_camera(int *supported, int *detected)
{
   char response[80] = "";
   if (vc_gencmd(response, sizeof response, "get_camera") == 0)
   {
      if (supported)
         vc_gencmd_number_property(response, "supported", supported);
      if (detected)
         vc_gencmd_number_property(response, "detected", detected);
   }
}*/

/**
 * Check to see if camera is supported, and we have allocated enough memory
 * Ask GPU about its camera abilities
 * @param supported None-zero if software supports the camera
 * @param detected  None-zero if a camera has been detected
 */
void raspicamcontrol_check_configuration(int min_gpu_mem)
{
	(void)min_gpu_mem;
//   int gpu_mem = raspicamcontrol_get_mem_gpu();
//   int supported = 0, detected = 0;
//   raspicamcontrol_get_camera(&supported, &detected);
//   if (!supported)
//      p_err_logger->error("Camera is not enabled in this build. Try running \"sudo raspi-config\" and ensure that \"camera\" has been enabled\n");
//   else if (gpu_mem < min_gpu_mem)
//      p_err_logger->error("Only {}M of gpu_mem is configured. Try running \"sudo raspi-config\" and ensure that \"memory_split\" has a value of {} or greater\n", gpu_mem, min_gpu_mem);
//   else if (!detected)
//      p_err_logger->error("Camera is not detected. Please check carefully the camera module is installed correctly\n");
//   else
//      p_err_logger->error("Failed to run camera app. Please check for firmware updates\n");
}


/** Default camera callback function
 * Handles the --settings
 * @param port
 * @param Callback data
 */
void DefaultCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	(void) port;
	p_err_logger->error( "Camera control callback  cmd=0x{:b}", buffer->cmd);

	if (buffer->cmd == MMAL_EVENT_PARAMETER_CHANGED)
	{
		MMAL_EVENT_PARAMETER_CHANGED_T *param = (MMAL_EVENT_PARAMETER_CHANGED_T *)buffer->data;
		switch(param->hdr.id)
		{
			case MMAL_PARAMETER_CAMERA_SETTINGS:
			{
				MMAL_PARAMETER_CAMERA_SETTINGS_T *settings = (MMAL_PARAMETER_CAMERA_SETTINGS_T*)param;
				p_err_logger->error("Exposure now {}, analog gain {}/{}, digital gain {}/{}",
								settings->exposure,
								settings->analog_gain.num, settings->analog_gain.den,
								settings->digital_gain.num, settings->digital_gain.den);
				p_err_logger->error("AWB R={}/{}, B={}/{}",
								settings->awb_red_gain.num, settings->awb_red_gain.den,
								settings->awb_blue_gain.num, settings->awb_blue_gain.den);
			}
			break;
		}
	}
	else if (buffer->cmd == MMAL_EVENT_ERROR)
	{
		p_err_logger->error("No data received from sensor. Check all connections, including the Sunny one on the camera board");
	}
	else
	{
		p_err_logger->error("Received unexpected camera control callback event, 0x%08x", buffer->cmd);
	}

	mmal_buffer_header_release(buffer);
}

/**
 * Convert a MMAL status return value to a simple boolean of success
 * ALso displays a fault if code is not success
 *
 * @param status The error code to convert
 * @return 0 if status is success, 1 otherwise
 */
static int MmalstatusToMsg(const MMAL_STATUS_T & status)
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
		return 0;
		p_err_logger->info( "Success" ); // DO not show success
	}
	try {
		p_err_logger->error( "MMAL error: " + map_mmal_status_to_string.at(status) );
	}
	catch(...) {
		p_err_logger->error("Unknown status error: {}", status);
			return 1;
	}
	
	return 1;
}


} // end of namespace RaspiCamControl
