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

#ifndef RASPICAMCONTROL_HPP_
#define RASPICAMCONTROL_HPP_

#include <unordered_map>
#include <string>
#include "mmal.h"

/* Various parameters
 *
 * Exposure Mode
 *          MMAL_PARAM_EXPOSUREMODE_OFF,
            MMAL_PARAM_EXPOSUREMODE_AUTO,
            MMAL_PARAM_EXPOSUREMODE_NIGHT,
            MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW,
            MMAL_PARAM_EXPOSUREMODE_BACKLIGHT,
            MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT,
            MMAL_PARAM_EXPOSUREMODE_SPORTS,
            MMAL_PARAM_EXPOSUREMODE_SNOW,
            MMAL_PARAM_EXPOSUREMODE_BEACH,
            MMAL_PARAM_EXPOSUREMODE_VERYLONG,
            MMAL_PARAM_EXPOSUREMODE_FIXEDFPS,
            MMAL_PARAM_EXPOSUREMODE_ANTISHAKE,
            MMAL_PARAM_EXPOSUREMODE_FIREWORKS,
 *
 * Flicker Avoid Mode
 *          MMAL_PARAM_FLICKERAVOID_OFF,
            MMAL_PARAM_FLICKERAVOID_AUTO,
            MMAL_PARAM_FLICKERAVOID_50HZ,
            MMAL_PARAM_FLICKERAVOID_60HZ,
 *
 * AWB Mode
 *          MMAL_PARAM_AWBMODE_OFF,
            MMAL_PARAM_AWBMODE_AUTO,
            MMAL_PARAM_AWBMODE_SUNLIGHT,
            MMAL_PARAM_AWBMODE_CLOUDY,
            MMAL_PARAM_AWBMODE_SHADE,
            MMAL_PARAM_AWBMODE_TUNGSTEN,
            MMAL_PARAM_AWBMODE_FLUORESCENT,
            MMAL_PARAM_AWBMODE_INCANDESCENT,
            MMAL_PARAM_AWBMODE_FLASH,
            MMAL_PARAM_AWBMODE_HORIZON,
 *
 * Image FX
            MMAL_PARAM_IMAGEFX_NONE,
            MMAL_PARAM_IMAGEFX_NEGATIVE,
            MMAL_PARAM_IMAGEFX_SOLARIZE,
            MMAL_PARAM_IMAGEFX_POSTERIZE,
            MMAL_PARAM_IMAGEFX_WHITEBOARD,
            MMAL_PARAM_IMAGEFX_BLACKBOARD,
            MMAL_PARAM_IMAGEFX_SKETCH,
            MMAL_PARAM_IMAGEFX_DENOISE,
            MMAL_PARAM_IMAGEFX_EMBOSS,
            MMAL_PARAM_IMAGEFX_OILPAINT,
            MMAL_PARAM_IMAGEFX_HATCH,
            MMAL_PARAM_IMAGEFX_GPEN,
            MMAL_PARAM_IMAGEFX_PASTEL,
            MMAL_PARAM_IMAGEFX_WATERCOLOUR,
            MMAL_PARAM_IMAGEFX_FILM,
            MMAL_PARAM_IMAGEFX_BLUR,
            MMAL_PARAM_IMAGEFX_SATURATION,
            MMAL_PARAM_IMAGEFX_COLOURSWAP,
            MMAL_PARAM_IMAGEFX_WASHEDOUT,
            MMAL_PARAM_IMAGEFX_POSTERISE,
            MMAL_PARAM_IMAGEFX_COLOURPOINT,
            MMAL_PARAM_IMAGEFX_COLOURBALANCE,
            MMAL_PARAM_IMAGEFX_CARTOON,

 */

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


/*Replaced typedef struct mmal_param_colourfx_s
{
   int enable;       /// Turn colourFX on or off
   int u,v;          /// U and V to use
} MMAL_PARAMETER_COLOURFX_T;*/

/*undefined typedef struct mmal_param_thumbnail_config_s
{
   int enable;
   int width,height;
   int quality;
} MMAL_PARAM_THUMBNAIL_CONFIG_T;*/




namespace RaspiCamControl
{
	enum class ZOOM_COMMAND_T
	{
		ZOOM_IN, 
		ZOOM_OUT, 
		ZOOM_RESET
	} ;
		
	struct FloatRect
	{
		double x;
		double y;
		double w;
		double h;
	};

	struct CamConfig
	{
		int sharpness;             /// -100 to 100
		int contrast;              /// -100 to 100
		int brightness;            ///  0 to 100
		int saturation;            ///  -100 to 100
		int ISO;                   ///  TODO : what range?
		int videoStabilisation;    /// 0 or 1 (false or true)
		int exposureCompensation;  /// -10 to +10 ?
		MMAL_PARAM_EXPOSUREMODE_T exposureMode;
		MMAL_PARAM_EXPOSUREMETERINGMODE_T exposureMeterMode;
		MMAL_PARAM_AWBMODE_T awbMode;
		MMAL_PARAM_IMAGEFX_T imageEffect;
		MMAL_PARAMETER_IMAGEFX_PARAMETERS_T imageEffectsParameters;
		MMAL_PARAMETER_COLOURFX_T colourEffects;
		MMAL_PARAM_FLICKERAVOID_T flickerAvoidMode;
		int rotation;              /// 0-359
		int hflip;                 /// 0 or 1
		int vflip;                 /// 0 or 1
		FloatRect  roi;   /// region of interest to use on the sensor. Normalised [0,1] values in the rect
		int shutter_speed;         /// 0 = auto, otherwise the shutter speed in ms
		float awb_gains_r;         /// AWB red gain
		float awb_gains_b;         /// AWB blue gain
		MMAL_PARAMETER_DRC_STRENGTH_T drc_level;  // Strength of Dynamic Range compression to apply
		MMAL_BOOL_T stats_pass;    /// Stills capture statistics pass on/off
		int enable_annotate;       /// Flag to enable the annotate, 0 = disabled, otherwise a bitmask of what needs to be displayed
		char annotate_string[MMAL_CAMERA_ANNOTATE_MAX_TEXT_LEN_V2]; /// String to use for annotate - overrides certain bitmask settings
		int annotate_text_size;    // Text size for annotation
		int annotate_text_colour;  // Text colour for annotation
		int annotate_bg_colour;    // Background colour for annotation
		unsigned int annotate_justify;
		unsigned int annotate_x;
		unsigned int annotate_y;

		MMAL_PARAMETER_STEREOSCOPIC_MODE_T stereo_mode;
		float analog_gain;         // Analog gain
		float digital_gain;        // Digital gain

		int settings;
	  public:
		void SetDefault();
	};
	
	/// Structure to cross reference exposure strings against the MMAL parameter equivalent
	static std::unordered_map<std::string, MMAL_PARAM_EXPOSUREMODE_T> exposure_map =
	{
	{"off",           MMAL_PARAM_EXPOSUREMODE_OFF},
	{"auto",          MMAL_PARAM_EXPOSUREMODE_AUTO},
	{"night",         MMAL_PARAM_EXPOSUREMODE_NIGHT},
	{"nightpreview",  MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW},
	{"backlight",     MMAL_PARAM_EXPOSUREMODE_BACKLIGHT},
	{"spotlight",     MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT},
	{"sports",        MMAL_PARAM_EXPOSUREMODE_SPORTS},
	{"snow",          MMAL_PARAM_EXPOSUREMODE_SNOW},
	{"beach",         MMAL_PARAM_EXPOSUREMODE_BEACH},
	{"verylong",      MMAL_PARAM_EXPOSUREMODE_VERYLONG},
	{"fixedfps",      MMAL_PARAM_EXPOSUREMODE_FIXEDFPS},
	{"antishake",     MMAL_PARAM_EXPOSUREMODE_ANTISHAKE},
	{"fireworks",     MMAL_PARAM_EXPOSUREMODE_FIREWORKS}
	};


	/// Structure to cross reference flicker avoid strings against the MMAL parameter equivalent

	static std::unordered_map<std::string, MMAL_PARAM_FLICKERAVOID_T> flicker_avoid_map =
	{
	{"off",           MMAL_PARAM_FLICKERAVOID_OFF},
	{"auto",          MMAL_PARAM_FLICKERAVOID_AUTO},
	{"50hz",          MMAL_PARAM_FLICKERAVOID_50HZ},
	{"60hz",          MMAL_PARAM_FLICKERAVOID_60HZ}
	};

	/// Structure to cross reference awb strings against the MMAL parameter equivalent
	static std::unordered_map<std::string, MMAL_PARAM_AWBMODE_T> awb_map =
	{
	{"off",           MMAL_PARAM_AWBMODE_OFF},
	{"auto",          MMAL_PARAM_AWBMODE_AUTO},
	{"sun",           MMAL_PARAM_AWBMODE_SUNLIGHT},
	{"cloud",         MMAL_PARAM_AWBMODE_CLOUDY},
	{"shade",         MMAL_PARAM_AWBMODE_SHADE},
	{"tungsten",      MMAL_PARAM_AWBMODE_TUNGSTEN},
	{"fluorescent",   MMAL_PARAM_AWBMODE_FLUORESCENT},
	{"incandescent",  MMAL_PARAM_AWBMODE_INCANDESCENT},
	{"flash",         MMAL_PARAM_AWBMODE_FLASH},
	{"horizon",       MMAL_PARAM_AWBMODE_HORIZON}
	};

	/// Structure to cross reference image effect against the MMAL parameter equivalent
	static std::unordered_map<std::string, MMAL_PARAM_IMAGEFX_T> imagefx_map =
	{
	{"none",          MMAL_PARAM_IMAGEFX_NONE},
	{"negative",      MMAL_PARAM_IMAGEFX_NEGATIVE},
	{"solarise",      MMAL_PARAM_IMAGEFX_SOLARIZE},
	{"sketch",        MMAL_PARAM_IMAGEFX_SKETCH},
	{"denoise",       MMAL_PARAM_IMAGEFX_DENOISE},
	{"emboss",        MMAL_PARAM_IMAGEFX_EMBOSS},
	{"oilpaint",      MMAL_PARAM_IMAGEFX_OILPAINT},
	{"hatch",         MMAL_PARAM_IMAGEFX_HATCH},
	{"gpen",          MMAL_PARAM_IMAGEFX_GPEN},
	{"pastel",        MMAL_PARAM_IMAGEFX_PASTEL},
	{"watercolour",   MMAL_PARAM_IMAGEFX_WATERCOLOUR},
	{"film",          MMAL_PARAM_IMAGEFX_FILM},
	{"blur",          MMAL_PARAM_IMAGEFX_BLUR},
	{"saturation",    MMAL_PARAM_IMAGEFX_SATURATION},
	{"colourswap",    MMAL_PARAM_IMAGEFX_COLOURSWAP},
	{"washedout",     MMAL_PARAM_IMAGEFX_WASHEDOUT},
	{"posterise",     MMAL_PARAM_IMAGEFX_POSTERISE},
	{"colourpoint",   MMAL_PARAM_IMAGEFX_COLOURPOINT},
	{"colourbalance", MMAL_PARAM_IMAGEFX_COLOURBALANCE},
	{"cartoon",       MMAL_PARAM_IMAGEFX_CARTOON}
	};

	static std::unordered_map<std::string, MMAL_PARAM_EXPOSUREMETERINGMODE_T> metering_mode_map =
	{
	{"average",       MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE},
	{"spot",          MMAL_PARAM_EXPOSUREMETERINGMODE_SPOT},
	{"backlit",       MMAL_PARAM_EXPOSUREMETERINGMODE_BACKLIT},
	{"matrix",        MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX}
	};
	
	static std::unordered_map<std::string, MMAL_PARAMETER_DRC_STRENGTH_T> drc_mode_map =
	{
	{"off",           MMAL_PARAMETER_DRC_STRENGTH_OFF},
	{"low",           MMAL_PARAMETER_DRC_STRENGTH_LOW},
	{"med",           MMAL_PARAMETER_DRC_STRENGTH_MEDIUM},
	{"high",          MMAL_PARAMETER_DRC_STRENGTH_HIGH}
	};
	
	void BuildRevMap(); // Note: this must be called before the usage of parameter dump
	
	
	
	/** Default camera callback function
	* Handles the --settings
	* @param port
	* @param Callback data
	*/
	void DefaultCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
	
	void DisplayHelp();
	void DumpParameters(const CamConfig &params);
	
	int SetAllParameters(MMAL_COMPONENT_T *camera, const CamConfig *params);
	int raspicamcontrol_get_all_parameters(MMAL_COMPONENT_T *camera, CamConfig *params);

	void SetDefaults(CamConfig *params);

	void raspicamcontrol_check_configuration(int min_gpu_mem);

	// Individual setting functions
	int SetSaturation(MMAL_COMPONENT_T *camera, int saturation);
	int SetSharpness(MMAL_COMPONENT_T *camera, int sharpness);
	int SetContrast(MMAL_COMPONENT_T *camera, int contrast);
	int SetBrightness(MMAL_COMPONENT_T *camera, int brightness);
	int raspicamcontrol_set_ISO(MMAL_COMPONENT_T *camera, int ISO);
	int SetMeteringMode(MMAL_COMPONENT_T *camera, MMAL_PARAM_EXPOSUREMETERINGMODE_T mode);
	int SetVideoStabilisation(MMAL_COMPONENT_T *camera, int vstabilisation);
	int SetExposureCompensation(MMAL_COMPONENT_T *camera, int exp_comp);
	int SetExposureMode(MMAL_COMPONENT_T *camera, MMAL_PARAM_EXPOSUREMODE_T mode);
	int SetFlicker_avoidMode(MMAL_COMPONENT_T *camera, MMAL_PARAM_FLICKERAVOID_T mode);
	int SetAwbMode(MMAL_COMPONENT_T *camera, MMAL_PARAM_AWBMODE_T awb_mode);
	int SetAwbGains(MMAL_COMPONENT_T *camera, float r_gain, float b_gain);
	int SetImageFX(MMAL_COMPONENT_T *camera, MMAL_PARAM_IMAGEFX_T imageFX);
	int SetColourFX(MMAL_COMPONENT_T *camera, const MMAL_PARAMETER_COLOURFX_T *colourFX);
	int SetRotation(MMAL_COMPONENT_T *camera, int rotation);
	int SetFlips(MMAL_COMPONENT_T *camera, int hflip, int vflip);
	int raspicamcontrol_set_ROI(MMAL_COMPONENT_T *camera, FloatRect rect);
	int ZoomInOut(MMAL_COMPONENT_T *camera, ZOOM_COMMAND_T zoom_command, FloatRect *roi);
	int SetShutterSpeed(MMAL_COMPONENT_T *camera, int speed_ms);
	int raspicamcontrol_set_DRC(MMAL_COMPONENT_T *camera, MMAL_PARAMETER_DRC_STRENGTH_T strength);
	int SetStatsPass(MMAL_COMPONENT_T *camera, int stats_pass);
	int SetAnnotate(MMAL_COMPONENT_T *camera, const int bitmask, const char *string,
									const int text_size, const int text_colour, const int bg_colour,
									const unsigned int justify, const unsigned int x, const unsigned int y);
	int SetStereoMode(MMAL_PORT_T *port, const MMAL_PARAMETER_STEREOSCOPIC_MODE_T & stereo_mode);
	int SetGains(MMAL_COMPONENT_T *camera, float analog, float digital);
	
	
} // end of namespace RaspiCamControl

// undefined
//void raspicamcontrol_check_configuration(int min_gpu_mem);

// TBD
//int raspicamcontrol_cycle_test(MMAL_COMPONENT_T *camera);

/* Undefined
//Individual getting functions
int raspicamcontrol_get_saturation(MMAL_COMPONENT_T *camera);
int raspicamcontrol_get_sharpness(MMAL_COMPONENT_T *camera);
int raspicamcontrol_get_contrast(MMAL_COMPONENT_T *camera);
int raspicamcontrol_get_brightness(MMAL_COMPONENT_T *camera);
int raspicamcontrol_get_ISO(MMAL_COMPONENT_T *camera);
MMAL_PARAM_EXPOSUREMETERINGMODE_T raspicamcontrol_get_metering_mode(MMAL_COMPONENT_T *camera);
int raspicamcontrol_get_video_stabilisation(MMAL_COMPONENT_T *camera);
int raspicamcontrol_get_exposure_compensation(MMAL_COMPONENT_T *camera);
MMAL_PARAM_THUMBNAIL_CONFIG_T raspicamcontrol_get_thumbnail_parameters(MMAL_COMPONENT_T *camera);
MMAL_PARAM_EXPOSUREMODE_T raspicamcontrol_get_exposure_mode(MMAL_COMPONENT_T *camera);
MMAL_PARAM_FLICKERAVOID_T raspicamcontrol_get_flicker_avoid_mode(MMAL_COMPONENT_T *camera);
MMAL_PARAM_AWBMODE_T raspicamcontrol_get_awb_mode(MMAL_COMPONENT_T *camera);
MMAL_PARAM_IMAGEFX_T raspicamcontrol_get_imageFX(MMAL_COMPONENT_T *camera);
MMAL_PARAMETER_COLOURFX_T raspicamcontrol_get_colourFX(MMAL_COMPONENT_T *camera);*/

/** Default camera callback function
  */



#endif /* RASPICAMCONTROL_H_ */
