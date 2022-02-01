#include <iostream>
#include "raspi_encamode.hpp"
#include "raspi_encamode_configs.hpp"
#include "raspicam_camcontrol.hpp"
#include "cxxopts.hpp"

// Prototypes
void default_signal_handler(int signal_number);


/**
 * Display usage information for the application to stdout
 *
 */
static void additional_help_options()
{
	std::cout <<  "Image parameter commands\n\n" ;
	// Profile options
	std::cout <<  "H264 Profile options :\n\t";
	for( auto local_it = profile_map.begin(); local_it!= profile_map.end(); ++local_it ) {
		std::cout << ", " << local_it->first;
	}
	
	// Level options
	std::cout <<  "\n\nH264 Level options :\n\t";
	for( auto local_it = level_map.begin(); local_it!= level_map.end(); ++local_it ) {
		std::cout << ", " << local_it->first;
	}
	
	// Intra refresh options
	std::cout <<  "\n\nH264 Intra refresh options :\n\t";
	for( auto local_it = intra_refresh_map.begin(); local_it!= intra_refresh_map.end(); ++local_it ) {
		std::cout << ", " << local_it->first;
	}

	// Raw output format options
	std::cout <<  "\n\nRaw output format options :\n\t";
	for( auto local_it = raw_output_fmt_map.begin(); local_it!= raw_output_fmt_map.end(); ++local_it ) {
		std::cout << ", " << local_it->first;
	}

	std::cout << "\n\n"
			  << "Raspivid allows output to a remote IPv4 host e.g. -o tcp://192.168.1.2:1234"
			  << "or -o udp://192.168.1.2:1234\n"
			  << "To listen on a TCP port (IPv4) and wait for an incoming connection use the -l option\n"
			  << "e.g. raspivid -l -o tcp://0.0.0.0:3333 -> bind to all network interfaces,\n"
			  << "raspivid -l -o tcp://192.168.1.1:3333 -> bind to a certain local IPv4 port\n";
	std::cout << std::endl;
	return;
}

/**
 * Parse the incoming command line and put resulting parameters in to the state
 *
 * @param argc Number of arguments in command line
 * @param argv Array of pointers to strings from command line
 * @param state Pointer to state structure to assign any discovered parameters to
 * @return false if failed for some reason, 0 otherwise
 */
static bool parse_cmdline(int argc, const char **argv, RaspiEncamodeConfig *conf, int * demoMode, int * demoInterval)
{
	cxxopts::Options opt("RaspiCamEncode test", "Display camera output to display, and optionally saves an H264 capture at requested bitrate");
	
	opt.add_options()
		("h,help", "Help")
		("pts", "debug output serial port", cxxopts::value<std::string>()) 
		("b,bitrate",  
		 "Set bitrate. Use bits per second (e.g. 10MBits/s would be -b 10000000)",
		 cxxopts::value<int>() )  //1
		("t,timeout",
		 "Time (in ms) to capture for. If not specified, set to 5s. Zero to disable", 
		 cxxopts::value<int>() )  //1
		("d,demo",
		 "Run a demo mode (cycle through range of camera options, no capture)", 
		 cxxopts::value<int>() )  //1
		("framerate", 
		 "Specify the frames per second to record", 
		 cxxopts::value<int>() )  //1
		("e,penc", 
		 "Display preview image *after* encoding (shows compression artifacts)", 
		 cxxopts::value<bool>() )  //0
		("g,intra",
		 "Specify the intra refresh period (key frame rate/GoP size). Zero to produce an initial I-frame and then just P-frames.",
		 cxxopts::value<int>()  )  //1
		("profile",
		 "Specify H264 profile to use for encoding",
		 cxxopts::value<std::string>() )  //1
		("timed_on",
		 "Cycle between capture and pause. -cycle on,off where on is record time and off is pause time in ms", 
		 cxxopts::value<int>()  )  //0
		("timed_off",
		 "Cycle between capture and pause. -cycle on,off where on is record time and off is pause time in ms", 
		 cxxopts::value<int>()  )  //0
		("s,signal", 
		 "Cycle between capture and pause on Signal",
		 cxxopts::value<int>()  ) // 0
		("k,keypress",
		 "Cycle between capture and pause on ENTER", 
		 cxxopts::value<int>()  )  //0
		("i,initial",
		 "Initial state. Use 'record' or 'pause'. Default 'record'",
		 cxxopts::value<std::string>()->default_value("record") )
		("qp",
		 "Quantisation parameter. Use approximately 10-40. Default 0 (off)",
		 cxxopts::value<int>()->default_value("0") ) // 0
		 ("inline",
		  "Insert inline headers (SPS, PPS) to stream", 
		  cxxopts::value<bool>() ) //0
		("segment",
		 "Segment output file in to multiple files at specified interval <ms>", 
		 cxxopts::value<int>()  ) //1
		("wrap", 
		 "In segment mode, wrap any numbered filename back to 1 when reach number", 
		 cxxopts::value<int>() )  //1
		("start", 
		 "In segment mode, start with specified segment number",
		 cxxopts::value<int>() ) //1
		("split",
		 "In wait mode, create new output file for each start event",
		 cxxopts::value<bool>() ) //0
		("c,circular", 
		 "Run encoded data through circular buffer until triggered then save", 
		 cxxopts::value<bool>() ) //0
		("x,vectors", 
		 "Output filename <filename> for inline motion vectors",
		 cxxopts::value<std::string>()  ) //1 
		("irefresh",
		 "Set intra refresh type", 
		 cxxopts::value<std::string>()  ) //1 1//1
		("flush",  
		 "Flush buffers in order to decrease latency",
		 cxxopts::value<bool>()   ) //0
		("save-pts",
		 "Save Timestamps to file for mkvmerge",
		 cxxopts::value<std::string>()  ) //1
		("raw-pts",
		 "Save Raw Timestamps to file",
		 cxxopts::value<std::string>()  ) 
		("codec",
		 "Specify the codec to use - H264 (default) or MJPEG", cxxopts::value<std::string>()->default_value("H264")  ) //1
		("level",
		 "Specify H264 level to use for encoding",
		 cxxopts::value<int>()  ) //1
		("r,raw", 
		 "Output filename <filename> for raw video",
		 cxxopts::value<std::string>()  ) //1
		("raw-format", 
		 "Specify output format for raw video. Default is yuv", 
		 cxxopts::value<std::string>())   //1
		("l,listen", 
		 "Listen on a TCP socket", 
		 cxxopts::value<bool>()  )  //0
		("spstimings", 
		 "Add in h.264 sps timings", 
		 cxxopts::value<bool>() )  //0
		("slices", 
		 "Horizontal slices per frame. Default 1 (off)", 
		 cxxopts::value<int>()->default_value("1") )  //1
		;
		
	opt.add_options( "common", {
		{ "width",  "Set image width <size>", cxxopts::value<int>() },
		{ "height", "Set image height <size>", cxxopts::value<int>() },
		{ "o,output", 
			"Output filename <filename> (to write to stdout, use '-o -'). If not specified, no file is saved", 
			cxxopts::value<std::string>() 		},
		{ "v,verbose",
			"Output verbose information during run", 
			cxxopts::value<bool>() 				},
		{ "camselect", 
			"Select camera <number>. Default 0", 
			cxxopts::value<int>() 				},
		{ "mode",  
			"Force sensor mode. 0=auto. See docs for other modes available",
			cxxopts::value<int>() 				},
		{ "gpsdexif", 
			"Unavailable for this library", 
			cxxopts::value<bool>() 				}  }
	);
	
	opt.add_options( "camera", {
		{"sharpness",  "Set image sharpness (-100 to 100)", cxxopts::value<int>() },
		{"contrast",   "Set image contrast (-100 to 100)",  cxxopts::value<int>() }, 
		{"brightness", "Set image brightness (0 to 100)",  cxxopts::value<int>() },
		{"saturation", "Set image saturation (-100 to 100)", cxxopts::value<int>() },
		{"ISO",       "Set capture ISO",  cxxopts::value<int>() },
		{"vstab",      "Turn on video stabilisation", cxxopts::value<bool>() },
		{"ev",         "Set EV compensation - steps of 1/6 stop",  cxxopts::value<int>() },
		{"exposure",   "Set exposure mode (see Notes)", cxxopts::value<std::string>() },
		{"flicker",   "Set flicker avoid mode (see Notes)", cxxopts::value<std::string>() },
		{"awb",       "Set AWB mode (see Notes)", cxxopts::value<std::string>() },
		{"imgfx",     "Set image effect (see Notes)", cxxopts::value<std::string>() },
		{"colfx",     "Set colour effect (U:V)",  cxxopts::value<std::string>() },
		{"metering",   "Set metering mode (see Notes)", cxxopts::value<std::string>() },
		{"rotation",  "Set image rotation (0-359)", cxxopts::value<int>() },
		{"hflip",      "Set horizontal flip", cxxopts::value<bool>() },
		{"vflip",      "Set vertical flip", cxxopts::value<bool>() },
		{"roi",       "Set region of interest (x,y,w,d as normalised coordinates [0.0-1.0])", cxxopts::value<std::string>() },
		{"shutter",    "Set shutter speed in microseconds", cxxopts::value<int>() },
		{"awbgains", "Set AWB gains - AWB mode must be off", cxxopts::value<float>() },
		{"drc",       "Set DRC Level (see Notes)", cxxopts::value<int>() },
		{"stats",      "Force recomputation of statistics on stills capture pass", cxxopts::value<bool>() },
		{"a,annotate",    "Enable/Set annotate flags or text", cxxopts::value<std::string>() },
		{"stereo",     "Select stereoscopic mode", cxxopts::value<int>() },
		{"decimate",  "Half width/height of stereo image", cxxopts::value<bool>() },
		{"3dswap",        "Swap camera order for stereoscopic", cxxopts::value<bool>() },
		{"annotateex", "Set extra annotation parameters (text size, text colour(hex YUV), bg colour(hex YUV), justify, x, y)", cxxopts::value<std::string>() },
		{"analoggain",  "Set the analog gain (floating point)", cxxopts::value<float>() },
		{"digitalgain", "Set the digital gain (floating point)", cxxopts::value<float>() },
		{"settings",   "Retrieve camera settings and write to stdout", cxxopts::value<bool>() } 
	} );
	
	opt.add_options( "preview", {
		{ "p,preview",    "Preview window settings <'x,y,w,h'>", cxxopts::value<std::string>() },
		{ "f,fullscreen", "Fullscreen preview mode", cxxopts::value<bool>() },
		{ "opacity",   "Preview window opacity (0-255)", cxxopts::value<int>()},
		{ "n,nopreview",  "Do not display a preview window", cxxopts::value<bool>()}
	} );
	
	cxxopts::ParseResult result;
	try
	{
		result = opt.parse(argc, argv);
	}
	catch(...)
	{
		std::cout << opt.help({"", "common","camera","preview"}) << std::endl;
		throw;
	}
	if(result.count("help"))
	{
		std::cout << opt.help({"", "common","camera","preview"}) << std::endl;
		additional_help_options();
		RaspiCamControl::DisplayHelp();
		return false;
	}
	
	// Parse the command line arguments.
	// We are looking for --<something> or -<abbreviation of something>
	if(result.count("bitrate")) {
		std::cout << opt.help() << std::endl;
		conf->bitrate = result["bitrate"].as<int>();
	}
	if(result.count("timeout")) {
		conf->timeout = result["timeout"].as<int>();
		std::cout << "Set timeout to: " << conf->timeout<< std::endl;
		// Ensure that if previously selected a waitMethod we don't overwrite it
		if (conf->timeout == 0 && conf->waitMethod == WaitMethod::NONE)
               conf->waitMethod = WaitMethod::FOREVER;
	}
	if(result.count("demo")) {
		std::cout << "Set to Demo mode" << std::endl;
		(*demoInterval) = result["demo"].as<int>();
		(*demoMode) = 1;
		if( (*demoInterval) == 0) (*demoInterval) = 250; // ms
	}
	if(result.count("framerate")) {
		conf->framerate = result["framerate"].as<int>();
		std::cout << "Set framerate to: " << conf->framerate << std::endl;
	}
	if(result.count("penc")) {
		std::cout << "Preview encoder" << std::endl;
         conf->immutableInput = 0;
	}
	if(result.count("intra")) {
		conf->intraperiod =  result["intra"].as<int>();
		std::cout << "Set Intra-refresh period to: " << conf->intraperiod << std::endl;
	}
	if(result.count("qp")) {
		conf->quantisationParameter =  result["qp"].as<int>();
		std::cout << "Set QP to: " << conf->quantisationParameter << std::endl;
	}
	if(result.count("profile")) {
		std::cout << opt.help() << std::endl;
		std::string profile = result["profile"].as<std::string>();
		try {
			conf->profile = profile_map.at(profile);
		}
		catch(...) {
			std::cerr << "No such profile: " << profile << std::endl;
			//conf->profile = MMAL_VIDEO_PROFILE_H264_HIGH;
			return false;
		}
	}
	if(result.count("inline")) {
		std::cout << "Set H264 inline to true" << std::endl;
		conf->bInlineHeaders = 1;
	}
	if(result.count("timed_on")) {
		std::cout << opt.help() << std::endl;
		conf->onTime =  result["timed_on"].as<int>();
		if (conf->onTime < 1000) conf->onTime = 1000;
		if (conf->offTime < 1000) conf->offTime = 1000;
		conf->waitMethod = WaitMethod::TIMED;
		if (conf->timeout == -1) conf->timeout = 0;
	}
	if(result.count("timed_off")) {
		std::cout << opt.help() << std::endl;
		conf->offTime = result["timed_off"].as<int>();
		if (conf->onTime < 1000) conf->onTime = 1000;
		if (conf->offTime < 1000) conf->offTime = 1000;
		conf->waitMethod = WaitMethod::TIMED;
		if (conf->timeout == -1) conf->timeout = 0;
	}
	if(result.count("keypress")) {
		std::cout << "Set wait method to keypress" << std::endl;
		conf->waitMethod = WaitMethod::KEYPRESS;
	}      
	if(result.count("signal"))	{
		std::cout << "Set signal" << std::endl;
		conf->waitMethod = WaitMethod::WM_SIGNAL;
		signal(SIGUSR1, default_signal_handler);// TODO from RaspiHelpers.h
	}
	if(result.count("initial")) {
		std::string profile = result["initial"].as<std::string>();
		try {
			conf->bStartRunning = initial_map.at(profile);
			std::cout << "Set initial states to: " << profile << std::endl;
		}
		catch(...) {
			std::cerr << "No such initial states: " << profile << std::endl;
			return false;
		}
	}
	if(result.count("segment")) {
		conf->segmentSize = result["segment"].as<int>();
		std::cout << "Set segment size to: " << conf->segmentSize << std::endl;
		// Must enable inline headers for this to work
		conf->bInlineHeaders = 1;
	}
	if(result.count("wrap")) {
		conf->segmentWrap = result["wrap"].as<int>();
		std::cout << "Set segment wrap to: " << conf->segmentWrap << std::endl;
	}
	if(result.count("start")) {
		conf->startingSegmentNumber = result["start"].as<int>();
		std::cout << "Set segment number to: "<< conf->startingSegmentNumber << std::endl;
	}
	if(result.count("split")) {
		std::cout << "Enable split" << std::endl;
		// Must enable inline headers for this to work
		conf->bInlineHeaders = 1;
		conf->splitWait = 1;
	}
	if(result.count("circular")) {
		std::cout << "Enable circular" << std::endl;
		conf->bCircularBuffer = 1;
	}
	if(result.count("vectors")) {
		conf->inlineMotionVectors = 1;
		conf->imv_filename = result["vectors"].as<std::string>();
		std::cout << "Set file name of inline Motion Vectors to: " << conf->imv_filename << std::endl;
	}
	if(result.count("irefresh")) {
		std::string profile = result["irefresh"].as<std::string>();
		try {
			conf->intra_refresh_type = intra_refresh_map.at(profile);
			std::cout << "Set refresh type to: " << profile << std::endl;
		}
		catch(...) {
			std::cerr << "No such refresh type: " << profile << std::endl;
			return false;
		}
	}
	if(result.count("flush")) {
		std::cout << "Set flush buffers" << std::endl;
		conf->flush_buffers = true;
	}
	if(result.count("save-pts")) {
		conf->save_pts = 1;
		conf->pts_filename = result["save-pts"].as<std::string>();
		std::cout << "Save pts file to: " << conf->pts_filename << std::endl;
	}
	if(result.count("raw-pts")) {
		conf->save_raw_pts = true;
		conf->raw_pts_filename = result["raw-pts"].as<std::string>();
		std::cout << "Save raw pts file to: " << conf->raw_pts_filename << std::endl;
	}
	if(result.count("codec")) {
		std::string str_codec = result["codec"].as<std::string>();
		std::cout << "Set codec to: " << str_codec << std::endl;
		if( str_codec.compare("H264")==0 ) conf->encoding = MMAL_ENCODING_H264;
		else if( str_codec.compare("MJPEG")==0 ) conf->encoding = MMAL_ENCODING_MJPEG;
		else
			return false;
	}
	if(result.count("level")) {
		std::string level = result["level"].as<std::string>();
		try {
			conf->level = level_map.at(level);
			std::cout << "Set h264 level to: " << level << std::endl;
		}
		catch(...) {
			std::cerr << "No such h264 level: " << level << std::endl;
			return false;
		}
	}
	if(result.count("raw")) {
		conf->raw_output = 1;
		conf->raw_filename = result["raw"].as<std::string>();
		std::cout << opt.help() << std::endl;
	}
	if(result.count("raw-format")) {
		std::string rfmt = result["raw-format"].as<std::string>();
		try {
			conf->raw_output_fmt = raw_output_fmt_map.at(rfmt);
			std::cout << "Set raw format to: " << rfmt << std::endl;
		}
		catch(...) {
			std::cerr << "No such raw format: " << rfmt << std::endl;
			return false;
		}
	}
	if(result.count("listen")) {
		std::cout << "Set to TCP listen" << std::endl;
		conf->netListen = true;
	}
	if(result.count("slices")) {
		conf->slices = result["slices"].as<int>();
		std::cout << "Set horizontal slices per frame: " << conf->slices << std::endl;
		
	}
	if(result.count("CommandSPSTimings")) {
		std::cout << "Set Add SPS Timing" << std::endl;
		conf->addSPSTiming = MMAL_TRUE;
	}

	
	// Common group
	if(result.count("width") ) {
		conf->width = result["width"].as<int>();
		std::cout << "Set width to: " << conf->width << std::endl;
	}
	if(result.count("height") ) {
		conf->height = result["height"].as<int>();
		std::cout << "Set height to: " << conf->height << std::endl;
	}
	if(result.count("output") ) {
		conf->filename = result["output"].as<std::string>();
		// Ensure that any %<char> is either %% or %d.
		for(auto it = conf->filename.begin(); it != conf->filename.end(); it++) {
			if(*it == '%') {
				it++;
				int digits=0;
				while(std::isdigit(*it)) {
					it++;
					digits++;
				}
				if(!((*it == '%' && !digits) || *it == 'd')) {
					std::cerr << "Filename contains %% characters, but not %%d or %%%% - sorry, will fail" << std::endl;
					return false;
				}
			}
		}
	}
	if(result.count("verbose") ) {
		conf->verbose = 1;
		std::cout << "Set Verbose" << std::endl;
	}
	if(result.count("camselect") ) {
		conf->cameraNum = result["camselect"].as<int>();
		std::cout << "Set camera number: " << conf->cameraNum << std::endl;
	}
	if(result.count("mode") ) {
		conf->sensor_mode = result["mode"].as<int>();
		std::cout << "Set sensor mode: " << conf->sensor_mode << std::endl;
	}
	if(result.count("gpsdexif") ) {
		conf->gps = true;
		std::cout << "Enable GPS (no use a.t.m)" << std::endl;
	}
	
	
	// ===== Camera group =====
	using namespace RaspiCamControl;
	auto params = &(conf->camera_parameters);
	if( result.count("sharpness") ) {
		params->sharpness = result["sharpness"].as<int>();
		std::cout << "Set sharpness to: " << params->sharpness << std::endl;
	}
	if( result.count("contrast") ) {
		params->contrast = result["contrast"].as<int>();
		std::cout << "Set contrast to: " << params->contrast << std::endl;
	}
	if( result.count("brightness") ) {
		params->brightness = result["brightness"].as<int>();
		std::cout << "Set brightness to: " << params->brightness << std::endl;
	}
	if( result.count("saturation") ) {
		params->saturation = result["saturation"].as<int>();
		std::cout << "Set saturation to: " << params->saturation << std::endl;
	}
	if( result.count("ISO") ) {
		params->ISO = result["ISO"].as<int>();
		std::cout << "Set ISO to: " << params->ISO << std::endl;
	}
	if( result.count("vstab") ) {
		params->videoStabilisation = 1;
		std::cout << "Set video stabilization to true" << std::endl;
	}
	if( result.count("ev") ) {
		 params->exposureCompensation = result["ev"].as<int>();
		std::cout << "Set ev to: " <<  params->exposureCompensation << std::endl;
	}
	if( result.count("exposure") ) {
		std::string arg_str = result["exposure"].as<std::string>();
		try {
			params->exposureMode = exposure_map.at(arg_str);
			std::cout << "Set exposure to: " << arg_str << std::endl;
		}
		catch(...) {
			std::cerr << "No such exposure: " << arg_str << std::endl;
			return false;
		}
	}
	if( result.count("flicker") ) {
		std::string arg_str = result["flicker"].as<std::string>();
		try {
			params->flickerAvoidMode = flicker_avoid_map.at(arg_str);
			std::cout << "Set flicker to: " << arg_str << std::endl;
		}
		catch(...) {
			std::cerr << "No such flicker: " << arg_str << std::endl;
			return false;
		}
	}
	if( result.count("awb") ) {
		std::string arg_str = result["awb"].as<std::string>();
		try {
			params->awbMode = awb_map.at(arg_str);
			std::cout << "Set awb to: " << arg_str << std::endl;
		}
		catch(...) {
			std::cerr << "No such awb: " << arg_str << std::endl;
			return false;
		}
	}
	if( result.count("imxfx") ) {
		std::string arg_str = result["imxfx"].as<std::string>();
		try {
			params->imageEffect = imagefx_map.at(arg_str);
			std::cout << "Set imxfx to: " << arg_str << std::endl;
		}
		catch(...) {
			std::cerr << "No such imxfx: " << arg_str << std::endl;
			return false;
		}
	}
	if( result.count("colfx") ) {
		// Colour FX - needs string "u:v"
		std::string arg_str = result["colfx"].as<std::string>();
		sscanf(arg_str.c_str(), "%u:%u", &params->colourEffects.u, &params->colourEffects.v);
		params->colourEffects.enable = 1;
	}
	if( result.count("metering") ) {
		std::string arg_str = result["metering"].as<std::string>();
		try {
			params->exposureMeterMode = metering_mode_map.at(arg_str);
			std::cout << "Set metering to: " << arg_str << std::endl;
		}
		catch(...) {
			std::cerr << "No such metering: " << arg_str << std::endl;
			return false;
		}
	}
	if( result.count("rotation") ) {
		params->rotation = result["rotation"].as<int>();
		std::cout << "Set rotation to: " << params->rotation << std::endl;
	}
	if( result.count("hflip") ) {
		params->hflip  = 1;
		std::cout << "Set hflip to true" << std::endl;
	}
	if( result.count("vflip") ) {
		params->vflip = 1;
		std::cout << "Set vflip to true" << std::endl;
	}
	if( result.count("roi") ) {
		double x,y,w,h;
		int args;
		std::string arg_str = result["roi"].as<std::string>();
		args = sscanf(arg_str.c_str(), "%lf,%lf,%lf,%lf", &x,&y,&w,&h);
		if (args != 4 || x > 1.0 || y > 1.0 || w > 1.0 || h > 1.0) {
			std::cerr << "No such ROI: " << arg_str << std::endl;
			return false;
		}
		// Make sure we stay within bounds
		if (x + w > 1.0) w = 1 - x;
		if (y + h > 1.0) h = 1 - y;

		params->roi.x = x;
		params->roi.y = y;
		params->roi.w = w;
		params->roi.h = h;
	}
	if( result.count("shutter") ) {
		params->shutter_speed = result["shutter"].as<int>();
		std::cout << "Set shutter to: " <<  params->shutter_speed << std::endl;
	}
	if( result.count("awbgains") ) {
		double r,b;
		int args;
		std::string arg_str = result["awbgains"].as<std::string>();
		args = sscanf(arg_str.c_str(), "%lf,%lf", &r,&b);
		if (args != 2 || r > 8.0 || b > 8.0) {
			std::cerr << "No such AWB: " << arg_str << std::endl;
			return false;
		}
		params->awb_gains_r = r;
		params->awb_gains_b = b;
	}
	if( result.count("drc") ) {
		std::string arg_str = result["drc"].as<std::string>();
		try {
			params->drc_level = drc_mode_map.at(arg_str);
			std::cout << "Set drc to: " << arg_str << std::endl;
		}
		catch(...) {
			std::cerr << "No such drc: " << arg_str << std::endl;
			return false;
		}
	}
	if( result.count("stats") ) {
		params->stats_pass = MMAL_TRUE;
		std::cout << "Set stats to true" << std::endl;
	}
	if( result.count("annotate") ) {
		std::string arg_str = result["annotate"].as<std::string>();
		char dummy;
		unsigned int bitmask;
		// If parameter is a number, assume its a bitmask, otherwise a string
		if (sscanf(arg_str.c_str(), "%u%c", &bitmask, &dummy) == 1)
		{
			params->enable_annotate |= bitmask;
		}
		else
		{
			params->enable_annotate |= ANNOTATE_USER_TEXT;
			//copy string char by char and replace "\n" with newline character
			unsigned char c;
			char const *s = arg_str.c_str();
			char *t = &params->annotate_string[0];
			int n=0;
			while ((c = *s++) && n < MMAL_CAMERA_ANNOTATE_MAX_TEXT_LEN_V3-1)
			{
				if (c == '\\' && *s)
				{
				switch (c = *s++)
				{
				case 'n':
					c = '\n';
					break;

				default:
					c = '\\';
					s--;
					break;
				}
				}
				*(t++) = c;
				n++;
			}
			*t='\0';
			//params->annotate_string[MMAL_CAMERA_ANNOTATE_MAX_TEXT_LEN_V3-1] = '\0';
		}
	}
	if( result.count("stereo") ) {
		std::string arg_str = result["stereo"].as<std::string>();
		try {
			params->stereo_mode.mode = stereo_mode_map.at(arg_str);
			std::cout << "Set stereo to: " << arg_str << std::endl;
		}
		catch(...) {
			std::cerr << "No such stereo: " << arg_str << std::endl;
			return false;
		}
	}
	if( result.count("decimate") ) {
		params->stereo_mode.decimate = MMAL_TRUE;
		std::cout << "Set decimate to true" << std::endl;
	}
	if( result.count("3dswap") ) {
		params->stereo_mode.swap_eyes = MMAL_TRUE;
		std::cout << "Set swap eyes to true" << std::endl;
	}
	if( result.count("annotateex") ) {
		// 3 parameters - text size (6-80), text colour (Hex VVUUYY) and background colour (Hex VVUUYY)
		std::string arg_str = result["annotate"].as<std::string>();
		sscanf(arg_str.c_str(), "%d,%d,%d,%u,%u,%u", &params->annotate_text_size,
				&params->annotate_text_colour,
				&params->annotate_bg_colour,
				&params->annotate_justify,
				&params->annotate_x,
				&params->annotate_y
				);
	}
	if( result.count("analoggain") ) {
		float gain = result["analoggain"].as<float>();
		if (gain > 16.0) {
			std::cerr << "No such analog gain: " << gain << std::endl;
			return false;
		}
		params->analog_gain = gain;
		std::cout << "Set analog gain to: " << params->analog_gain << std::endl;
	}
	if( result.count("digitalgain") ) {
		double gain = result["digitalgain"].as<float>();
		if (gain > 64.0) {
			std::cerr << "No such digital gain: " << gain << std::endl;
			return false; 
		}
		params->digital_gain = gain;
		std::cout << "Set digital gain to: " << params->digital_gain << std::endl;
	}
	if( result.count("settings") ) {
		params->settings = 1;
		std::cout << "Enable cam setting dump" << std::endl;
	}
	
	
	// ===== Preview group =====
	if( result.count("preview"))  
	{
		std::string arg_str = result["preview"].as<std::string>();
		int tmp;
		conf->wantPreview = 1;
		tmp = sscanf(arg_str.c_str(), "%d,%d,%d,%d",
					&conf->previewWindow.x, &conf->previewWindow.y,
					&conf->previewWindow.width, &conf->previewWindow.height);
		// Failed to get any window parameters, so revert to full screen
		if (tmp == 0) 	conf->wantFullScreenPreview = 1;
		else {
			conf->wantFullScreenPreview = 0;
			std::cout << "Set preview to: " << arg_str << std::endl;
		}
	}
	if( result.count("fullscreen") ) {
		// Want full screen preview mode (overrides display rect)
		conf->wantPreview = 1;
		conf->wantFullScreenPreview = 1;
	}
	if( result.count("opacity") ) {
		conf->opacity = result["opacity"].as<int>();
		std::cout << "Set opacity to: " << conf->opacity << std::endl;
	}
	if( result.count("nopreview") ) {
		conf->wantPreview = 0;
		std::cout << "Disable preview" << std::endl;
	}
	
	// End of cmdline parsing
	return true;
}

void default_signal_handler(int signal_number)
{
   if (signal_number == SIGUSR1)
   {
      // Handle but ignore - prevents us dropping out if started in none-signal mode
      // and someone sends us the USR1 signal anyway
   }
   else
   {
      // Going to abort on all other signals
      std::cerr << "Aborting program" << std::endl;
      exit(130);
   }
}

/**
 * main
 */
int main(int argc, const char **argv)
{
	//TBD
	int exit_code = 0;
	signal(SIGINT, default_signal_handler);  // TBD
	// Disable USR1 for the moment - may be reenabled if go in to signal capture mode
	signal(SIGUSR1, SIG_IGN);

	//ORIG set_app_name(argv[0]); EWING: in RaspiHelpers.c

	// Do we have any parameters
	if (argc == 1)
	{
		std::cout << "Use --help to display usage" << std::endl;
		return -1;
	}

	// Our main data storage vessel..
	RaspiEncamodeConfig rec_config;
	rec_config.SetDefault();
	int demoMode = 0;        /// Run app in demo mode
	int demoInterval = 250;  /// Interval (ms) between camera settings changes
	
	
	// Parse the command line and put options in to our status structure
	if( !parse_cmdline(argc, argv, &rec_config, &demoMode, &demoInterval) )
	{
		std::cout << "Some error occured during CLI parsing..." << std::endl;
		return -1;
	}
	if (rec_config.timeout == -1) rec_config.timeout = 5000;
	
	// Main executing object
	RaspiEncamode vid_cam;
	
	if( !vid_cam.Init(rec_config) ) 
	{
		std::cout << "Some error occured during initialization..." << std::endl;
		return -1;
	}
	
	
	if (demoMode)
	{
		// Run for the user specific time..
		int num_iterations = rec_config.timeout / demoInterval;
		int i;

		if (rec_config.verbose)
			std::cout << "Running in demo mode" << std::endl;

		for (i=0; rec_config.timeout == 0 || i<num_iterations; i++)
		{
			//TODO raspicamcontrol_cycle_test(rec_config.camera_component);
			//TODO vcos_sleep(demoInterval);
		}
	}
	
	vid_cam.Run();
	//WIP copied till here
	
	// TODO
		//ORIG RaspiVidState state;
// 	if (status != MMAL_SUCCESS)
// 		raspicamcontrol_check_configuration(128);

	//   if (state.common_settings.gps)
	//      raspi_gps_shutdown(rec_config.verbose);

	return exit_code;
}
