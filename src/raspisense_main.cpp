#include <iostream>

#include "cxxopts.hpp"
#include "raspisense.hpp"

bool ParseCli(int argc, char *argv[],
			  RaspiSenseConfig * conf);


int main(int argc, char *argv[])
{
	RaspiSenseConfig sense_config;
	if(! ParseCli(argc, argv, &sense_config) )
	{
		return 0;
	}
	
	//sense_config.enable_gnss_csv = true; // Deubg
	sense_config.log_img_after_gnss_fix = false; // Debug
	
	RaspiSense sense;
	if( !sense.Init(sense_config) )
	{
		std::cout << "RaspiSense init failed." << std::endl;
		return -1;
	}
	
	sense.Spin();
	
	return 0;
}


bool ParseCli(int argc, char *argv[],
			  RaspiSenseConfig * conf)
{
	cxxopts::Options opt("Raspisense", "Data collector for raspirry PI's compatible sensors");
	
	opt.add_options()
		("h,help", "Help")
		("d,dir", "Output directory prefix", cxxopts::value<std::string>()) 
		("pts", "Debug output serial port", cxxopts::value<std::string>()) 
		("gnss_csv", "Output CSV for GNSS", cxxopts::value<bool>())
		("img_no_wait", "Start image saving without waiting for GNSS to start", cxxopts::value<bool>())
		;
	cxxopts::ParseResult result = opt.parse(argc, argv);
	
	if(result.count("help"))
	{
		std::cout << opt.help() << std::endl;
		return false;
	}
	
	std::cout << "Read options: " << std::endl;
	if( result.count("dir") )
	{
		conf->log_prefix = result[std::string("dir")].as<std::string>();
		std::cout << "  - Cnage output directory prefix to: " 
				  << conf->log_prefix << std::endl;
	}

	if( result.count("pts") )
	{
		conf->gnss_dbg_port = result[std::string("pts")].as<std::string>();
		std::cout << "  - Enable GNSS debug port to: " 
				  << conf->gnss_dbg_port << std::endl;
	}
	
	if( result.count("gnss_csv") )
	{
		std::cout<< "  - Enable GNSS csv output" << std::endl;
		conf->enable_gnss_csv = true;
	}
	
	if( result.count("img_no_wait") )
	{
		std::cout<< "  - Disable image wait after GNSS fix" << std::endl;
		conf->log_img_after_gnss_fix = false;
	}
	
	return true;
}
