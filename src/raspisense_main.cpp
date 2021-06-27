#include <iostream>

#include "cxxopts.hpp"
#include "raspisense.hpp"


struct CliConfig
{
	std::string dbg_port;
};

bool ParseCli(int argc, char *argv[],
			  CliConfig * conf);


int main(int argc, char *argv[])
{
	CliConfig conf;
	if(! ParseCli(argc, argv, &conf) )
	{
		return 0;
	}
	
	RaspiSenseConfig sense_config;
	sense_config.gnss_dbg_port = conf.dbg_port;
	//sense_config.enable_gnss_csv = true; // Deubg
	
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
			  CliConfig * conf)
{
	cxxopts::Options opt("Raspisense", "Data collector for raspirry PI's compatible sensors");
	
	opt.add_options()
		("h,help", "Help")
		("pts", "debug output serial port", cxxopts::value<std::string>()) 
		;
	cxxopts::ParseResult result = opt.parse(argc, argv);
	
	if(result.count("help"))
	{
		std::cout << opt.help() << std::endl;
		return false;
	}

	if( result.count("pts") )
	{
		conf->dbg_port = result[std::string("pts")].as<std::string>();
	}
	
	return true;
}
