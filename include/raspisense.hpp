#ifndef RASPISENSE_HPP__
#define RASPISENSE_HPP__
#include <string>
#include <ublox_gnss_library.h>

struct RaspiSenseConfig
{
	std::string log_prefix = "rslog_";
	std::string gnss_port = "/dev/ttyAMA1";
	std::string gnss_dbg_port = "";
	unsigned int port_retry = 3;
	float port_retry_interval = 2;		//sec
	bool enable_gnss_csv = false;
	
	
	// Internal use only, do not set;
	std::string pvt_log_file;
	std::string img_dir;
};

class RaspiSense
{
public:
	bool Init(const RaspiSenseConfig & config);
	
	bool Spin();

private:
	RaspiSenseConfig _config;
	SFE_UBLOX_GNSS _m8n;
	bool _initialized = false;

	bool InitDirectory();
	bool InitGnss();
	bool ConfigGnss();
	bool InitCamera();
	
	bool OpenPort(serial::Serial * tgt_port, 
				  const std::string & port_dev,
				  unsigned int boud_rate	) const;
};
#endif
