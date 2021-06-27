#ifndef RASPISENSE_HPP__
#define RASPISENSE_HPP__
#include <atomic>
#include <string>
#include <ublox_gnss_library.h>
#include <raspicam/raspicam.h>

struct RaspiSenseConfig
{
	std::string log_prefix = "rslog_";
	std::string gnss_port = "/dev/ttyAMA1";
	std::string gnss_dbg_port = "";
	unsigned int port_retry = 3;
	float port_retry_interval = 2;		//sec
	bool enable_gnss_csv = false;
	bool log_img_after_gnss_fix = true;
	
	
	// Internal use only, do not set;
	std::string pvt_log_file;
	std::string img_dir;
	std::string img_log_file;
};

class RaspiSense
{
public:
	bool Init(const RaspiSenseConfig & config);
	
	bool Spin();

private:
	RaspiSenseConfig _config;
	SFE_UBLOX_GNSS _m8n;
	raspicam::RaspiCam _cam;
	bool _initialized = false;
	
	std::atomic<bool> _spin_end;
	std::atomic<bool> _gnss_fixed;

	bool InitDirectory();
	bool InitGnss();
	bool ConfigGnss();
	bool InitCamera();
	
	bool SpinGnss();
	bool SpinCamera();
	
	bool OpenPort(serial::Serial * tgt_port, 
				  const std::string & port_dev,
				  unsigned int boud_rate	) const;
};
#endif
