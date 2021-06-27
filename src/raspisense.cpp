#include "raspisense.hpp"

#include <chrono>
#include <thread>   // for sleep
#include <iostream>
#include <iomanip>
#include <bitset>
#include <fstream>

#include <ublox_gnss_library.h>
#include "serial/serial.h"


bool RaspiSense::Init(const RaspiSenseConfig & config)
{
	_config = config;
	if( !InitGnss() ) return false;
	if( !ConfigGnss() ) return false;
	if( !InitCamera() ) return false;
	return true;
}

bool RaspiSense::Spin()
{
	// TODO: put GNSS and CAMERA on different thread since they're not related at all
	
	// Receiving PVT messages
	_m8n.logNAVPVT(true); // enable PVT log to file buffer
	const int pvt_total_len = UBX_NAV_PVT_LEN+8;
	uint8_t pvt_bin_bfr[pvt_total_len];
	uint8_t bin_header[8] = {0xFF, 0xFF, 0xFF, 0xAB, 0X00, 0X00, 0x01, 0x00};
	uint8_t tp_buf[2*sizeof(std::chrono::nanoseconds::rep)];
	std::ofstream bin_file;
	bin_file.open("pvt_log.bin", std::ios::binary);
	
	
	auto t0 = std::chrono::steady_clock::now();
	auto last_pvt_msg_time = t0;
	
	int msg_cnt = 0;
	while(true)
	{
		bool ret = _m8n.getPVT();
		if(!ret)
		{
			static int last_noret_warn = 0;
			auto diff = std::chrono::duration_cast<std::chrono::seconds>( std::chrono::steady_clock::now() - last_pvt_msg_time );
			if(diff.count() > 3 && last_noret_warn != diff.count() )
			{
				std::cout << "WARNING: no PVT message for " << diff.count() << "s" << std::endl;
				last_noret_warn = diff.count();
			}
		}
		else
		{
			auto tn = std::chrono::steady_clock::now();
			
			if(_config.enable_gnss_csv)
			{
				std::cout << "Time: " << _m8n.getYear() << "."
						<< std::setfill('0') << std::setw(2)
						<< (int)_m8n.getMonth() << "."
						<< std::setfill('0') << std::setw(2)
						<< (int)_m8n.getDay() << " "
						<< std::setfill('0') << std::setw(2)
						<< (int)_m8n.getHour() << ":"
						<< std::setfill('0') << std::setw(2)
						<< (int)_m8n.getMinute() << ":"
						<< std::setfill('0') << std::setw(2)
						<< (int)_m8n.getSecond() << "."
						<< std::setfill('0') << std::setw(3)
						<< _m8n.getMillisecond() << ".."
						<< std::setfill('0') << std::setw(6)
						<< _m8n.getNanosecond() % 1000000
						<< ", Fixtype: " << (int)_m8n.getFixType()
						<< ", LLH (deg, m): ["
						<< _m8n.getLatitude() << ", "
						<< _m8n.getLongitude() << ", "
						<< _m8n.getAltitude() << "]" << std::endl;
			}
			
			uint16_t avail = _m8n.fileBufferAvailable();
			
			if(avail != pvt_total_len)
			{
				std::cout << "Something went wrong, " << avail << " bytes available in m8n buffer clear buffer" << std::endl;
				uint8_t * trash = new uint8_t[avail];
				_m8n.extractFileBufferData(trash, avail);
				delete[] trash;
			}
			else
			{
				msg_cnt++;
				std::chrono::duration<double> diff = tn - t0;
				std::cout << "Average freq: " << msg_cnt / diff.count() << std::endl;
				last_pvt_msg_time = tn;
				
				_m8n.extractFileBufferData(pvt_bin_bfr, pvt_total_len);
				std::chrono::time_point<std::chrono::steady_clock,std::chrono::nanoseconds> sen_start, sen_end;
				_m8n.getSentenceTime(&sen_start, &sen_end);
				
				std::chrono::nanoseconds::rep start_cnt = sen_start.time_since_epoch().count();
				std::chrono::nanoseconds::rep end_cnt = sen_end.time_since_epoch().count();
				
				memcpy(tp_buf, (uint8_t *)&start_cnt, sizeof(std::chrono::nanoseconds::rep));
				memcpy(tp_buf+sizeof(std::chrono::nanoseconds::rep), (uint8_t *)&end_cnt, sizeof(std::chrono::nanoseconds::rep));
				
				bin_file.write((char *)bin_header, 8);
				bin_file.write((char *)tp_buf, 2*sizeof(std::chrono::nanoseconds::rep));
				bin_file.write((char *)pvt_bin_bfr, pvt_total_len);
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	
	bin_file.close();
}

static serial::Serial m8n_port;
bool RaspiSense::InitGnss()
{
	if(!_config.gnss_dbg_port.empty())
	{
		std::cout << "Starting GNSS debug output port " 
				  << _config.gnss_dbg_port << "..." <<std::endl;
		serial::Serial m8n_debug;
		if(!OpenPort(&m8n_debug, _config.gnss_dbg_port, 115200))
		{
			std::cout << "Failed.\n";
			std::cout << "You may want to try creating a pseudo port with: \"socat -d -d pty,raw,echo=0 pty,raw,echo=0 \" in another terminal\n"  << std::endl;
			return false;
		}
		_m8n.enableDebugging(m8n_debug);
		_m8n.setNMEAOutputPort(m8n_debug);
	}
	_m8n.disableNmeaDebugPrint();
	_m8n.setFileBufferSize(1024);  // Enable file buffer
	
	
	//UARL GPS StartUp
	std::cout << "Starting GNSS port " << _config.gnss_port << "..." << std::endl;
	if(!OpenPort(&m8n_port, _config.gnss_port, 9600))
	{
		std::cout << "Failed " << std::endl;
		return false;
	}
	_m8n.begin(m8n_port);
	
	std::vector<int> baud_rates {4800,9600,19200,38400,57600,115200,230400,460800 };
	auto baud_rate_it = ++baud_rates.begin();  // start from 9600
	const auto baud_rate_it_beg = baud_rate_it;
	while ( !_m8n.isConnected() )
	{
		// Wrap around baudrate options
		if( ++baud_rate_it == baud_rates.end() ) 
		{ 
			baud_rate_it = baud_rates.begin(); 
		}
		if( baud_rate_it == baud_rate_it_beg)
		{
			std::cout << "No respond from M8N." << std::endl;
			return false;
		}
		std::cout << "M8N not responding, try to connect with new baudrate: " << *baud_rate_it << std::endl;
		m8n_port.setBaudrate(*baud_rate_it);
	}
	std::cout << "M8N connected" << std::endl;
	return true;
}

bool RaspiSense::ConfigGnss()
{
	std::cout  << "Getting port settings from M8N" << std::endl;
	
	//See ublox_class_id.h for port and commtype definition
	bool ret = _m8n.getPortSettings(COM_PORT_I2C);
	std::bitset<8> x(ret);
	std::cout << "Port I2C: " << x << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	x = _m8n.getPortSettings(COM_PORT_UART1);
	std::cout << "Port UART1: " << x << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	x = _m8n.getPortSettings(COM_PORT_UART2);
	std::cout << "Port UART2: " << x << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	x = _m8n.getPortSettings(COM_PORT_USB);
	std::cout << "Port USB: " << x << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	x = _m8n.getPortSettings(COM_PORT_SPI);
	std::cout << "Port SPI: " << x << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	std::cout << "Run default M8N setting for 3 seconds" << std::endl;
	for(int i=0; i<30; i++)
	{
		//std::cout << "Check " << i++ << std::endl;
		_m8n.checkUblox();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	
	std::cout << "Disable NMEA" << std::endl;
	_m8n.setUART1Output(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	// See recv spec: [11.3 UART Ports]
	std::cout << "Set baudrate to maximum of 460800" << std::endl;
	_m8n.setSerialRate(460800);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	std::cout << "Enable Auto PVT, set frequency to 10" << std::endl;
	_m8n.setNavigationFrequency(10); //Produce two solutions per second
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	_m8n.setAutoPVT(true); //Tell the GPS to "send" each solution
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	//_m8n.saveConfiguration(); //Save the current settings to flash and BBR
	
	std::cout << "Disable debugging" << std::endl;
	_m8n.disableDebugging(); // to supress message
	
	return true;
}

bool RaspiSense::OpenPort(serial::Serial * tgt_port, 
						 const std::string & port_dev, 
						 unsigned int boud_rate) const
{
	unsigned int try_cnt = 0;
	while (!tgt_port->isOpen()) 
	{
		tgt_port->setPort(port_dev);
		tgt_port->setBytesize(serial::eightbits);
		tgt_port->setBaudrate(boud_rate);
		tgt_port->setStopbits(serial::stopbits_one);
		tgt_port->setFlowcontrol(serial::flowcontrol_none);
		tgt_port->setTimeout(3000, 300, 10, 300, 10);
		try {
			tgt_port->open();
		}
		catch (...) {
			std::cerr << "ERROR: Unable to open " << port_dev << std::endl;
			return false;
		}
			
		if (tgt_port->isOpen()) {
			std::cout << "Port " << port_dev << " opened." << std::endl;
			return true;
		}
		else 
		{
			try_cnt++;
			if(try_cnt == _config.port_retry) 
			{ 
				return false; 
			}
			std::cout << "Failed to start, retrying in " 
					  << _config.port_retry_interval << " seconds, " 
					  << _config.port_retry-try_cnt << " tries left.\n";
			std::this_thread::sleep_for(std::chrono::milliseconds((int)(_config.port_retry_interval*1000)));
		}
	}
	return false;
}

bool RaspiSense::InitCamera()
{
	return true;
}
