#include <chrono>
#include <thread> // for sleep
#include <iomanip>
#include <fstream>

#include <cxxopts.hpp>
#include <ublox_gnss_library.h>
#include "serial/serial.h"

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
	
	serial::Serial m8n_port;
	//UARL GPS StartUp
	while (!m8n_port.isOpen()) 
	{
		m8n_port.setPort("/dev/ttyAMA1");
		m8n_port.setBytesize(serial::eightbits);
		m8n_port.setBaudrate(9600);
		m8n_port.setStopbits(serial::stopbits_one);
		m8n_port.setFlowcontrol(serial::flowcontrol_none);
		m8n_port.setTimeout(3000, 300, 10, 300, 10);
		m8n_port.open();

		if (m8n_port.isOpen()) {
			std::cout << "GPS has been started at /dev/ttyAMA1.\n";
		}
		else 
		{
			std::cout << "GPS failed to start, retrying in 3 seconds.\n";
			std::chrono::seconds timespan(3); // or whatever
			std::this_thread::sleep_for(timespan);
		}
	}
	
	serial::Serial m8n_debug;
	//UARL GPS StartUp
	while (!m8n_debug.isOpen()) 
	{
		std::string debug_port(conf.dbg_port);
		m8n_debug.setPort(debug_port);
		m8n_debug.setBytesize(serial::eightbits);
		m8n_debug.setBaudrate(9600);
		m8n_debug.setStopbits(serial::stopbits_one);
		m8n_debug.setFlowcontrol(serial::flowcontrol_none);
		m8n_debug.setTimeout(3000, 300, 10, 300, 10);
		
		try {
			m8n_debug.open();
		}
		catch (...)
		{
			std::cerr << "ERROR: Unable to open device " << debug_port << std::endl;
			std::cout << "\nYou may want to try creating a pseudo port with: \"socat -d -d pty,raw,echo=0 pty,raw,echo=0 \" in another terminal\n"  << std::endl;
			throw;
		}

		if (m8n_debug.isOpen()) {
			std::cout << "Debug port has been started at " << debug_port << "\n";
			m8n_debug.write("Debug port has been started at " + debug_port + "\n");
			
		}
		else 
		{
			std::cout << "Debug port failed to start, retrying in 3 seconds.\n";
			std::chrono::seconds timespan(3); // or whatever
			std::this_thread::sleep_for(timespan);
		}
	}
	
	SFE_UBLOX_GNSS m8n;
	m8n.enableDebugging(m8n_debug);
	m8n.setNMEAOutputPort(m8n_debug);
	m8n.disableNmeaDebugPrint();
	m8n.setFileBufferSize(1024);  // Enable file buffer
	m8n.begin(m8n_port);
	
	std::vector<int> baud_rates {4800,9600,19200,38400,57600,115200,230400,460800 };
	auto baud_rate_it = ++baud_rates.begin();  // start from 9600
	if( m8n.isConnected() )
	{
		std::cout << "M8N connected" << std::endl;
	}
	else
	{
		do
		{
			if( ++baud_rate_it == baud_rates.end() ) { 
				baud_rate_it = baud_rates.begin(); 
			}
			std::cout << "Trying to reconnect with serial rate: " << *baud_rate_it << std::endl;
			m8n_port.setBaudrate(*baud_rate_it);
		} while (!m8n.isConnected());
	}
	
	std::cout  << "Getting port settings from M8N" << std::endl;
	
	//See ublox_class_id.h for port and commtype definition
	bool ret = m8n.getPortSettings(COM_PORT_I2C);
	std::bitset<8> x(ret);
	std::cout << "Port I2C: " << x << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	x = m8n.getPortSettings(COM_PORT_UART1);
	std::cout << "Port UART1: " << x << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	x = m8n.getPortSettings(COM_PORT_UART2);
	std::cout << "Port UART2: " << x << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	x = m8n.getPortSettings(COM_PORT_USB);
	std::cout << "Port USB: " << x << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	x = m8n.getPortSettings(COM_PORT_SPI);
	std::cout << "Port SPI: " << x << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	std::cout << "Run default setting for 5 seconds" << std::endl;
	for(int i=0; i<50; i++)
	{
		//std::cout << "Check " << i++ << std::endl;
		m8n.checkUblox();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	
	std::cout << "Disable NMEA" << std::endl;
	m8n.setUART1Output(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	// See recv spec: [11.3 UART Ports]
	std::cout << "Increase serial rate to max" << std::endl;
	m8n.setSerialRate(460800);
	m8n_port.setBaudrate(460800);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	std::cout << "Enable Auto PVT, set frequency to 20" << std::endl;
	m8n.setNavigationFrequency(20); //Produce two solutions per second
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	m8n.setAutoPVT(true); //Tell the GPS to "send" each solution
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	//m8n.saveConfiguration(); //Save the current settings to flash and BBR
	
	
	m8n.disableDebugging(); // to supress message
	
	
	// Receiving PVT messages
	m8n.logNAVPVT(true); // enable PVT log to file buffer
	const int pvt_total_len = UBX_NAV_PVT_LEN+8;
	uint8_t pvt_bin_bfr[pvt_total_len];
	uint8_t bin_header[8] = {0xFF, 0xFF, 0xFF, 0xAB, 0X00, 0X00, 0x01, 0x00};
	uint8_t tp_buf[2*sizeof(std::chrono::nanoseconds::rep)];
	std::ofstream bin_file;
	bin_file.open("pvt_log.bin", std::ios::binary);
	
	
	auto t0 = std::chrono::steady_clock::now();
	int msg_cnt = 0;
	while(true)
	{
		bool ret = m8n.getPVT();
		if(!ret)
		{
			//std::cout << "getPVT return false" << std::endl;
		}
		else
		{
			std::cout << "Time: " << m8n.getYear() << "."
					  << std::setfill('0') << std::setw(2)
					  << (int)m8n.getMonth() << "."
					  << std::setfill('0') << std::setw(2)
					  << (int)m8n.getDay() << " "
					  << std::setfill('0') << std::setw(2)
					  << (int)m8n.getHour() << ":"
					  << std::setfill('0') << std::setw(2)
					  << (int)m8n.getMinute() << ":"
					  << std::setfill('0') << std::setw(2)
					  << (int)m8n.getSecond() << "."
					  << std::setfill('0') << std::setw(3)
					  << m8n.getMillisecond() << ".."
					  << std::setfill('0') << std::setw(6)
					  << m8n.getNanosecond() % 1000000
					  << ", Fixtype: " << (int)m8n.getFixType()
					  << ", LLH (deg, m): ["
					  << m8n.getLatitude() << ", "
					  << m8n.getLongitude() << ", "
					  << m8n.getAltitude() << "]" << std::endl;
			auto tn = std::chrono::steady_clock::now();
			msg_cnt++;
			std::chrono::duration<double> diff = tn - t0;
			std::cout << "Average freq: " << msg_cnt / diff.count() << std::endl;
			
			uint16_t avail = m8n.fileBufferAvailable();
			
			if(avail != pvt_total_len)
			{
				std::cout << "Something went wrong, avail: " << avail << "bytes, clear buffer" << std::endl;
				uint8_t * trash = new uint8_t[avail];
				m8n.extractFileBufferData(trash, avail);
				delete[] trash;
			}
			else
			{
				m8n.extractFileBufferData(pvt_bin_bfr, pvt_total_len);
				std::chrono::time_point<std::chrono::steady_clock,std::chrono::nanoseconds> sen_start, sen_end;
				m8n.getSentenceTime(&sen_start, &sen_end);
				
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


bool ParseCli(int argc, char *argv[],
			  CliConfig * conf)
{
	cxxopts::Options opt("M8N test", "hahaha");
	
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

	conf->dbg_port = result[std::string("pts")].as<std::string>();
	
	return true;
}
