#include <chrono>
#include <thread> // for sleep
#include <cxxopts.hpp>
#include <ublox_gnss_library.h>
#include "serial/serial.h"


int main(int argc, char *argv[])
{
	cxxopts::Options opt("M8N test", "hahaha");
	opt.parse(argc, argv);
	
	
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
		m8n_debug.setPort("/dev/pts/6");
		m8n_debug.setBytesize(serial::eightbits);
		m8n_debug.setBaudrate(9600);
		m8n_debug.setStopbits(serial::stopbits_one);
		m8n_debug.setFlowcontrol(serial::flowcontrol_none);
		m8n_debug.setTimeout(3000, 300, 10, 300, 10);
		m8n_debug.open();

		if (m8n_debug.isOpen()) {
			std::cout << "Debug port has been started at /dev/pts/6.\n";
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
	m8n.begin(m8n_port);
	
	if( m8n.isConnected() )
	{
		std::cout << "M8N connected" << std::endl;
	}
	else
	{
		do
		{
			std::cout << "Trying to reconnect... " << std::endl;
		} while (!m8n.isConnected());
	}
	
	int i = 0;
	while(true)
	{
		m8n.checkUblox();
		std::cout << "Check " << i++ << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
}
