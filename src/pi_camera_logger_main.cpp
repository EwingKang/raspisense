//#include <ctime>
#include <thread>
#include <chrono>
#include <fstream>
#include <iostream>
#include <raspicam/raspicam.h>

 
int main ( int argc,char **argv)
{
	using namespace std;
	raspicam::RaspiCam Camera; //Cmaera object
	//Open camera 
	cout<<"Opening Camera..."<<endl;
	if ( !Camera.open()) {cerr<<"Error opening camera"<<endl;return -1;}
	//wait a while until camera stabilizes
	cout<<"Sleeping for 3 secs"<<endl;
	std::this_thread::sleep_for(std::chrono::seconds(3) );
	//capture
	Camera.grab();
	//allocate memory
	unsigned char *data=new unsigned char[  Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB )];
	//extract the image in rgb format
	int64_t timestamp;
	Camera.retrieve ( &timestamp, data,raspicam::RASPICAM_FORMAT_IGNORE );//get camera image
	//save
	std::ofstream outFile ( "raspicam_image.ppm",std::ios::binary );
	outFile<<"P6\n"<<Camera.getWidth() <<" "<<Camera.getHeight() <<" 255\n";
	outFile.write ( ( char* ) data, Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB ) );
	cout<<"Image saved at raspicam_image.ppm"<<endl;
	//free resrources    
	delete data;
	return 0;
}
