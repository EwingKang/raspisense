//#include <ctime>
#include <thread>
#include <chrono>
#include <fstream>
#include <iostream>
#include <raspicam/raspicam.h>

 
int main ( int argc,char **argv)
{
	(void) argc;
	(void) argv;
	using namespace std;
	raspicam::RaspiCam Camera; //Camera object
	//Open camera 
	cout<<"Opening Camera..."<<endl;
	if ( !Camera.open()) {cerr<<"Error opening camera"<<endl;return -1;}
	//wait a while until camera stabilizes
	cout<<"Connected to camera ="<<Camera.getId() <<" bufs="<<Camera.getImageBufferSize( )<<endl;
	cout<<"Sleeping for 3 secs"<<endl;
	std::this_thread::sleep_for(std::chrono::seconds(3) );
	
	//allocate memory
	unsigned char *data=new unsigned char[ Camera.getImageBufferSize() ];
	int64_t timestamp=0;
	
	int nCount = 100;
	for ( int i=0; i<nCount; i++ ) {
        //capture
		Camera.grab();
		Camera.retrieve ( &timestamp, data,raspicam::RASPICAM_FORMAT_IGNORE );//get camera image
        if ( i%5==0 )  
		{
			cout<<"\r captured "<<i<<" images, "
			    << "timestamp: " << timestamp << std::endl;
		}
    }
	
	//save
	std::ofstream outFile ( "raspicam_image.ppm",std::ios::binary );
	outFile<<"P6\n"<<Camera.getWidth() <<" "<<Camera.getHeight() <<" 255\n";
	outFile.write ( ( char* ) data, Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB ) );
	cout<< "timestamp: " << timestamp << endl;
	cout<<"Image saved at raspicam_image.ppm"<<endl;
	//free resrources    
	delete data;
	return 0;
}
