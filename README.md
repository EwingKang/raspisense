RaspiSense project


# Build project
## Prepare prerequisite
* Install the [Raspberry Pi VideoCore](https://github.com/raspberrypi/userland) library.
    * It's under `/opt/vc`. This directory usually comes with the RPI image. 
    * It can also be install with apt, according to the "VideoCore" section of [this tutorial](https://wiki.ubuntu.com/ARM/RaspberryPi#Videocore), 
    * or it can be built from source with [this] if it's not already under `/opt/vc`.
## Clone the project
```bash
git clone git@github.com:EwingKang/raspisense.git
cd raspisense
git submodule --update --init --recursive
```
## Build the project
```bash
mkdir build && build/
cmake .. -G Ninja
ninja
```

# Execution
```bash	
socat -d -d pty,raw,echo=0 pty,raw,echo=0
 
 # Running encoder with callback
 cd build/module/rpicam_cpp/
 ./rpicam_cpp_test --save-pts timestamp.csv --nopreview --output vid.h264 --raw raw.yuv --raw-pts raw_pts.csv -t 10000 -v

ffmpeg -r 30 -i video.h264 -fflags +genpts -vcodec copy video.mp4
ffmpeg -r 30 -i video.h264 -vcodec copy video.mp4

```
# TODO


