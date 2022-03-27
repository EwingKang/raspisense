RaspiSense project

# Build project
```bash
mkdir build && build/
cmake .. -G Ninja
ninja
```

# Execution
```bash	
socat -d -d pty,raw,echo=0 pty,raw,echo=0^C
 
 # Running encoder with callback
 ./raspi_encamode_test --save-pts timestamp.csv --nopreview --output vid.h264 --raw raw.yuv --raw-pts raw_pts.csv -t 10000 -v^C

 ffmpeg -r 30 -i video.h264 -fflags +genpts -vcodec copy video.mp4
ffmpeg -r 30 -i video.h264 -vcodec copy video.mp4

```
# TODO
- [ ] Finish secondary TODOs in raspicam_camcontrol.cpp/ hpp
- [ ] Isolate raspicam CPP to a stand alone project
- [ ] (Maybe?) get a better name then raspi_encamode. Call it CppRaspiVid maybe?  

