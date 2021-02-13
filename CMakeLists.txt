cmake_minimum_required (VERSION 2.8) 
project (raspicam_test)

# Required packages
find_package(raspicam REQUIRED)
message( "Raspcam lib: ${raspicam_INCLUDE_DIR}" )
find_package(OpenCV REQUIRED)
message( "Open CV version: ${OpenCV_VERSION}" )

# Executables
add_executable(simpletest_raspicam 
	src/simpletest_raspicam_main.cpp
)  

target_link_libraries (simpletest_raspicam 
	${raspicam_LIBS}
)


add_executable(simpletest_raspicam_cv
	src/simpletest_raspicam_cv_main.cpp
)
target_link_libraries (simpletest_raspicam_cv
  ${raspicam_LIBS}
  ${raspicam_CV_LIBS}
  ${OpenCV_LIBS}
)
