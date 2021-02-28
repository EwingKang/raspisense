cmake_minimum_required (VERSION 3.10) 
project (raspicam_test 
    VERSION 0.1.0 
    LANGUAGES CXX
)

# Required packages
find_package(raspicam REQUIRED)
message( "Raspcam lib: ${raspicam_INCLUDE_DIR}" )
find_package(OpenCV REQUIRED)
message( "Open CV version: ${OpenCV_VERSION}" )

# External libs
add_subdirectory(lib/cxxopts)

cmake_policy(SET CMP0048 OLD)
add_subdirectory(lib/serial)

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


add_executable(simple_m8n_test
	module/ublox_neo_m8n-gps/gps.cpp
)
target_include_directories(simple_m8n_test
  PUBLIC
	lib/serial/include/
)
target_link_libraries(simple_m8n_test
	serial
)
