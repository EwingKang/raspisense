cmake_minimum_required (VERSION 3.10) 
project (raspicam_test 
    VERSION 0.2.0 
    LANGUAGES CXX
)
set (CMAKE_CXX_STANDARD 17)

# ===== Required packages ===== 
find_package(raspicam REQUIRED)
message( "Raspcam lib: ${raspicam_INCLUDE_DIR}" )
find_package(OpenCV REQUIRED)
message( "Open CV version: ${OpenCV_VERSION}" )
find_library(wiringPi_LIB wiringPi)
message( "Find WiringPI lib" )


# ===== Build 3rd party libs =====
add_subdirectory(lib/cxxopts)
add_subdirectory(lib/spdlog)
#cmake_policy(SET CMP0048 OLD)
add_subdirectory(lib/serial)


# ===== Compile settings =====
add_compile_options(-Wall -Wextra -pedantic)


# ===== Costum libraries =====
add_library(ublox_gnss_library
	src/ublox_gnss_library.cpp)
target_include_directories(ublox_gnss_library 
  PUBLIC
	include/
	lib/serial/include/
)
target_link_libraries(ublox_gnss_library
  PUBLIC
	serial
  PRIVATE
	${wiringPi_LIB}
	spdlog
)

add_library(bcm_host IMPORTED SHARED)
  #PATHS /opt/vc/lib/libbcm_host.so)
target_include_directories(bcm_host
  INTERFACE
	/opt/vc/include
)
target_link_directories(bcm_host
  INTERFACE
	/opt/vc/lib
)
set_target_properties(bcm_host PROPERTIES IMPORTED_LOCATION /opt/vc/lib/libbcm_host.so )

add_library(mmal IMPORTED SHARED)
target_include_directories(mmal
  INTERFACE
	/opt/vc/include/interface/mmal
	/opt/vc/include     # for vcos required by mmal_common.h
)
set_target_properties(mmal PROPERTIES IMPORTED_LOCATION /opt/vc/lib/libmmal.so )

# Not currently directly needed
add_library(vcos IMPORTED SHARED)
target_include_directories(vcos
  INTERFACE
	/opt/vc/include/interface/vcos
)
set_target_properties(vcos PROPERTIES IMPORTED_LOCATION /opt/vc/lib/libvcos.so )

add_library(raspisense
	src/raspisense.cpp 
	)
target_include_directories(raspisense
  PRIVATE
	lib/serial/include/
)
target_link_libraries(raspisense
  PUBLIC
	ublox_gnss_library
	${raspicam_LIBS}
  PRIVATE
	serial
	stdc++fs
)

add_library(raspi_encamode
	src/raspi_encamode.cpp
	src/raspicam_camcontrol.cpp
)
target_include_directories(raspi_encamode
  PUBLIC
	include/
	#dependencies/interface/mmal/
	#dependencies/
  PRIVATE
)
target_link_libraries(raspi_encamode
  PUBLIC
	mmal
  PRIVATE
	spdlog
	bcm_host
	#${raspicam_LIBS}
)

# ===== Executables =====
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
	src/simpletest_m8n.cpp
)
target_include_directories(simple_m8n_test
  PRIVATE
	lib/serial/include/
)
target_link_libraries(simple_m8n_test
	serial
)

add_executable(m8n_test
	src/m8n_test_main.cpp 
	)
target_include_directories(m8n_test
  PRIVATE
	lib/serial/include/
)
target_link_libraries(m8n_test
	serial
	ublox_gnss_library
	cxxopts
)

add_executable(raspi_encamode_test
	src/raspi_encamode_testmain.cpp)
target_link_libraries(raspi_encamode_test
  PRIVATE
	cxxopts
	raspi_encamode
)

add_executable(raspisense_main
	src/raspisense_main.cpp 
	)
target_link_libraries(raspisense_main
	raspisense
	cxxopts
)

