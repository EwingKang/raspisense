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

find_library(wiringPi_LIB wiringPi)
message( "Find WiringPI lib" )

# External libs
add_subdirectory(lib/cxxopts)
add_subdirectory(lib/spdlog)

cmake_policy(SET CMP0048 OLD)
add_subdirectory(lib/serial)

add_compile_options(-Wall -Wextra -pedantic)

# library
add_library(ublox_gnss_library
	src/ublox_gnss_library.cpp)
target_include_directories(ublox_gnss_library 
  PUBLIC
	include/
  PRIVATE
	lib/serial/include/
)
target_link_libraries(ublox_gnss_library
	${wiringPi_LIB}
	spdlog
	serial
)

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

# TODO: dont' include serial manually?
add_executable(simple_m8n_test
	src/simpletest_m8n.cpp
)
target_include_directories(simple_m8n_test
  PUBLIC
	lib/serial/include/
)
target_link_libraries(simple_m8n_test
	serial
)

add_executable(m8n_test
	src/m8n_test_main.cpp 
	)
target_include_directories(m8n_test
  PUBLIC
	lib/serial/include/
)
target_link_libraries(m8n_test
	serial
	ublox_gnss_library
	cxxopts
)
