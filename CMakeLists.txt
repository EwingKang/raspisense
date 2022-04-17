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

add_subdirectory(module/rpicam_cpp)

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


add_executable(raspisense_main
	src/raspisense_main.cpp 
	)
target_link_libraries(raspisense_main
	raspisense
	cxxopts
)

