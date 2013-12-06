# Standard setup
cmake_minimum_required (VERSION 2.6)
project (cameraIMU_calibTest CXX C)

# Source files
set (SRC ${SRC}
	./cpp/main.cpp
	${ROVER_DIR}/cpp/QuadLogger.cpp
	${ROVER_DIR}/cpp/Rotation.cpp
	)

# Headers
set (H ${H}
	${ROVER_DIR}/cpp/Time.h
	${ROVER_DIR}/cpp/TNT_Utils.h
	${ROVER_DIR}/cpp/QuadLogger.h
	${ROVER_DIR}/cpp/Rotation.h
	${ROVER_DIR}/cpp/Data.h
	)

add_definitions(-DSCHED_NORMAL=0) # linux and android appear to have this defined on slightly different paths

find_package (OpenCV REQUIRED)

# set (OPENCV_LIB_TYPE=SHARED)
if (OPENCV-NOTFOUND)
	message("Could NOT find OpenCV")
else()
	message("OpenCV include dirs:")
	foreach (dir ${OpenCV_INCLUDE_DIRS})
		message(${dir})
	endforeach()
endif()

# Find toadlet
find_package (toadlet REQUIRED)

# Includes
set (INC ${INC}
	${cameraIMU_calibTest_SOURCE_DIR}/cpp
	${TOADLET_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${ROVER_DIR}/cpp
	${ANDROID_NDK}
	)
include_directories (${INC})

# Libraries
set (LIBS 
	${TOADLET_EGG_LIB} 
	${OpenCV_LIBS_OPT} 
	)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Executable
add_executable (cameraIMU_calibTest ${SRC} ${H})
target_link_libraries (cameraIMU_calibTest ${LIBS})
