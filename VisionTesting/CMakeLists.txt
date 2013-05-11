# Standard setup
cmake_minimum_required (VERSION 2.6)
project (visionTest CXX C)


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
	${visionTest_SOURCE_DIR}/cpp
	${TOADLET_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	)
include_directories (${INC})

# Libraries
set (LIBS 
	${TOADLET_EGG_LIB} 
	${OpenCV_LIBS_OPT} 
)

# Source files
set (SRC ${SRC}
	./cpp/main.cpp
	./cpp/mser.cpp
	../../Rover/cpp/Data.cpp
)

# Headers
set (H ${H}
	./cpp/mser.cpp
	../../Rover/cpp/Data.h
	../../Rover/cpp/Time.h
	../../Rover/cpp/Common.h
	../../Rover/cpp/TNT_Utils.h
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Executable
add_executable (visionTest ${SRC} ${H})
target_link_libraries (visionTest ${LIBS})
install (TARGETS visionTest DESTINATION ${visionTest_SOURCE_DIR}/libs/${NDK_NAME_ARCH})
