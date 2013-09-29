# Standard setup
cmake_minimum_required (VERSION 2.6)
project (targetFindTest CXX C)


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
	${targetFindTest_SOURCE_DIR}/cpp
	${TOADLET_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${ROVER_DIR}/cpp
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
	./cpp/funcs.cpp
)

# Headers
set (H ${H}
	./cpp/mser.h
	./cpp/funcs.h
	./cpp/ActiveObject.h
	${ROVER_DIR}/cpp/Time.h
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Executable
add_executable (targetFindTest ${SRC} ${H})
target_link_libraries (targetFindTest ${LIBS})
install (TARGETS targetFindTest DESTINATION ${targetFindTest_SOURCE_DIR}/libs/${NDK_NAME_ARCH})
