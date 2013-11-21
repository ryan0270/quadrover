# Standard setup
cmake_minimum_required (VERSION 2.6)
project (adbTest CXX C)

# Find toadlet
find_package (toadlet REQUIRED)

# Includes
set (INC ${INC}
	${adbTest_SOURCE_DIR}/cpp
	${TOADLET_INCLUDE_DIR}
	)
include_directories (${INC})

message("Include dirs: ")
foreach(dir ${INC})
	message(${dir})
endforeach()

# Libraries
set (LIBS 
	${TOADLET_EGG_LIB} 
	-lgnustl_static # this is already included by the toadlet toolchain, but the link order causes problems so adding it again here seems to help
	-lz -llog -ldl EGL android
)

# Source files
set (SRC ${SRC}
	./cpp/jniInterface.cpp
	./cpp/MotorInterface.cpp
)

# Headers
set (H ${H}
	./cpp/MotorInterface.h
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Executable
add_library (adbTest SHARED ${SRC} ${H})
target_link_libraries (adbTest ${LIBS})
install (TARGETS adbTest DESTINATION ${adbTest_SOURCE_DIR}/libs/${NDK_NAME_ARCH})
