# Standard setup
cmake_minimum_required (VERSION 2.6)
project (SerialTest CXX C)



# Includes
set (INC ${INC}
	${SerialTest_SOURCE_DIR}/cpp
	)
include_directories (${INC})

message("Include dirs: ")
foreach(dir ${INC})
	message(${dir})
endforeach()

message("OpenCV libs: ")
foreach(lib ${OpenCV_LIBS_OPT})
	message(${lib})
endforeach()

# Libraries
set (LIBS 
	-lgnustl_static # this is already included by the toadlet toolchain, but the link order causes problems for OpenCV so adding it again here seems to help
	-lz -llog -ldl EGL android
)

# Source files
set (SRC ${SRC}
	./cpp/jniInterface.cpp
)

# Headers
set (H ${H}
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Executable
add_library (SerialTest SHARED ${SRC} ${H})
target_link_libraries (SerialTest ${LIBS})
install (TARGETS SerialTest DESTINATION ${SerialTest_SOURCE_DIR}/libs/${NDK_NAME_ARCH})

#if (${ANDROID_NDK_API_LEVEL} LESS 9)
	install (FILES ${JTOADLET_FLICK_JAR} ${JTOADLET_PEEPER_JAR} ${JTOADLET_RIBBIT_JAR} ${JTOADLET_PAD_JAR} ${JTOADLET_TADPOLE_JAR} DESTINATION ${SerialTest_SOURCE_DIR}/libs)
#endif (${ANDROID_NDK_API_LEVEL} LESS 9)
