# Standard setup
cmake_minimum_required (VERSION 2.6)
project (adkTest CXX C)

# Includes
set (INC ${INC}
	${adkTest_SOURCE_DIR}/cpp
	)
include_directories (${INC})

message("Include dirs: ")
foreach(dir ${INC})
	message(${dir})
endforeach()

# Libraries
set (LIBS 
	-lgnustl_static # this is already included by the toadlet toolchain, but the link order causes problems so adding it again here seems to help
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
add_library (adkTest SHARED ${SRC} ${H})
target_link_libraries (adkTest ${LIBS})
install (TARGETS adkTest DESTINATION ${adkTest_SOURCE_DIR}/libs/${NDK_NAME_ARCH})

#if (${ANDROID_NDK_API_LEVEL} LESS 9)
	install (FILES ${JTOADLET_FLICK_JAR} ${JTOADLET_PEEPER_JAR} ${JTOADLET_RIBBIT_JAR} ${JTOADLET_PAD_JAR} ${JTOADLET_TADPOLE_JAR} DESTINATION ${adkTest_SOURCE_DIR}/libs)
#endif (${ANDROID_NDK_API_LEVEL} LESS 9)
