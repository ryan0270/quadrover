# Standard setup
cmake_minimum_required (VERSION 2.6)
project (Playground CXX C)

# Find toadlet
find_package (toadlet REQUIRED)

# need to set this varaible that OpenCV is expecting to indicate the processor type
set(ANDROID_NDK_ABI_NAME ${NDK_NAME_ARCH}) 
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


# Includes
set (INC ${INC}
	${Playground_SOURCE_DIR}/cpp
	${TOADLET_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
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
set(LIBS ${LIBS}
	${TOADLET_EGG_LIB} 
	${OpenCV_LIBS_OPT} 
	-lz -llog -ldl
	 EGL android
)

# Source files
set (SRC ${SRC}
	jni/jni_part.cpp
	./cpp/Playground.cpp
	./cpp/VisionProcessor.cpp
	./cpp/QuadLogger.cpp
)

# Headers
set (H ${H}
	./cpp/Playground.h
	./cpp/Time.h
	./cpp/VisionProcessor.h
	./cpp/QuadLogger.h
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -fno-exceptions")

# set (OpenCV_LIB_DIR_OPT "${OpenCV_LIB_DIR_OPT}/${NDK_NAME_ARCH}")
# Why doesn't this work?
#if (COMMAND COPY_NATIVE_CAMERA_LIBS)
#	message("Copying opencv camera libs to ${Playground_SOURCE_DIR}/libs/${NDK_NAME_ARCH}")
#	COPY_NATIVE_CAMERA_LIBS(${Playground})
#else()
#	message("Failed to copy opencv camera libs: command not found")
#endif()


# Executable
add_library (Playground SHARED ${SRC} ${H})
target_link_libraries (Playground ${LIBS})
install (TARGETS Playground DESTINATION ${Playground_SOURCE_DIR}/libs/${NDK_NAME_ARCH})

#if (${ANDROID_NDK_API_LEVEL} LESS 9)
	install (FILES ${JTOADLET_FLICK_JAR} ${JTOADLET_PEEPER_JAR} ${JTOADLET_RIBBIT_JAR} ${JTOADLET_PAD_JAR} ${JTOADLET_TADPOLE_JAR} DESTINATION ${Playground_SOURCE_DIR}/libs)
#endif (${ANDROID_NDK_API_LEVEL} LESS 9)
