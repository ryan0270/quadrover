# Standard setup
cmake_minimum_required (VERSION 2.6)
project (VisionTestApp CXX C)


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

# Find toadlet
find_package (toadlet REQUIRED)

# Includes
set (INC ${INC}
	${VisionTestApp_SOURCE_DIR}/cpp
	${TOADLET_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${TBB_DIR}/include # note that the TBB lib comes in with OpenCV libs
	${ANDROID_NDK}/sources/android
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
	${TOADLET_EGG_LIB} 
	${OpenCV_LIBS_OPT} 
#	${TBB_DIR}/libs/${NDK_NAME_ARCH}/libtbb.a
	-lgnustl_static # this is already included by the toadlet toolchain, but the link order causes problems for OpenCV so adding it again here seems to help
	-lz -llog -ldl EGL GLESv1_CM android -lm
)

# Source files
set (SRC ${SRC}
	./cpp/main.cpp
	./cpp/mapFuncs.cpp
	../../Rover/cpp/Data.cpp
	${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c
)

# Headers
set (H ${H}
	./cpp/mapFuncs.h
	../../Rover/cpp/Data.h
	../../Rover/cpp/Time.h
	../../Rover/cpp/Common.h
	../../Rover/cpp/TNT_Utils.h
	${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti  -std=c++11")

set (OpenCV_LIB_DIR_OPT "${OpenCV_LIB_DIR_OPT}/${NDK_NAME_ARCH}")


# Executable
add_library (VisionTestApp SHARED ${SRC} ${H})
target_link_libraries (VisionTestApp ${LIBS})
install (TARGETS VisionTestApp DESTINATION ${VisionTestApp_SOURCE_DIR}/libs/${NDK_NAME_ARCH})
