# Standard setup
cmake_minimum_required (VERSION 2.6)
project (Rover CXX C)


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

set( CPUFEATURES_PATH "${ANDROID_NDK}/sources/android/cpufeatures")

# Includes
set (INC ${INC}
	${Rover_SOURCE_DIR}/cpp
	${TOADLET_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
#	${MXML_DIR}/include
#	${TBB_DIR}/include # note that the TBB lib comes in with OpenCV libs
	${CPUFEATURES_PATH}
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
#	${MXML_DIR}/lib/${NDK_NAME_ARCH}/libmxml.a
#	${TBB_DIR}/libs/${NDK_NAME_ARCH}/libtbb.a
	-lgnustl_static # this is already included by the toadlet toolchain, but the link order causes problems for OpenCV so adding it again here seems to help
	-lz -llog -ldl EGL android
)

# Source files
set (SRC ${SRC}
	./cpp/Rotation.cpp
	./cpp/SensorManager.cpp
	./cpp/Observer_Translational.cpp
	./cpp/DataLogger.cpp
	./cpp/CommManager.cpp
	./cpp/TrackedObject.cpp
	./cpp/ObjectTracker.cpp
	./cpp/FeatureFinder.cpp
	./cpp/RegionFinder.cpp
	./cpp/TranslationController.cpp
	./cpp/AttitudeThrustController.cpp
	./cpp/MotorInterface.cpp
	./cpp/VideoMaker.cpp
	./cpp/Observer_Angular.cpp
	./cpp/VelocityEstimator.cpp
	${CPUFEATURES_PATH}/cpu-features.c
#	./cpp/ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.cpp
	./cpp/Rover.cpp
	jni/jni_part.cpp
)

# Headers
set (H ${H}
	./cpp/Listeners.h
	./cpp/Rotation.h
	./cpp/Common.h
	./cpp/Rover.h
	./cpp/Observer_Angular.h
	./cpp/Observer_Translational.h
	./cpp/DataLogger.h
	./cpp/Time.h
	./cpp/CommManager.h
	./cpp/TrackedObject.h
	./cpp/ObjectTracker.h
	./cpp/FeatureFinder.h
	./cpp/RegionFinder.h
	./cpp/TranslationController.h
	./cpp/AttitudeThrustController.h
	./cpp/MotorInterface.h
	./cpp/SensorManager.h
	./cpp/VideoMaker.h
	./cpp/constants.h
	./cpp/Data.h
	./cpp/VelocityEstimator.h
	${CPUFEATURES_PATH}/cpu-features.h
#	./cpp/ICSL/SystemModel/ISystemModel.h
#	./cpp/ICSL/SystemModel/ISystemModelAffine.h
#	./cpp/ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h
)

#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -fno-exceptions -std=c++0x")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti  -std=c++11")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")

set (OpenCV_LIB_DIR_OPT "${OpenCV_LIB_DIR_OPT}/${NDK_NAME_ARCH}")

# Executable
add_library (Rover SHARED ${SRC} ${H})
target_link_libraries (Rover ${LIBS})
install (TARGETS Rover DESTINATION ${Rover_SOURCE_DIR}/libs/${NDK_NAME_ARCH})
