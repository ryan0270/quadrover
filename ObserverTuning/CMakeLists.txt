# Standard setup
cmake_minimum_required (VERSION 2.6)
project (observerTuning CXX C)

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
	${observerTuning_SOURCE_DIR}/cpp
	${ROVER_DIR}/cpp
	${TOADLET_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${ANDROID_NDK_DIR}
	)
include_directories (${INC})

# Libraries
set (LIBS 
	${TOADLET_EGG_LIB} 
	${OpenCV_LIBS_OPT} 
	-lmxml
)

# Source files
set (SRC ${SRC}
	${ROVER_DIR}/cpp/RegionFinder.cpp
	${ROVER_DIR}/cpp/TrackedObject.cpp
	${ROVER_DIR}/cpp/ObjectTracker.cpp
	${ROVER_DIR}/cpp/DataLogger.cpp
	${ROVER_DIR}/cpp/Rotation.cpp
	${ROVER_DIR}/cpp/CommManager.cpp
	${ROVER_DIR}/cpp/FeatureFinder.cpp
	${ROVER_DIR}/cpp/MotorInterface.cpp
	${ROVER_DIR}/cpp/VideoMaker.cpp
	${ROVER_DIR}/cpp/Observer_Angular.cpp
	${ROVER_DIR}/cpp/VelocityEstimator.cpp
#	${ROVER_DIR}/cpp/ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.cpp
	${ROVER_DIR}/cpp/Observer_Translational.cpp
	${ROVER_DIR}/cpp/DataLogger.cpp
	${ROVER_DIR}/cpp/TranslationController.cpp
	${ROVER_DIR}/cpp/AttitudeThrustController.cpp
#	${ROVER_DIR}/cpp/SensorManager.cpp
	./cpp/main.cpp
)

# Headers
set (H ${H}
	${ROVER_DIR}/cpp/RegionFinder.h
	${ROVER_DIR}/cpp/TrackedObject.h
	${ROVER_DIR}/cpp/ObjectTracker.h
	${ROVER_DIR}/cpp/Rotation.h
	${ROVER_DIR}/cpp/Observer_Angular.h
	${ROVER_DIR}/cpp/Data.h
	${ROVER_DIR}/cpp/DataLogger.h
	${ROVER_DIR}/cpp/Common.h
	${ROVER_DIR}/cpp/Observer_Translational.h
	${ROVER_DIR}/cpp/Time.h
	${ROVER_DIR}/cpp/CommManager.h
	${ROVER_DIR}/cpp/FeatureFinder.h
	${ROVER_DIR}/cpp/TranslationController.h
	${ROVER_DIR}/cpp/AttitudeThrustController.h
	${ROVER_DIR}/cpp/MotorInterface.h
	${ROVER_DIR}/cpp/Listeners.h
#	${ROVER_DIR}/cpp/SensorManager.h
	${ROVER_DIR}/cpp/VideoMaker.h
	${ROVER_DIR}/cpp/constants.h
	${ROVER_DIR}/cpp/VelocityEstimator.h
	${ROVER_DIR}/cpp/TNT_Utils.h
	${ROVER_DIR}/cpp/ICSL/SystemModel/ISystemModel.h
	${ROVER_DIR}/cpp/ICSL/SystemModel/ISystemModelAffine.h
#	${ROVER_DIR}/cpp/ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h
)


set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")

add_definitions(-DICSL_OBSERVER_SIMULATION)
add_definitions(-DSCHED_NORMAL=0) # linux and android appear to have this defined on slightly different paths

# Executable
add_executable (observerTuning ${SRC} ${H})
target_link_libraries (observerTuning ${LIBS})
install (TARGETS observerTuning DESTINATION ${observerTuning_SOURCE_DIR}/libs/${NDK_NAME_ARCH})
