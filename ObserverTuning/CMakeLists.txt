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
	${ROVER_DIR}/cpp/Rotation.cpp
	${ROVER_DIR}/cpp/CommManager.cpp
	${ROVER_DIR}/cpp/FeatureFinder.cpp
	${ROVER_DIR}/cpp/MotorInterface.cpp
	${ROVER_DIR}/cpp/VideoMaker.cpp
	${ROVER_DIR}/cpp/Observer_Angular.cpp
	${ROVER_DIR}/cpp/VelocityEstimator.cpp
	${ROVER_DIR}/cpp/ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.cpp
	${ROVER_DIR}/cpp/TargetFinder.cpp
	${ROVER_DIR}/cpp/TargetFinder2.cpp
	${ROVER_DIR}/cpp/Observer_Translational.cpp
	${ROVER_DIR}/cpp/QuadLogger.cpp
	${ROVER_DIR}/cpp/TranslationController.cpp
	${ROVER_DIR}/cpp/AttitudeThrustController.cpp
	${ROVER_DIR}/cpp/ActiveRegion.cpp
	# ${ROVER_DIR}/cpp/SensorManager.cpp
	./cpp/main.cpp
)

# Headers
set (H ${H}
	${ROVER_DIR}/cpp/Rotation.h
	${ROVER_DIR}/cpp/Observer_Angular.h
	${ROVER_DIR}/cpp/Data.h
	${ROVER_DIR}/cpp/QuadLogger.h
	${ROVER_DIR}/cpp/Common.h
	${ROVER_DIR}/cpp/Observer_Translational.h
	${ROVER_DIR}/cpp/Time.h
	${ROVER_DIR}/cpp/CommManager.h
	${ROVER_DIR}/cpp/TargetFinder.h
	${ROVER_DIR}/cpp/TargetFinder2.h
	${ROVER_DIR}/cpp/FeatureFinder.h
	${ROVER_DIR}/cpp/TranslationController.h
	${ROVER_DIR}/cpp/AttitudeThrustController.h
	${ROVER_DIR}/cpp/ActiveRegion.h
	${ROVER_DIR}/cpp/MotorInterface.h
	${ROVER_DIR}/cpp/Listeners.h
#	${ROVER_DIR}/cpp/SensorManager.h
	${ROVER_DIR}/cpp/VideoMaker.h
	${ROVER_DIR}/cpp/constants.h
	${ROVER_DIR}/cpp/VelocityEstimator.h
	${ROVER_DIR}/cpp/TNT_Utils.h
	${ROVER_DIR}/cpp/ICSL/SystemModel/ISystemModel.h
	${ROVER_DIR}/cpp/ICSL/SystemModel/ISystemModelAffine.h
	${ROVER_DIR}/cpp/ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h
)


set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")  # disable line-wrapping for error messages

add_definitions(-DICSL_OBSERVER_SIMULATION)
add_definitions(-DSCHED_NORMAL=0) # linux and android appear to have this defined on slightly different paths

# Executable
add_executable (observerTuning ${SRC} ${H})
target_link_libraries (observerTuning ${LIBS})
install (TARGETS observerTuning DESTINATION ${observerTuning_SOURCE_DIR}/libs/${NDK_NAME_ARCH})
