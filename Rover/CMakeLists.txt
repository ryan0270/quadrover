# Standard setup
cmake_minimum_required (VERSION 2.6)
project (QuadPhone CXX C)

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
	${QuadPhone_SOURCE_DIR}/cpp
	${TOADLET_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${MXML_DIR}/include
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
	${MXML_DIR}/lib/${NDK_NAME_ARCH}/libmxml.a
	-lz -llog -ldl)
if (${ANDROID_NDK_API_LEVEL} GREATER 8)
	SET (LIBS ${LIBS} EGL android)
endif (${ANDROID_NDK_API_LEVEL} GREATER 8) 

# Source files
set (SRC ${SRC}
	jni/jni_part.cpp
	./cpp/QuadPhone.cpp
	./cpp/Observer_Angular.cpp
	./cpp/Observer_Translational.cpp
	./cpp/QuadLogger.cpp
	./cpp/CommManager.cpp
	./cpp/VisionProcessor.cpp
#	./cpp/SystemControllerFeedbackLin.cpp
	./cpp/TranslationController.cpp
	./cpp/AttitudeThrustController.cpp
	./cpp/MotorInterface.cpp
	./cpp/ICSL/SisoController/SisoControllerPID/src/SisoControllerPID.cpp
	./cpp/ICSL/SystemControllerPID/src/SystemControllerPID.cpp
	./cpp/ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.cpp
)

# Headers
set (H ${H}
	./cpp/QuadPhone.h
	./cpp/Observer_Angular.h
	./cpp/Observer_Translational.h
	./cpp/QuadLogger.h
	./cpp/Common.h
	./cpp/Time.h
	./cpp/CommManager.h
	./cpp/VisionProcessor.h
#	./cpp/SystemControllerFeedbackLin.h
	./cpp/TranslationController.h
	./cpp/AttitudeThrustController.h
	./cpp/AttitudeThrustControllerListener.h
	./cpp/MotorInterface.h
	./cpp/ICSL/SisoController/SisoControllerPID/src/SisoControllerPID.h
	./cpp/ICSL/SystemControllerPID/src/SystemControllerPID.h
	./cpp/ICSL/SystemModel/ISystemModel.h
	./cpp/ICSL/SystemModel/ISystemModelAffine.h
	./cpp/ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -fno-exceptions")

set (OpenCV_LIB_DIR_OPT "${OpenCV_LIB_DIR_OPT}/${NDK_NAME_ARCH}")
# Why doesn't this work?
#if (COMMAND COPY_NATIVE_CAMERA_LIBS)
#	message("Copying opencv camera libs to ${QuadPhone_SOURCE_DIR}/libs/${NDK_NAME_ARCH}")
#	COPY_NATIVE_CAMERA_LIBS(${QuadPhone})
#else()
#	message("Failed to copy opencv camera libs: command not found")
#endif()


# Executable
add_library (QuadPhone SHARED ${SRC} ${H})
target_link_libraries (QuadPhone ${LIBS})
install (TARGETS QuadPhone DESTINATION ${QuadPhone_SOURCE_DIR}/libs/${NDK_NAME_ARCH})

#if (${ANDROID_NDK_API_LEVEL} LESS 9)
	install (FILES ${JTOADLET_FLICK_JAR} ${JTOADLET_PEEPER_JAR} ${JTOADLET_RIBBIT_JAR} ${JTOADLET_PAD_JAR} ${JTOADLET_TADPOLE_JAR} DESTINATION ${QuadPhone_SOURCE_DIR}/libs)
#endif (${ANDROID_NDK_API_LEVEL} LESS 9)
