# Standard setup cmake_minimum_required (VERSION 2.6)
cmake_minimum_required (VERSION 2.6)
project (leash CXX C)
IF( ${UNIX})
	set(ICSL_ICSL_DIR "/home/ryantr/Software/ICSL" CACHE PATH " ")
	set(ICSL_EXTERNALSOFTWARE_DIR "/home/ryantr/Software/ExternalSoftware" CACHE PATH " ")
	set(ICSL_QUADROTOR_DIR "/home/ryantr/Software/Quadrotor" CACHE PATH " ")
	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};/home/ryantr/toadlet/cmake/Modules CACHE PATH " ")
	set(ICSL_CNTL_TIMER "Software" CACHE STRING " ")

	set (LIBS ${LIBS} 
		 -lmxml
		 -lrt
		 -ldl
		 )
ELSE()
	set(ICSL_ICSL_DIR "C:/Users/ICSL/Desktop/Tyler/ICSL" CACHE PATH " ")
	set(ICSL_EXTERNALSOFTWARE_DIR "C:/Users/ICSL/Desktop/Tyler/ExternalSoftware" CACHE PATH " ")
	set(ICSL_QUADROTOR_DIR "C:/Users/ICSL/Desktop/Tyler/Quadrotor" CACHE PATH " ")
	set(CMAKE_MODULE_PATH "C:/Users/ICSL/Desktop/Tyler/Toadlet/cmake/Modules" CACHE PATH " ")
	set (OpenCV_DIR "C:/opencv/2.4 vs9/install" CACHE PATH " ")
	set(ICSL_CNTL_TIMER "DAQNavi" CACHE STRING " ")
ENDIF()
	set_property(CACHE ICSL_CNTL_TIMER PROPERTY STRINGS "Software" "DAQNavi")

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

find_package (toadlet REQUIRED)
find_package (OpenCV REQUIRED core highgui imgproc)
message("OpenCV include: " )
foreach (dir ${OpenCV_INCLUDE_DIRS})
	message("\t" ${dir})
endforeach()
message("OpenCV libs: ")
foreach (lib ${OpenCV_LIBS})
	message("\t" ${lib})
endforeach()
set(LIBS ${LIBS}
	${TOADLET_EGG_LIB}
#	${TOADLET_LIBRARIES}
#	${TOADLET_LIBRARY_DIR}/libtoadlet_egg.so
	${OpenCV_LIBS}
	${QT_LIBRARIES}
	)

# Includes
set (includeDirs ${includeDirs}
	${leash_SOURCE_DIR/src}
	${ICSL_ICSL_DIR}
	${ICSL_QUADROTOR_DIR}
	${ICSL_EXTERNALSOFTWARE_DIR}
	${TOADLET_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}
	)

# Source files
set (SRC ${SRC}
	src/main.cpp
	src/FlightInterface.cpp
#	src/SystemControllerHierarchical.cpp
#	src/SystemControllerFeedbackLin.cpp
#	src/QuadrotorInterface.cpp
	src/Leash.cpp
	${ICSL_QUADROTOR_DIR}/Quadrotor/TelemetryVicon/src/TelemetryVicon.cpp
	${ICSL_QUADROTOR_DIR}/Quadrotor/SystemModelQuadrotor/src/SystemModelQuadrotor.cpp
)

# Headers
set (H ${H}
	src/FlightInterface.h
#	src/SystemControllerHierarchical.h
#	src/SystemControllerFeedbackLin.h
#	src/QuadrotorInterface.h
	src/Leash.h
	../Rover/cpp/Common.h
	${ICSL_QUADROTOR_DIR}/Quadrotor/TelemetryVicon/src/TelemetryVicon.h
	${ICSL_QUADROTOR_DIR}/Quadrotor/SystemModelQuadrotor/src/SystemModelQuadrotor.h
)

set (FORMS ${FORMS}
	src/FlightInterface.ui
#	src/QuadrotorInterface.ui
	src/Leash.ui
	)

set (RES ${RES})

INCLUDE(${ICSL_ICSL_DIR}/ICSL/ControlTimer/headers.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/ControlTimer/source.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/ControlTimer/libraries.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/MeasurementSystems/headers.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/MeasurementSystems/source.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/MeasurementSystems/libraries.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/SystemModel/headers.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/SystemModel/source.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/SystemModel/libraries.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/Timer/headers.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/Timer/source.cmake)
#INCLUDE(${ICSL_ICSL_DIR}/ICSL/Timer/libraries.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/TNT_Utils/headers.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/xml_utils/headers.cmake)

set (SRC ${SRC} ${ICSL_SRC})
set (H ${H} ${ICSL_H})
set (LIBS ${LIBS} ${ICSL_LIBS})

message("ICSL_LIBS")
foreach (LIB ${ICSL_LIBS})
	message("\t" ${LIB})
endforeach()

INCLUDE(${ICSL_EXTERNALSOFTWARE_DIR}/TNT/headers.cmake)
INCLUDE(${ICSL_EXTERNALSOFTWARE_DIR}/Vicon/headers.cmake)
INCLUDE(${ICSL_EXTERNALSOFTWARE_DIR}/Vicon/libraries.cmake)
INCLUDE(${ICSL_EXTERNALSOFTWARE_DIR}/mxml/headers.cmake)
INCLUDE(${ICSL_EXTERNALSOFTWARE_DIR}/mxml/libraries.cmake)
set (H ${H} ${EXT_H})
set (LIBS ${LIBS} ${EXT_LIBS})

message("EXT_LIBS")
foreach (LIB ${EXT_LIBS})
	message("\t" ${LIB})
endforeach()

# Create QT intermediates
QT4_WRAP_CPP(H_MOC ${H})
QT4_WRAP_UI(FORM_HEADERS ${FORMS})
QT4_ADD_RESOURCES(RES_RCC ${RES})

SOURCE_GROUP(QuadRover\\Leash FILES ./src/main.cpp)
SOURCE_GROUP(QuadRover\\Leash FILES ./src/FlightInterface.ui)
SOURCE_GROUP(QuadRover\\Leash FILES ./src/FlightInterface.h)
SOURCE_GROUP(QuadRover\\Leash FILES ./src/FlightInterface.cpp)
# SOURCE_GROUP(QuadRover\\Leash FILES ./src/QuadrotorInterface.ui)
# SOURCE_GROUP(QuadRover\\Leash FILES ./src/QuadrotorInterface.h)
# SOURCE_GROUP(QuadRover\\Leash FILES ./src/QuadrotorInterface.cpp)
SOURCE_GROUP(QuadRover\\Leash FILES ./src/Leash.ui)
SOURCE_GROUP(QuadRover\\Leash FILES ./src/Leash.h)
SOURCE_GROUP(QuadRover\\Leash FILES ./src/Leash.cpp)
SOURCE_GROUP(QuadRover\\Leash FILES ./src/SystemControllerHierarchical.h)
SOURCE_GROUP(QuadRover\\Leash FILES ./src/SystemControllerHierarchical.cpp)
SOURCE_GROUP(QuadRover\\Leash FILES ./src/SystemControllerFeedbackLin.h)
SOURCE_GROUP(QuadRover\\Leash FILES ./src/SystemControllerFeedbackLin.cpp)
SOURCE_GROUP(Quadrotor\\SystemModelQuadrotor ${ICSL_QUADROTOR_DIR}/Quadrotor/SystemModelQuadrotor/.*)
SOURCE_GROUP(Quadrotor\\Telemetry\\Vicon ${ICSL_QUADROTOR_DIR}/Quadrotor/TelemetryVicon/.*)
SOURCE_GROUP(Quadrotor\\Moc ../.*moc.*)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreorder -Wunused-but-set-variable -Wunused-variable -Wsign-compare")
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

include_directories (${includeDirs})
# Executable
add_executable (leash ${SRC} ${H} ${H_MOC} ${FORM_HEADERS} ${RES_RCC})
target_link_libraries (leash ${LIBS})
install (TARGETS leash DESTINATION ${leash_BINARY_DIR} )
