# Standard setup cmake_minimum_required (VERSION 2.6)
cmake_minimum_required (VERSION 2.6)
project (leash CXX C)
IF( ${UNIX})
	set(ICSL_ICSL_DIR "/home/ryantr/Software/ICSL" CACHE PATH " ")
	set(ICSL_EXTERNALSOFTWARE_DIR "/home/ryantr/Software/ExternalSoftware" CACHE PATH " ")
	set(ICSL_QUADROVER_DIR "/home/ryantr/Software/QuadRover" CACHE PATH " ")
	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};/home/ryantr/toadlet/cmake/Modules CACHE PATH " ")

	set (LIBS ${LIBS} 
		 -lmxml
		 -lrt
		 -ldl
		 )
ELSE()
	set(ICSL_ICSL_DIR "C:/Users/ICSL/Desktop/Tyler/ICSL" CACHE PATH " ")
	set(ICSL_EXTERNALSOFTWARE_DIR "C:/Users/ICSL/Desktop/Tyler/ExternalSoftware" CACHE PATH " ")
	set(ICSL_QUADROVER_DIR "C:/Users/ICSL/Desktop/Tyler/QuadRover" CACHE PATH " ")
	set(CMAKE_MODULE_PATH "C:/Users/ICSL/Desktop/Tyler/Toadlet/cmake/Modules" CACHE PATH " ")
	set (OpenCV_DIR "C:/opencv-2.4.3/vs9/install" CACHE PATH " ")
ENDIF()

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
	${OpenCV_LIBS}
	${QT_LIBRARIES}
	)

# Includes
set (includeDirs ${includeDirs}
	${leash_SOURCE_DIR/src}
	${ICSL_ICSL_DIR}
	${ICSL_QUADROVER_DIR}
	${ICSL_EXTERNALSOFTWARE_DIR}
	${TOADLET_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}
	)

# Source files
set (SRC ${SRC}
	src/main.cpp
	src/Leash.cpp
	${ICSL_QUADROVER_DIR}/TelemetryVicon/src/TelemetryVicon.cpp
)

# Headers
set (H ${H}
	src/Leash.h
	../Rover/cpp/Common.h
	${ICSL_QUADROVER_DIR}/TelemetryVicon/src/TelemetryVicon.h
)

set (FORMS ${FORMS}
	src/Leash.ui
	)

set (RES ${RES})

INCLUDE(${ICSL_ICSL_DIR}/ICSL/MeasurementSystems/headers.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/MeasurementSystems/source.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/MeasurementSystems/libraries.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/SystemModel/headers.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/SystemModel/source.cmake)
INCLUDE(${ICSL_ICSL_DIR}/ICSL/SystemModel/libraries.cmake)
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
SOURCE_GROUP(QuadRover\\Leash FILES ./src/Leash.ui)
SOURCE_GROUP(QuadRover\\Leash FILES ./src/Leash.h)
SOURCE_GROUP(QuadRover\\Leash FILES ./src/Leash.cpp)
SOURCE_GROUP(TelemetryVicon ${ICSL_QUADROVER_DIR}/TelemetryVicon/.*)
SOURCE_GROUP(QuadRover\\Moc ../.*moc.*)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreorder -Wunused-but-set-variable -Wunused-variable -Wsign-compare")
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

include_directories (${includeDirs})
# Executable
add_executable (leash ${SRC} ${H} ${H_MOC} ${FORM_HEADERS} ${RES_RCC})
target_link_libraries (leash ${LIBS})
install (TARGETS leash DESTINATION ${leash_BINARY_DIR} )
