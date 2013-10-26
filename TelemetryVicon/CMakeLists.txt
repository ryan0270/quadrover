# Standard setup cmake_minimum_required (VERSION 2.6)
cmake_minimum_required (VERSION 2.6)
project (telemViconExample CXX C)
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

INCLUDE ($ENV{ICSL_SOFTWARE_ROOT}/SetIcslParams.cmake)
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

# Includes
set (includeDirs ${includeDirs}
	${telemViconExample_SOURCE_DIR/src}
	${ICSL_QUADROTOR_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
	)
include_directories (${telemViconExample_SOURCE_DIR} ${includeDirs})

# Source files
set (SRC ${SRC}
	src/main.cpp
	src/TelemetryVicon.cpp
	src/TelemetryViconDisplay.cpp
)

# Headers
set (H ${H}
	src/TelemetryVicon.h
	src/TelemetryViconDisplay.h
	${ICSL_QUADROTOR_DIR}/quadrotor_config.h
)

set (FORMS ${FORMS}
	src/TelemetryViconDisplay.ui
	)

set (LIBS ${LIBS}
	${QT_LIBRARIES}
	)

# set (RES ${RES})

INCLUDE(${ICSL_ICSL_DIR}/headers.cmake)
INCLUDE(${ICSL_ICSL_DIR}/source.cmake)
INCLUDE(${ICSL_ICSL_DIR}/libraries.cmake)
set (SRC ${SRC} ${ICSL_SRC})
set (H ${H} ${ICSL_H})
set (LIBS ${LIBS} ${ICSL_LIBS})

INCLUDE(${ICSL_EXTERNALSOFTWARE_DIR}/headers.cmake)
INCLUDE(${ICSL_EXTERNALSOFTWARE_DIR}/libraries.cmake)
set (H ${H} ${EXT_H})
set (LIBS ${LIBS} ${EXT_LIBS})

# Create QT intermediates
QT4_WRAP_CPP(H_MOC ${H})
QT4_WRAP_UI(FORM_HEADERS ${FORMS})
# QT4_ADD_RESOURCES(RES_RCC ${RES})

SOURCE_GROUP(Quadrotor FILES ${ICSL_QUADROTOR_DIR}/quadrotor_config.h)
SOURCE_GROUP(Quadrotor\\TelemetryVicon ${ICSL_QUADROTOR_DIR}/TelemetryVicon/.*[.]h)
SOURCE_GROUP(Quadrotor\\Moc ${ICSL_QUADROTOR_DIR}/TelemetryVicon/.*moc.*)

# Executable
add_executable (telemViconExample ${SRC} ${H} ${H_MOC} ${FORM_HEADERS} ${RES_RCC})
target_link_libraries (telemViconExample ${LIBS})
install (TARGETS telemViconExample DESTINATION ${telemViconExample_BINARY_DIR} )
