# Standard setup cmake_minimum_required (VERSION 2.6)
cmake_minimum_required (VERSION 2.6)
project (usbTest CXX C)
	set (LIBS ${LIBS} 
		 -lusb-1.0
		 )

# Includes
set (includeDirs ${includeDirs}
	${usbTest_SOURCE_DIR/src}
	${CMAKE_CURRENT_BINARY_DIR}
	)

# Source files
set (SRC ${SRC}
	src/listdevs.cpp
)

# Headers
set (H ${H}
)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)

include_directories (${includeDirs})
# Executable
add_executable (usbTest ${SRC} ${H} ${H_MOC} ${FORM_HEADERS} ${RES_RCC})
target_link_libraries (usbTest ${LIBS})
install (TARGETS usbTest DESTINATION ${usbTest_BINARY_DIR} )
