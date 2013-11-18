# Standard setup
cmake_minimum_required (VERSION 2.6)
project (TargetFindTest CXX C)

# Source files
set (SRC ${SRC}
	${ROVER_DIR}/cpp/TargetFinder2.cpp
	${ROVER_DIR}/cpp/ActiveRegion.cpp
	)

# Headers
set (H ${H}
	${ROVER_DIR}/cpp/TargetFinder2.h
	${ROVER_DIR}/cpp/Time.h
	${ROVER_DIR}/cpp/TNT_Utils.h
	${ROVER_DIR}/cpp/Listeners.h
	)

add_definitions(-DICSL_TARGETFIND_SIMULATION)
add_definitions(-DSCHED_NORMAL=0) # linux and android appear to have this defined on slightly different paths

if(BUILD_ANDROID)
	set (SRC ${SRC}
		./cpp/android_main.cpp
		${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c
		)

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
		${TargetFindTest_SOURCE_DIR}/cpp
		${ROVER_DIR}/cpp
		${TOADLET_INCLUDE_DIR}
		${OpenCV_INCLUDE_DIRS}
		${MXML_DIR}/include
		${TBB_DIR}/include # note that the TBB lib comes in with OpenCV libs
		${ANDROID_NDK}/sources/android
		)
	include_directories (${INC})

	message("Include dirs: ")
	foreach(dir ${INC})
		message(${dir})
	endforeach()


	# Libraries
	set (LIBS  ${LIBS}
		${TOADLET_EGG_LIB} 
		${OpenCV_LIBS} 
		${MXML_DIR}/lib/${NDK_NAME_ARCH}/libmxml.a
		#	${TBB_DIR}/libs/${NDK_NAME_ARCH}/libtbb.a
		-lgnustl_static # this is already included by the toadlet toolchain, but the link order causes problems for OpenCV so adding it again here seems to help
		-lz -llog -ldl EGL GLESv1_CM android -lm
		)

	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti  -std=c++11")

	set (OpenCV_LIB_DIR_OPT "${OpenCV_LIB_DIR_OPT}/${NDK_NAME_ARCH}")

	message("libs: ")
	foreach(lib ${LIBS})
		message(${lib})
	endforeach()

	message("shared libs: ")
	foreach(lib ${SHARED_LIBS})
		message(${lib})
	endforeach()

	add_library (TargetFindTest SHARED ${SRC} ${H})
	target_link_libraries (TargetFindTest ${LIBS})
	install (TARGETS TargetFindTest DESTINATION ${TargetFindTest_SOURCE_DIR}/libs/${NDK_NAME_ARCH})
else()
	set (SRC ${SRC}
		./cpp/main.cpp
		)
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
		${TargetFindTest_SOURCE_DIR}/cpp
		${TOADLET_INCLUDE_DIR}
		${OpenCV_INCLUDE_DIRS}
		${ROVER_DIR}/cpp
		)
	include_directories (${INC})

	# Libraries
	set (LIBS 
		${TOADLET_EGG_LIB} 
		${OpenCV_LIBS_OPT} 
		)

	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

	# Executable
	add_executable (TargetFindTest ${SRC} ${H})
	target_link_libraries (TargetFindTest ${LIBS})
#	install (TARGETS TargetFindTest DESTINATION ${TargetFindTest_SOURCE_DIR}/libs/${NDK_NAME_ARCH})
endif()

