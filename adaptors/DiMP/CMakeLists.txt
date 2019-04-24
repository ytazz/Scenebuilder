cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

if(BUILD_DIMP_ADAPTOR)
	set(DIMP_INCLUDE_DIR "" CACHE PATH "include directory of DiMP")

	include_directories(${DIMP_INCLUDE_DIR})

	set(SCENEBUILDER_DIMP_HEADERS
		sbdimp.h
	)
	set(SCENEBUILDER_DIMP_SOURCES
		sbdimp.cpp
	)

	add_library(sbdimp
	  ${SCENEBUILDER_DIMP_SOURCES}
	)

	install(TARGETS sbdimp DESTINATION lib)
	install(FILES ${SCENEBUILDER_DIMP_HEADERS} DESTINATION include/Scenebuilder/DiMP)

endif()

