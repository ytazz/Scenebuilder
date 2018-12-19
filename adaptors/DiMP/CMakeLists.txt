cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

if(BUILD_DIMP_ADAPTOR)
	set(DIMP_DIR "" CACHE PATH "root directory of dimp")

	include_directories(${DIMP_DIR}/src)

	add_library(sbdimp
	  sbdimp.cpp
	)
endif()

