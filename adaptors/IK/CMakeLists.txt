cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

set(SCENEBUILDER_IK_HEADERS
	sbik.h
)
set(SCENEBUILDER_IK_SOURCES
	sbik.cpp
)

add_library(sbik
  ${SCENEBUILDER_IK_SOURCES}
)

install(TARGETS sbik DESTINATION lib)
install(FILES ${SCENEBUILDER_IK_HEADERS} DESTINATION include/Scenebuilder/IK)
