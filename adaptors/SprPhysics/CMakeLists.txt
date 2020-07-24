cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

set(SCENEBUILDER_SPRPHYSICS_HEADERS
	sbsprphysics.h
)
set(SCENEBUILDER_SPRPHYSICS_SOURCES
	sbsprphysics.cpp
)

find_package(dimp)

add_library(sbsprphysics
  ${SCENEBUILDER_SPRPHYSICS_SOURCES}
  ${SCENEBUILDER_SPRPHYSICS_HEADERS}
)
target_link_libraries(dimp)

install(TARGETS sbsprphysics
	EXPORT   sbsprphysicsExport
	LIBRARY  DESTINATION lib
)
install(FILES
	${SCENEBUILDER_SPRPHYSICS_HEADERS} DESTINATION include/Scenebuilder/SprPhysics
)

install(EXPORT sbsprphysicsExport
	FILE         sbsprphysicsConfig.cmake
	DESTINATION  cmake
)