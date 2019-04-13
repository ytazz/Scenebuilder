cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

set(SCENEBUILDER_SPRGRAPHICS_HEADERS
	sbsprgraphics.h
)
set(SCENEBUILDER_SPRGRAPHICS_SOURCES
	sbsprgraphics.cpp
)

add_library(sbsprgraphics
  ${SCENEBUILDER_SPRGRAPHICS_SOURCES}
)

install(TARGETS sbsprgraphics DESTINATION lib)
install(FILES ${SCENEBUILDER_SPRGRAPHICS_HEADERS} DESTINATION include/Scenebuilder/SprGraphics)
