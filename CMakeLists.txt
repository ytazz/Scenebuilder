cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

project(Scenebuilder)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

if(NOT CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX d)
endif()

set(MKL_INCLUDE_DIR  "" CACHE PATH "include directory of MKL")
set(USE_CURSES false CACHE BOOL "use curses or not")

if(UNIX)
	set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-invalid-offsetof -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-reorder")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
	set(CMAKE_CXX_FLAGS_DEBUG "-g")
	set(CMAKE_CXX_FLAGS_RELEASE "-O2")
endif()

find_package(Boost REQUIRED COMPONENTS thread)
find_package(GLEW)
find_package(PNG)
find_package(EXPAT)
find_package(Springhead)

if(UNIX)
	find_package(Curses)
	add_compile_definitions(USE_CURSES)
endif()
if(WIN32 AND USE_CURSES)
	set(CURSES_INCLUDE_DIR  "" CACHE PATH "include directory of curses")
	include_directories(${CURSES_INCLUDE_DIR})
	add_compile_definitions(USE_CURSES)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/adaptors)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${PNG_PNG_INCLUDE_DIR})
include_directories(${EXPAT_INCLUDE_DIR})
include_directories(${MKL_INCLUDE_DIR})

set(BUILD_DIMP_ADAPTOR false CACHE BOOL "build dimp adaptor or not")

set(SCENEBUILDER_HEADERS
	include/sbadaptor.h
	include/sbbuffer.h
	include/sbbuilder.h
	include/sbcalc.h
	include/sbconsole.h
	include/sbconstraint.h
	include/sbconverter.h
	include/sbcriticalsection.h
	include/sbcsv.h
	include/sbevent.h
	include/sbikbody.h
	include/sbikjoint.h
	include/sbikbodyhandle.h
	include/sbikjointhandle.h
	include/sbikjointsync.h
	include/sbikcomhandle.h
	include/sbiksolver.h
	include/sbimage.h
	include/sbimagebmp.h
	include/sbimagepng.h
	include/sbimagesvg.h
	include/sblink.h
	include/sblogger.h
	include/sbmesh.h
	include/sbmessage.h
	include/sbmodel.h
	include/sbmodel3ds.h
	include/sbmodelobj.h
	include/sbmodelpcd.h
	include/sbmodelpmx.h
	include/sbmodelstl.h
	include/sbmutex.h
	include/sbpath.h
	include/sbrandom.h
	include/sbremote.h
	include/sbrollpitchyaw.h
	include/sbscene.h
	include/sbscenelocal.h
	include/sbsceneremote.h
	include/sbsceneshared.h
	include/sbsci.h
	include/sbshared.h
	include/sbsolver.h
	include/sbtcp.h
	include/sbthread.h
	include/sbtimer.h
	include/sbtokenizer.h
	include/sbtree.h
	include/sbtypedb.h
	include/sbtypes.h
	include/sbudp.h
	include/sbvariable.h
	include/sbxml.h
)

set(SCENEBUILDER_SOURCES
	src/sbadaptor.cpp
	src/sbbuilder.cpp
	src/sbcalc.cpp
	src/sbconsole.cpp
	src/sbconstraint.cpp
	src/sbconverter.cpp
	src/sbcriticalsection.cpp
	src/sbevent.cpp
	src/sbikbody.cpp
	src/sbikbodyhandle.cpp
	src/sbikjointhandle.cpp
	src/sbikjointsync.cpp
	src/sbikcomhandle.cpp
	src/sbikjoint.cpp
	src/sbiksolver.cpp
	src/sbimage.cpp
	src/sbimagebmp.cpp
	src/sbimagepng.cpp
	src/sbimagesvg.cpp
	src/sblink.cpp
	src/sblogger.cpp
	src/sbmesh.cpp
	src/sbmessage.cpp
	src/sbmodel.cpp
	src/sbmodel3ds.cpp
	src/sbmodelobj.cpp
	src/sbmodelpcd.cpp
	src/sbmodelpmx.cpp
	src/sbmodelstl.cpp
	src/sbmutex.cpp
	src/sbpath.cpp
	src/sbrandom.cpp
	src/sbremote.cpp
	src/sbrollpitchyaw.cpp
	src/sbscene.cpp
	src/sbscenelocal.cpp
	src/sbsceneremote.cpp
	src/sbsceneshared.cpp
	src/sbsci.cpp
	src/sbshared.cpp
	src/sbsolver.cpp
	src/sbsolverddp.cpp
	src/sbtcp.cpp
	src/sbthread.cpp
	src/sbtimer.cpp
	src/sbtokenizer.cpp
	src/sbtree.cpp
	src/sbtypedb.cpp
	src/sbtypes.cpp
	src/sbudp.cpp
	src/sbvariable.cpp
	src/sbxml.cpp
)

add_library(Scenebuilder ${SCENEBUILDER_SOURCES} ${SCENEBUILDER_HEADERS})

set_target_properties(Scenebuilder PROPERTIES PUBLIC_HEADER "${SCENEBUILDER_HEADERS}")

target_link_libraries(Scenebuilder Springhead::Springhead)
target_link_libraries(Scenebuilder GLEW::GLEW)
if(UNIX)
	target_link_libraries(Scenebuilder Curses)
endif()

install(TARGETS Scenebuilder
  EXPORT        ScenebuilderExport
  LIBRARY       DESTINATION lib
  INCLUDES      DESTINATION include/Scenebuilder
  PUBLIC_HEADER DESTINATION include/Scenebuilder
  )
  
install(EXPORT ScenebuilderExport
  FILE         ScenebuilderConfig.cmake
  DESTINATION  cmake
  )

add_subdirectory(adaptors)

