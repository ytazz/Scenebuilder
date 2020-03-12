cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

project(Scenebuilder)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

if(NOT CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX d)
endif()

set(SPRINGHEAD_DIR   "" CACHE PATH "root directory of Springhead")
set(MKL_INCLUDE_DIR  "" CACHE PATH "include directory of MKL")

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

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/adaptors)
include_directories(${SPRINGHEAD_DIR}/core/include)
include_directories(${SPRINGHEAD_DIR}/core/src)
include_directories(${DIMP_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${GLEW_INCLUDE_DIR})
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
	include/sbikhandle.h
	include/sbikjoint.h
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
	src/sbikhandle.cpp
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

install(TARGETS Scenebuilder DESTINATION lib)
install(FILES ${SCENEBUILDER_HEADERS} DESTINATION include/Scenebuilder)

add_subdirectory(adaptors)

