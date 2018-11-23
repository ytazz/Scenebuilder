cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

project(Scenebuilder)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

set(SPRINGHEAD_DIR "" CACHE PATH "root directory of Springhead")

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-invalid-offsetof -Wno-sign-compare -Wno-delete-non-virtual-dtor")

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${SPRINGHEAD_DIR}/core/include)
include_directories(${SPRINGHEAD_DIR}/core/src)

add_library(Scenebuilder
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
