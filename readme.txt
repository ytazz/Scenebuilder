[ about Scenebuilder ]
Scenebuilder is a C++ library that provides various miscellaneous functionalities.
This library was basically developed for the author's own use, so it is not
designed friendly enough to be used by general users.

[ dependency ]
Scenebuilder is dependent on the following libraries.
You need to build and/or install them prior to building Scenebuilder.
- Springhead
  github.com/sprphys/Springhead
  see description there.

- Intel MKL
  Used as a replacement of Lapack for Windows environment.

[ how to build ]
Use CMake. Scenebuilder is mainly developed and tested on Windows + Visual Studio environment.
It should be not major problem in building it on linux, but there might be some platform-dependent
 issues such as MKL and Lapack. 
Also some classes for inter-process communication call Windows API; they are not fully migrated and
 tested on linux platform.

- Open CMake-gui, choose Scenebuilder's top directory as source directory and create and specify your 
  build directory (e.g. Scenebuilder/build).
- Set the following variables.
 SPRINGHEAD_DIR            top directory of Springhead (e.g. c:\Springhead)
 MKL_INCLUDE_DIR           include directory of MKL. something like:
   C:\Program Files (x86)\IntelSWTools\compilers_and_libraries_2017.2.187\windows\mkl\include
 CMAKE_INSTALL_PREFIX      where you would like to install Scenebuilder.
   recommended to set the same path as the other libraries' install directory.
 BUILD_DIMP_ADAPTOR        check if you would like to use the dimp-adaptor functionality of Scenebuilder.
   if you are building Scenebuilder for using DiMP, then probably you want check this option.

 If you have checked BUILD_DIMP_ADAPTOR, use will also need to set the following.
 DIMP_INCLUDE_DIR          include directory of DiMP.
   if you have installed DiMP using CMake, it is
   ${CMAKE_INSTALL_PREFIX}/include
   * note: be careful not to set ${CMAKE_INSTALL_PREFIX}/include/DiMP

[ note on building DiMP adaptor ]
Scenebuilder and DiMP are somewhat inter-dependent on each other, so the build and install order of
 these libraries are a little complicated.
Do it in the following order, if it is your first build.
- build/install Scenebuilder main library.
- build/install DiMP main library.
- build/install Scenebuilder's DiMP adaptor.
