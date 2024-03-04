# Install script for directory: /home/musashi/catkin_ws/MPCC/src/External/blasfeo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/lib")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/libblasfeo.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/blasfeoConfig.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/blasfeoConfig.cmake"
         "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/CMakeFiles/Export/cmake/blasfeoConfig.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/blasfeoConfig-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/cmake/blasfeoConfig.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cmake" TYPE FILE FILES "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/CMakeFiles/Export/cmake/blasfeoConfig.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cmake" TYPE FILE FILES "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/CMakeFiles/Export/cmake/blasfeoConfig-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_block_size.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_common.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_aux.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_aux_ext_dep.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_aux_ext_dep_ref.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_aux_old.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_aux_ref.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_aux_test.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_blas.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_blas_api.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_blasfeo_api.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_blasfeo_api_ref.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_blasfeo_hp_api.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_blasfeo_ref_api.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_d_kernel.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_i_aux_ext_dep.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_m_aux.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_memory.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_naming.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_processor_features.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_s_aux.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_s_aux_ext_dep.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_s_aux_ext_dep_ref.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_s_aux_old.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_s_aux_ref.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_s_aux_test.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_s_blas.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_s_blas_api.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_s_blasfeo_api.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_s_blasfeo_api_ref.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_s_blasfeo_ref_api.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_s_kernel.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_stdlib.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_target.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_timing.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/blasfeo_v_aux_ext_dep.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/d_blas.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/d_blas_64.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/s_blas.h"
    "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/include/s_blas_64.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
