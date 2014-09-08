if (DEFINED ENV{ROS_ROOT})

  cmake_minimum_required(VERSION 2.4.6)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
  rosbuild_add_boost_directories()
  rosbuild_add_library(manipulation manipulationmain.cpp manipulation.cpp trajectoryproblem.cpp)
  rosbuild_link_boost(manipulation regex)

else (DEFINED ENV{ROS_ROOT})

  cmake_minimum_required (VERSION 2.4)
  project (manipulation)

  include(FindOpenRAVE.cmake)

  if( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
    set(PLUGIN_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../plugins")
  else( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
    set(PLUGIN_INSTALL_DIR "$ENV{PLUGIN_INSTALL_DIR}")
  endif( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )

  if( NOT OPENRAVE_FOUND )
    message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
  endif( NOT OPENRAVE_FOUND )

  if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
    add_definitions("-fno-strict-aliasing ")
  endif()

  find_package(Boost COMPONENTS regex)

  add_library(manipulation SHARED manipulationmain.cpp manipulation.cpp trajectoryproblem.cpp)
  set_target_properties(manipulation PROPERTIES
                        LINK_FLAGS ${OPENRAVE_LINK_FLAGS}
                        COMPILE_FLAGS ${OPENRAVE_CXXFLAGS})
  include_directories(${OPENRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  link_directories(${OPENRAVE_LINK_DIRS} ${Boost_LIBRARY_DIRS})
  target_link_libraries(manipulation ${OPENRAVE_LIBRARY} ${Boost_LIBRARIES})

  install(TARGETS manipulation DESTINATION ${PLUGIN_INSTALL_DIR} )

endif (DEFINED ENV{ROS_ROOT})
