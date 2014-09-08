# Check that:
#   PATH includes openrave-config
#   PYTHONPATH includes `openrave-config --python-dir`
#   OPENRAVE_DATA includes robot file
#   PLUGIN_INSTALL_DIR=.

cmake_minimum_required (VERSION 2.4)
project (Herb2IK)

include(FindOpenRAVE.cmake)

if( NOT OPENRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )
exec_program(openrave-config ARGS --version OUTPUT_VARIABLE ORVERS)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing ")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# The robot xml file we want to generate the iksolver for.
set(ROBOTXML "robots/herb2_padded.robot.xml")
## Generate robothash by running 
## openrave-hash.py robots/herb2_padded.robot.xml --robothash
##exec_program(openrave-hash.py ARGS "${ROBOTXML} --robothash" OUTPUT_VARIABLE ROBOTHASH)
if (ORVERS VERSION_LESS "0.4.2")
  exec_program(openrave-hash.py ARGS "--kinematics ${ROBOTXML} --manipname=left_wam" OUTPUT_VARIABLE LEFTWAMHASH)
  exec_program(openrave-hash.py ARGS "--kinematics ${ROBOTXML} --manipname=right_wam" OUTPUT_VARIABLE RIGHTWAMHASH)
else (ORVERS VERSION_LESS "0.4.2")
  exec_program(openrave-robot.py ARGS "--hash=kinematics ${ROBOTXML} --manipname=left_wam" OUTPUT_VARIABLE LEFTWAMHASH)
  exec_program(openrave-robot.py ARGS "--hash=kinematics ${ROBOTXML} --manipname=right_wam" OUTPUT_VARIABLE RIGHTWAMHASH)
endif (ORVERS VERSION_LESS "0.4.2")
## Below are the latest hash values for herb2_padded.robot.xml and its manipulators, in case the code above does not work. 
## But hashes seem to change a lot, so don't depend on this hard coded value.
#set(ROBOTHASH "377cd773d208a72bddc8a263bd299dc3")
##set(LEFTWAMHASH "13f26efb7bf7ceb93b1eff57bd52841c")
##set(RIGHTWAMHASH "944ceafced7b35dacb1ccbcae0c5b112")
#set(ROBOTDIRECTORY "robot.${ROBOTHASH}")
#message("Trying to see if the cpp files are already generated:")
#exec_program(ls ARGS "${CMAKE_CURRENT_SOURCE_DIR}/${ROBOTDIRECTORY}/*${LEFTWAMHASH}*.cpp" OUTPUT_VARIABLE LEFTARMCPP)
#exec_program(ls ARGS "${CMAKE_CURRENT_SOURCE_DIR}/${ROBOTDIRECTORY}/*${RIGHTWAMHASH}*.cpp" OUTPUT_VARIABLE RIGHTARMCPP)
#
#exec_program(openrave-config ARGS "--python-dir" OUTPUT_VARIABLE ORINSTALLATION)
#set(IKFASTPYTHON "${ORINSTALLATION}/openravepy/databases/inversekinematics.py")
#message("${IKFASTPYTHON}")

if (ORVERS VERSION_LESS "0.4.2")
  exec_program(openrave-config ARGS "--python-dir" OUTPUT_VARIABLE ORINSTALLATION)
  set(IKFASTPYTHON "${ORINSTALLATION}/openravepy/databases/inversekinematics.py")
  message("${IKFASTPYTHON}")
endif (ORVERS VERSION_LESS "0.4.2")

set(LEFTARMCPP "./leftarm_ikfast31_2011_02_17.cpp")
set(RIGHTARMCPP "./rightarm_ikfast31_2011_02_17.cpp")

if( NOT EXISTS "${LEFTARMCPP}" )
  message("The ikfast cpp file for left arm does not exist. Will attempt to generate it. This takes ~10 minutes.")
  # Set the OPENRAVE_HOME environment variable the current directory
  # so that the ikfast code is created here
  set(ENV{OPENRAVE_HOME} ${CMAKE_CURRENT_SOURCE_DIR})
  # call the inversekinematics script for left wam
  if (ORVERS VERSION_LESS "0.4.2")
    exec_program(python ARGS "${IKFASTPYTHON} --robot=${ROBOTXML} --manipname=left_wam")
  else (ORVERS VERSION_LESS "0.4.2")
    exec_program("openrave.py --database inversekinematics --robot=${ROBOTXML} --manipname=left_wam")
  endif (ORVERS VERSION_LESS "0.4.2")
  # get the name of the file
  exec_program(ls ARGS "${CMAKE_CURRENT_SOURCE_DIR}/kinematics.${LEFTWAMHASH}/*.cpp" OUTPUT_VARIABLE LEFTARMCPP)
else ( NOT EXISTS "${LEFTARMCPP}" )
  message("The file ${LEFTARMCPP} already exists. If you want it regenerated, first run make clean to clean these files and force a regeneration.")
endif( NOT EXISTS "${LEFTARMCPP}" )

if( NOT EXISTS "${RIGHTARMCPP}" )
  message("The ikfast cpp file for right arm does not exist. Will attempt to generate it. This takes ~10 minutes.")
  # Set the OPENRAVE_HOME environment variable the current directory
  # so that the ikfast code is created here
  set(ENV{OPENRAVE_HOME} ${CMAKE_CURRENT_SOURCE_DIR})
  # call the inversekinematics script for right wam
  if (ORVERS VERSION_LESS "0.4.2")
    exec_program(python ARGS "${IKFASTPYTHON} --robot=${ROBOTXML} --manipname=right_wam")
  else (ORVERS VERSION_LESS "0.4.2")
    exec_program("openrave.py --database inversekinematics --robot=${ROBOTXML} --manipname=right_wam")
  endif (ORVERS VERSION_LESS "0.4.2")
  # get the name of the file
  exec_program(ls ARGS "${CMAKE_CURRENT_SOURCE_DIR}/kinematics.${RIGHTWAMHASH}/*.cpp" OUTPUT_VARIABLE RIGHTARMCPP)
else ( NOT EXISTS "${RIGHTARMCPP}" )
  message("The file ${RIGHTARMCPP} already exists. If you want it regenerated, first run make clean to clean these files and force a regeneration.")
endif( NOT EXISTS "${RIGHTARMCPP}" )

# replace the filename patterns to produce the actual file
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/ikfastsolvers.cpp.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/ikfastsolvers.cpp"
)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/${ROBOTDIRECTORY}")
#set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${LEFTARMCPP};${RIGHTARMCPP};${CMAKE_CURRENT_SOURCE_DIR}/ikfastsolvers.cpp") 
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/ikfastsolvers.cpp") 
#set_source_files_properties(ikfastsolvers.cpp PROPERTIES OBJECT_DEPENDS "${LEFTARMCPP} ${RIGHTARMCPP}")

link_directories(${OPENRAVE_LINK_DIRS})
add_library(Herb2IK SHARED ikfastsolvers.cpp)
set_target_properties(Herb2IK PROPERTIES
                      LINK_FLAGS ${OPENRAVE_LINK_FLAGS}
                      COMPILE_FLAGS ${OPENRAVE_CXXFLAGS})
include_directories(${OPENRAVE_INCLUDE_DIRS})
target_link_libraries(Herb2IK ${OPENRAVE_LIBRARY})

if( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
  set(PLUGIN_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../plugins")
else( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
  set(PLUGIN_INSTALL_DIR "$ENV{PLUGIN_INSTALL_DIR}")
endif( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )

install(TARGETS Herb2IK DESTINATION ${PLUGIN_INSTALL_DIR} )

