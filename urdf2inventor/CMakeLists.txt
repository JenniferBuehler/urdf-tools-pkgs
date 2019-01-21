cmake_minimum_required(VERSION 2.8.3)
project(urdf2inventor)

add_compile_options(-std=c++11)

set (CATKIN_PKGS
  roscpp
  urdf
  eigen_conversions
  baselib_binding
  urdf_traverser
  urdf_transform
  roslint
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

## System dependencies are found with CMake's conventions
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeMacros/")
find_package(SoQt4 REQUIRED)
find_package(Coin REQUIRED)
find_package(Qt4 REQUIRED)
find_package(assimp REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

# list of catkin packages this package depends on, to be put in catkin_package CATKIN_DEPENDS
set (CATKIN_DEPEND_PKGS roscpp urdf eigen_conversions baselib_binding urdf_traverser urdf_transform)
# list of cmake projects this package depends on, to be put in catkin_package DEPENDS
set (CATKIN_DEPENDS_LIBS SoQt4 Qt4 Coin assimp Boost) # libsoqt4-dev libqt4-dev libcoin80-dev assimp-dev)

# Libraries this project depends on, and which also need to be exported
# because depending projects will require them.
set (DEPEND_LIBRARIES
    ${SOQT_LIBRARY}
    ${QT_LIBRARIES}
    ${ASSIMP_LIBRARY}
    ${ASSIMP_LIBRARIES}
    ${Boost_LIBRARIES}
)


# directories to include and to export with catkin_package
set (URDF2INVENTOR_INCLUDE_DIRS
  include
  ${Boost_INCLUDE_DIRS}
  ${assimp_INCLUDE_DIRS}
  ${SOQt_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
  ${QT_INCLUDE_DIR}/Qt
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${URDF2INVENTOR_INCLUDE_DIRS} 
  LIBRARIES urdf2inventor ${DEPEND_LIBRARIES} # if DEPEND_LIBRARIES not added here, binaries in bloom will fail
  CATKIN_DEPENDS ${CATKIN_DEPEND_PKGS} 
  DEPENDS ${CATKIN_DEPENDS_LIBS} 
  CFG_EXTRAS urdf2inventor_definitions.cmake
)

###########
## Build ##
###########

add_definitions(${baselib_binding_DEFINITIONS} ${urdf_traverser_DEFINITIONS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
  ${URDF2INVENTOR_INCLUDE_DIRS}
)

set (TARGETLINK_LIBRARIES
    ${catkin_LIBRARIES}
    ${DEPEND_LIBRARIES}
)

## Declare a C++ library
add_library(urdf2inventor
  src/Urdf2Inventor.cpp
  src/Helpers.cpp
  src/IVHelpers.cpp
  src/ConvertMesh.cpp
  src/AssimpImport.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(urdf2inventor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(urdf2inventor_node src/urdf2inventor_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(urdf2inventor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

include (cmake/urdf2inventor_definitions.cmake)

## Specify libraries to link a library or executable target against
target_link_libraries(urdf2inventor ${TARGETLINK_LIBRARIES})
target_link_libraries(urdf2inventor_node urdf2inventor ${TARGETLINK_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS urdf2inventor urdf2inventor_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY CMakeMacros/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
    FILES_MATCHING PATTERN "*.cmake"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_urdf2inventor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
