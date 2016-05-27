cmake_minimum_required(VERSION 2.8.3)
project(urdf_traverser)

set (CATKIN_PKGS
  roscpp
  urdf
  eigen_conversions
  baselib_binding
  roslint
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

###################################
## catkin specific configuration ##
###################################

set (CATKIN_DEPEND_PKGS roscpp urdf eigen_conversions baselib_binding)
#set (CATKIN_DEPENDS_LIBS <system libraries>)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES urdf_traverser
  CATKIN_DEPENDS ${CATKIN_DEPEND_PKGS} 
  #DEPENDS ${CATKIN_DEPENDS_LIBS}
  CFG_EXTRAS export_definitions.cmake
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(urdf_traverser
  src/UrdfTraverser.cpp
  src/Helpers.cpp
  src/PrintModel.cpp
  src/JointNames.cpp
  src/ActiveJoints.cpp
  src/DependencyOrderedJoints.cpp
  src/Functions.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(urdf_traverser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_definitions(${baselib_binding_DEFINITIONS})

## Declare a C++ executable
add_executable(print_model test/print_model_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(print_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set (DEPEND_LIBRARIES
   ${catkin_LIBRARIES}
)

add_definitions(${baselib_binding_DEFINITIONS})

## Specify libraries to link a library or executable target against
target_link_libraries(urdf_traverser ${DEPEND_LIBRARIES})
target_link_libraries(print_model urdf_traverser ${DEPEND_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS urdf_traverser
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_urdf_traverser.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
