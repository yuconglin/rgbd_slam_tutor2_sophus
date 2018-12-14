cmake_minimum_required( VERSION 2.8 ) 
project( rgbd_slam_tutor2_sophus )

set( CMAKE_BUILD_TYPE Release )

#set( CMAKE_CXX_COMPILER "clang++" )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -ggdb -std=c++11 -march=native -O3 -pthread" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( PCL 1.7 REQUIRED )
find_package( G2O )
find_package(VTK REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( Cholmod )
include_directories( ${CHOLMOD_INCLUDE_DIR} )

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs cmake_modules message_generation)

add_service_files(
  FILES
  rgbd_receive.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib )

set(g2o_libs
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_modified/lib/libg2o_core.so 
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_modified/lib/libg2o_types_slam3d.so 
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_modified/lib/libg2o_solver_csparse.so 
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_modified/lib/libg2o_stuff.so 
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_modified/lib/libg2o_csparse_extension.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_modified/lib/libg2o_types_sim3.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_modified/lib/libg2o_types_sba.so
)

include_directories(
    ${PROJECT_SOURCE_DIR}/
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/Thirdparty/
    ${PROJECT_SOURCE_DIR}/Thirdparty/sophus/
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_modified/
    ${EIGEN3_INCLUDE_DIR}
    )

add_subdirectory( ${PROJECT_SOURCE_DIR}/src/ )
add_subdirectory( ${PROJECT_SOURCE_DIR}/experiment/ )
