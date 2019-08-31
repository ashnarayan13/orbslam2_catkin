cmake_minimum_required(VERSION 2.8.3)
project(orb_slam)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  image_transport
  image_geometry
  cv_bridge
  pcl_ros
  nav_msgs
)

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
#find_package(Cholmod REQUIRED)
#set(G2O_DIR "/home/ashwath/g2o/cmake_modules")
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Pangolin REQUIRED)


catkin_package()

message(*************************)
message(${catkin_INCLUDE_DIRS})
message(*************************)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${G2O_INCLUDE_DIR}
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

add_subdirectory(Thirdparty/DBoW2)

add_executable(${PROJECT_NAME}_stereo
  src/ros_stereo_test.cc
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
  src/Viewer.cc
  src/RosPublisher.cc
)
target_link_libraries(${PROJECT_NAME}_stereo
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${Pangolin_LIBRARIES}
  DBoW2
)

add_executable(${PROJECT_NAME}_mono
  src/ros_mono.cc
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
  src/Viewer.cc
  src/RosPublisher.cc
)
target_link_libraries(${PROJECT_NAME}_mono
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${Pangolin_LIBRARIES}
  DBoW2
)

add_executable(${PROJECT_NAME}_rgbd
  src/ros_rgbd.cc
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
  src/Viewer.cc
  src/RosPublisher.cc
)
target_link_libraries(${PROJECT_NAME}_rgbd
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${Pangolin_LIBRARIES}
  DBoW2
)