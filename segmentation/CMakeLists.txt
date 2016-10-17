cmake_minimum_required(VERSION 2.8.3)
project(segmentation)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(OpenCV 2.4.8 REQUIRED)

add_message_files(
   FILES
   box_data.msg
   processed_box_data.msg
   box_3D_data.msg
   processed_box_3D_data.msg
   cylinder_data.msg
   processed_cylinder_data.msg
   sphere_data.msg
   processed_sphere_data.msg
)


generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
   CATKIN_DEPENDS cv_bridge image_transport message_generation pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cylinder_extraction_cv_pcl_node src/cylinder_extraction_cv_pcl.cpp)
target_link_libraries(cylinder_extraction_cv_pcl_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable(cylinder_segmentation_node src/cylinder_segmentation.cpp)
target_link_libraries(cylinder_segmentation_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable(multi_cylinder_segmentation_node src/multi_cylinder_segmentation.cpp)
target_link_libraries(multi_cylinder_segmentation_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable(box_segmentation_node src/box_segmentation.cpp)
target_link_libraries(box_segmentation_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable(box_extraction_cv_pcl_node src/box_extraction_cv_pcl.cpp)
target_link_libraries(box_extraction_cv_pcl_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable(box_extraction_cv_pcl_3D_node src/box_extraction_cv_pcl_3D.cpp)
target_link_libraries(box_extraction_cv_pcl_3D_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable(sphere_segmentation_node src/sphere_segmentation.cpp)
target_link_libraries(sphere_segmentation_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

