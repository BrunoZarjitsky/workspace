add_library(model_push SHARED model_push.cpp model_push.cpp)
add_dependencies(model_push ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(model_push ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(RayPlugin SHARED publisher.cpp)
add_dependencies(RayPlugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(RayPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# Build our plugin
add_library(velodyne_plugin SHARED velodyne_plugin.cpp)
target_link_libraries(velodyne_plugin ${GAZEBO_LIBRARIES})


# Build our plugin
add_library(skate_plugin SHARED skate_plugin.cpp)
target_link_libraries(skate_plugin ${GAZEBO_LIBRARIES})

# Build the stand-alone test program
add_executable(vel vel.cpp)

target_link_libraries(vel ${GAZEBO_LIBRARIES})