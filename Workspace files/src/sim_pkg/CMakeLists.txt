cmake_minimum_required(VERSION 3.0.2)
project(sim_pkg)


find_package(catkin REQUIRED COMPONENTS
  dynamixel_workbench_controllers
  dynamixel_workbench_msgs
  roscpp
  std_msgs
  cmake_modules
     message_generation
    dynamixel_sdk
)
 find_package(Eigen REQUIRED)
 find_package(rosbag REQUIRED)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
    SetPosition.msg
    SyncSetPosition.msg
    BulkSetItem.msg
    Position.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    GetPosition.srv
    SyncGetPosition.srv
    BulkGetItem.srv
    ResetMotors.srv
)
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    dynamixel_workbench_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sim_pkg
 CATKIN_DEPENDS dynamixel_workbench_controllers dynamixel_workbench_msgs roscpp std_msgs message_runtime
  DEPENDS system_lib Eigen
      roscpp
    std_msgs
    dynamixel_sdk
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${rosbag_INCLUDE_DIRS}
  
)
add_executable(sim_v1 src/sim_v1.cpp) 
target_link_libraries(sim_v1 ${rosbag_LIBRARIES})
target_link_libraries(sim_v1 ${catkin_LIBRARIES})
add_dependencies(sim_v1 ${catkin_EXPORTED_TARGETS})

add_executable(rec src/rec.cpp) 
target_link_libraries(rec ${rosbag_LIBRARIES})
target_link_libraries(rec ${catkin_LIBRARIES})
add_dependencies(rec ${catkin_EXPORTED_TARGETS})

add_executable(rec_v2 src/rec_v2.cpp) 
target_link_libraries(rec_v2 ${rosbag_LIBRARIES})
target_link_libraries(rec_v2 ${catkin_LIBRARIES})
add_dependencies(rec_v2 ${catkin_EXPORTED_TARGETS})

add_executable(move_to_config src/move_to_config.cpp) 
target_link_libraries(move_to_config ${catkin_LIBRARIES})
add_dependencies(move_to_config ${catkin_EXPORTED_TARGETS})


add_executable(cvx src/cvx.cpp) 
add_library(solver src/solver.cpp)
add_library(matrix_support src/matrix_support.cpp)
add_library(ldl src/ldl.cpp)
add_library(util src/util.cpp)
target_link_libraries(cvx ${catkin_LIBRARIES} solver matrix_support ldl util)
add_dependencies(cvx ${catkin_EXPORTED_TARGETS})

add_executable(testsolver src/testsolver.cpp) 
target_link_libraries(testsolver ${catkin_LIBRARIES} solver matrix_support ldl util)
target_link_libraries(testsolver ${rosbag_LIBRARIES})
add_dependencies(testsolver ${catkin_EXPORTED_TARGETS})

add_executable(joy_v1 src/joy_v1.cpp) 
target_link_libraries(joy_v1 ${catkin_LIBRARIES})
target_link_libraries(joy_v1 ${rosbag_LIBRARIES})
add_dependencies(joy_v1 ${catkin_EXPORTED_TARGETS})

add_executable(joy_v2 src/joy_v2.cpp) 
target_link_libraries(joy_v2 ${catkin_LIBRARIES})
target_link_libraries(joy_v2 ${rosbag_LIBRARIES})
add_dependencies(joy_v2 ${catkin_EXPORTED_TARGETS})

add_executable(main src/main.cpp) 
target_link_libraries(main ${catkin_LIBRARIES})
target_link_libraries(main ${rosbag_LIBRARIES})
add_dependencies(main ${catkin_EXPORTED_TARGETS})

add_executable(spm_test1 src/spm_test1.cpp)
add_dependencies(spm_test1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(spm_test1 ${catkin_LIBRARIES})

add_executable(phys_model src/phys_model.cpp)
add_dependencies(phys_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(phys_model ${catkin_LIBRARIES})

