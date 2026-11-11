cmake_minimum_required(VERSION 3.16)
project(experiment9_fusion)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Pure Python package: no compiled node. The Gazebo plugins live in
# experiment7_lidar and are reused as-is; nothing here touches the simulator
# except through the ROS graph.
ament_python_install_package(experiment9_core)
ament_python_install_package(experiment9_student)

install(PROGRAMS
  scripts/fusion_node.py
  scripts/odom_tf.py
  scripts/check_ready.py
  scripts/costmap_dump.py
  scripts/cloud_check.py
  scripts/record_run.py
  scripts/send_goal.sh
  DESTINATION lib/${PROJECT_NAME})

# docs/ lives at the repository root, not inside the package: only what the
# launch files and nodes actually read at runtime is installed here.
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_algorithms tests/test_algorithms.py)
  endif()
endif()

ament_package()
