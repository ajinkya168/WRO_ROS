cmake_minimum_required(VERSION 3.8)
project(urdf_lidar_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}/
)


install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/worlds
)
install(PROGRAMS
  src/fake_odometry.py
  src/fake_lidar.py
  src/odom_to_base_link_tf.py
  src/joystick_teleop.py
  src/slam_mode_switcher.py
  src/get_pose_from_tf.py
  src/send_multiple_goals.py
  src/image_processor.py
  src/get_block.py
  src/obstacle_challenge.py
  src/spawn_dummy_obstacle.py
  src/spawn_dummy_obstacle2.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
