find_package(gafro REQUIRED)
find_package(gafro_robot_descriptions REQUIRED)
find_package(gafro_ros2 REQUIRED)
find_package(sackmesser_ros2 REQUIRED)

add_executable(visualize_franka_robot visualize_franka_robot.cpp)

target_compile_options(visualize_franka_robot PUBLIC -Wall -Wextra -Wpedantic -Werror)

ament_target_dependencies(visualize_franka_robot
  PUBLIC
    sackmesser_ros2
    gafro_ros2
)

ament_export_dependencies(
  sackmesser_ros2
  gafro_ros2
)

target_link_libraries(visualize_franka_robot
  PUBLIC
    gafro::gafro
    gafro_robot_descriptions::gafro_robot_descriptions
)

install(
  TARGETS 
    visualize_franka_robot
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  FILES
    visualize_franka_robot_launch.py
    visualize_franka_robot_config.yaml
    visualize_franka_robot.rviz
  DESTINATION 
    share/${PROJECT_NAME}/visualize_franka_robot/
)