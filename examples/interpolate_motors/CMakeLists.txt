find_package(gafro REQUIRED)

add_executable(interpolate_motors interpolate_motors.cpp)

target_compile_options(interpolate_motors PUBLIC -Wall -Wextra -Wpedantic -Werror)

ament_target_dependencies(interpolate_motors
  PUBLIC
    sackmesser_ros2
)

ament_export_dependencies(
  sackmesser_ros2
)

target_link_libraries(interpolate_motors
  PUBLIC
    gafro::gafro
    gafro_ros2
)

install(
  TARGETS 
    interpolate_motors
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  FILES
    interpolate_motors_launch.py
    interpolate_motors_config.yaml
    interpolate_motors.rviz
  DESTINATION 
    share/${PROJECT_NAME}/interpolate_motors/
)