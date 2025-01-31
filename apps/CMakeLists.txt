add_executable(convert_urdf convert_urdf.cpp)

target_compile_options(convert_urdf PUBLIC -Wall -Wextra -Wpedantic -Werror)

target_link_libraries(convert_urdf PUBLIC gafro_ros2)

install(
  TARGETS 
    convert_urdf
  DESTINATION
    lib/gafro_ros2
)