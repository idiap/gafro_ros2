target_sources(gafro_ros2
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/WrenchStamped.cpp
)

install(
    FILES
        WrenchStamped.hpp
    DESTINATION
        include/gafro_ros2/subscriber
)