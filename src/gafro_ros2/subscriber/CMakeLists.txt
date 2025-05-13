target_sources(gafro_ros2
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/WrenchStamped.cpp
        ${CMAKE_CURRENT_LIST_DIR}/TwistStamped.cpp
)

install(
    FILES
        WrenchStamped.hpp
        TwistStamped.hpp
    DESTINATION
        include/gafro_ros2/subscriber
)