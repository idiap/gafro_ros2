target_sources(gafro_ros2
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/Motor.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Point.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Sphere.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Circle.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Plane.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Line.cpp

        ${CMAKE_CURRENT_LIST_DIR}/Wrench.cpp
)

install(
    FILES
        Motor.hpp
        Point.hpp
        Sphere.hpp
        Circle.hpp
        Plane.hpp
        Line.hpp
        Wrench.hpp
    DESTINATION
        include/gafro_ros2/conversion
)