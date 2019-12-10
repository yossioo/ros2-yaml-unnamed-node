cmake_minimum_required(VERSION 3.5)
project(ros2-yaml-unnamed-node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(main src/main.cpp)
ament_target_dependencies(main rclcpp)


# ============================== #
#           Installs             #
# ============================== #

# It is possible to put all the targets (executables) here
install(TARGETS
        main
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY launch param
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS
        main
        DESTINATION lib/${PROJECT_NAME})
ament_package()
