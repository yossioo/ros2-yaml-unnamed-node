cmake_minimum_required(VERSION 3.5)
project(ros2-yaml-unnamed-node)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

include(FindPkgConfig)
if (!${PKG_CONFIG_FOUND})
    MESSAGE(FATAL_ERROR "pkg-config required. Please install it")
    return("Error - pkg-config not found")
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

#add_library(SimpleNode include/simple_node.cpp)
#ament_target_dependencies(SimpleNode rclcpp)
add_executable(main src/main.cpp)
#target_link_libraries(main SimpleNode)
ament_target_dependencies(main
        rclcpp
        )


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
message(STATUS "Installing files to: lib/${PROJECT_NAME})")
ament_package()
