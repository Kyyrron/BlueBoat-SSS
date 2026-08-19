cmake_minimum_required(VERSION 3.5)
project(blueboat_sss)

find_package(ament_cmake REQUIRED)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS src/_custom_libraries/sss_helper.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/_custom_libraries/svlog_helper.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/_custom_libraries/math_helper.py
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS src/processed_sss_listener.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/sss_node.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/sss_processor_node.py
        DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_package()
