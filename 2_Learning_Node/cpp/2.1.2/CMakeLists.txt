cmake_minimum_required(VERSION 3.8)
project(first_node)

add_executable(first_node first_node.cpp)

find_package(rclcpp REQUIRED)                                         # 直接查找到对应的头文件和库文件
message(STATUS ${rclcpp_INCLUDE_DIRS})                                # 头文件及rclcpp依赖的头文件的路径
message(STATUS ${rclcpp_LIBRARIES})                                   # 库文件及rclcpp依赖的库文件的路径    

target_include_directories(first_node PUBLIC ${rclcpp_INCLUDE_DIRS})  # 添加头文件路径
target_link_libraries(first_node ${rclcpp_LIBRARIES})                 # 添加库文件
