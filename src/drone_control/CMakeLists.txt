cmake_minimum_required(VERSION 3.8)
project(drone_control)

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};~/Qt/6.8.0/gcc_64")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_FIND_DEBUG_MODE TRUE)
# set(QT_DEBUG_FIND_PACKAGE ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

find_package(Qt6 COMPONENTS
  Core
  Gui
  Widgets
  WebChannel
  WebEngineCore
  WebEngineWidgets
  REQUIRED)

set(SOURCES
  "src/drone_controller.cpp"
  "src/drone_controller_node.cpp"
  "src/main_window.cpp"
)

set(HEADERS
  "include/drone_controller.hpp"
  "include/main_window.hpp"
)

add_executable(drone_controller
  ${SOURCES}
  ${HEADERS}
)

target_include_directories(drone_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  # ${CMAKE_CURRENT_BINARY_DIR}  # For MOC-generated files
)

target_link_libraries(drone_controller
  Qt6::Core
  Qt6::Gui
  Qt6::Widgets
  Qt6::WebChannel
  Qt6::WebEngineCore
  Qt6::WebEngineWidgets
)

target_include_directories(drone_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(drone_controller PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
  drone_controller
  "rclcpp"
  "px4_msgs"
)

install(TARGETS drone_controller
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
