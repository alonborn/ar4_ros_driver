cmake_minimum_required(VERSION 3.5)
project(annin_ar4_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  std_srvs  # Add this line
  std_msgs
)


# Important: Generate code for typed support targets
# rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Create the shared library with renamed target
add_library(${PROJECT_NAME} SHARED 
  src/ar_hardware_interface.cpp
  src/ar_servo_gripper_hw_interface.cpp
  src/teensy_driver.cpp
  src/arduino_nano_driver.cpp
)

# Set the C++ standard and include directories
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies for the shared library
ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  hardware_interface
  std_srvs  # Add this line to link the std_srvs package
  std_msgs
  Boost
)

# Link against the generated interfaces
target_link_libraries(${PROJECT_NAME}
  "${cpp_typesupport_target}"
)

# Create an executable for running Arduino Nano
add_executable(run_arduino_nano src/run_arduino_nano.cpp)

# Link the executable to the library
target_link_libraries(
  run_arduino_nano
  ${PROJECT_NAME}
)

# Install the executable
install(TARGETS
  run_arduino_nano
  DESTINATION lib/${PROJECT_NAME}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# Install the shared library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers and other resources
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/reset_estop.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# Finalize the package
ament_package()