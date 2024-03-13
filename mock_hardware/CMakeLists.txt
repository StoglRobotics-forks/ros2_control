cmake_minimum_required(VERSION 3.16)
project(mock_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcpputils
  rcutils
  transmission_interface
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(mock_components SHARED
  src/generic_system.cpp
)
target_compile_features(mock_components PUBLIC cxx_std_17)
target_include_directories(mock_components PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mock_components>
)
ament_target_dependencies(mock_components PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mock_components PRIVATE "MOCK_COMPONENTS_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  hardware_interface mock_components_plugin_description.xml)

if(BUILD_TESTING)

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_generic_system test/test_generic_system.cpp)
  target_include_directories(test_generic_system PRIVATE include)
  target_link_libraries(test_generic_system mock_components)
  ament_target_dependencies(test_generic_system
    pluginlib
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/mock_components
)
install(
  TARGETS
    mock_components
  EXPORT export_mock_components
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_mock_components HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
