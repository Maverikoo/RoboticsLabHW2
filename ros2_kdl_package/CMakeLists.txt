cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

# Warning level
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Per Action
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- Interfacce (Action) ----
# Genera il codice C++ dal file .action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTrajectory.action"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

# ---- Executables esistenti (1a, 1b, example) ----

add_executable(ros2_kdl_node_1a
  src/ros2_kdl_node_1a.cpp
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp
)
target_include_directories(ros2_kdl_node_1a PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(ros2_kdl_node_1a PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ros2_kdl_node_1a
  orocos_kdl rclcpp kdl_parser sensor_msgs std_msgs geometry_msgs
)

add_executable(ros2_kdl_node_1b
  src/ros2_kdl_node_1b.cpp
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp
)
target_include_directories(ros2_kdl_node_1b PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(ros2_kdl_node_1b PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ros2_kdl_node_1b
  orocos_kdl rclcpp kdl_parser sensor_msgs std_msgs geometry_msgs
)

add_executable(ros2_kdl_node_example src/ros2_kdl_node_example.cpp)
target_include_directories(ros2_kdl_node_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(ros2_kdl_node_example PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ros2_kdl_node_example orocos_kdl rclcpp)


# ---- Nuovi eseguibili per 1.c (Action Server + Client) ----

# 1. ACTION SERVER (Componente kdl_node_1c.cpp)
# Costruiamo la libreria condivisa (.so)
add_library(action_server SHARED
  src/ros2_kdl_node_1c.cpp
  # Aggiungiamo i file sorgente condivisi manualmente
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp
)

target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(action_server PUBLIC c_std_99 cxx_std_17)

# Dipendenze per action_server (deve includere tutto)
ament_target_dependencies(action_server
  # Dipendenze KDL
  orocos_kdl
  kdl_parser
  sensor_msgs
  std_msgs
  # Dipendenze Action
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
)

# === SOLUZIONE (dal Warning) ===
# Collega l'azione generata al target 'action_server'
rosidl_get_typesupport_target(action_server_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(action_server ${action_server_typesupport})
# ===============================

# Registra la libreria come Componente
rclcpp_components_register_node(action_server 
  PLUGIN "Iiwa_pub_sub"             # Corrisponde a RCLCPP_COMPONENTS_REGISTER_NODE(Iiwa_pub_sub)
  EXECUTABLE ros2_kdl_node_1c            # Nome dell'eseguibile "wrapper" standalone
) 


# 2. ACTION CLIENT
add_executable(execute_trajectory_client 
  src/execute_trajectory_client.cpp
)

target_include_directories(execute_trajectory_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(execute_trajectory_client PUBLIC c_std_99 cxx_std_17)

# Dipendenze per execute_trajectory_client
ament_target_dependencies(execute_trajectory_client 
  rclcpp 
  rclcpp_action 
  geometry_msgs
)

# === SOLUZIONE (dal Warning) ===
# Collega l'azione generata al target 'execute_trajectory_client'
rosidl_get_typesupport_target(client_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(execute_trajectory_client ${client_typesupport})
# ===============================


# ---- Install ----

# Installa la LIBRERIA del componente (per il loader)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Installa TUTTI gli ESEGUIBILI
install(TARGETS
  # Vecchi
  ros2_kdl_node_1a
  ros2_kdl_node_1b
  ros2_kdl_node_example
  # Nuovi
  ros2_kdl_node_1c                
  execute_trajectory_client
  DESTINATION lib/${PROJECT_NAME} 
)

# Installa i file di launch e config
install(
  DIRECTORY config launch action
  DESTINATION share/${PROJECT_NAME}
)

# Installa gli header (buona pratica)
install(
  DIRECTORY include/
  DESTINATION include
)

# ---- Testing ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Necessario per i tipi generati da rosidl
ament_export_dependencies(rosidl_default_runtime)
ament_package()