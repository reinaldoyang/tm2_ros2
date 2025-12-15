cmake_minimum_required(VERSION 3.5)
project(robotiq_description)

# 使用 ament_cmake
find_package(ament_cmake REQUIRED)

# 把描述檔、mesh 等安裝到 share/${PROJECT_NAME}
install(
  DIRECTORY
    config
    launch
    meshes
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
