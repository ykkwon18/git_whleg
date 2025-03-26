cmake_minimum_required(VERSION 3.8)
project(urdf_test)

find_package(ament_cmake REQUIRED)

install(
	DIRECTORY launch urdf meshes config
	DESTINATION share/${PROJECT_NAME}
)

ament_package()

