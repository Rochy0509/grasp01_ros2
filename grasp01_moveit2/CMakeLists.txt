cmake_minimum_required(VERSION 3.22)
project(grasp01_moveit2)

find_package(ament_cmake REQUIRED)
find_package(grasp01_hardware REQUIRED)
find_package(grasp01_description REQUIRED)
find_package(moveit_py REQUIRED)



ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
