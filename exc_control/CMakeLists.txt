cmake_minimum_required(VERSION 2.8.3)
project(exc_control)
#set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS} /usr/include /usr/include/SFML)
link_directories(/usr/lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu)

add_executable(driver_station src/DriverStation.cpp)
target_link_libraries(driver_station ${catkin_LIBRARIES} sfml-system sfml-window sfml-graphics)
#add_dependencies(driver_station exc_control_generate_messages_cpp)
