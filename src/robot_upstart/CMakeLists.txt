cmake_minimum_required(VERSION 2.8.3)
project(robot_upstart)

find_package(catkin REQUIRED roscpp)

catkin_package()
catkin_python_setup()

file(GLOB SCRIPTS scripts/*)
install(PROGRAMS ${SCRIPTS} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY templates DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


 add_executable(start_node  src/myrobot.cpp)
 target_link_libraries(start_node
    ${catkin_LIBRARIES}
 )


if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_python()
  roslint_add_test()

  catkin_add_nosetests(test)
endif()
