cmake_minimum_required(VERSION 2.8.3)
project(numato_relay_interface)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/numato_relay_interface
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslint)
  roslint_python(scripts/numato_relay_interface)
  roslint_add_test()
endif()
