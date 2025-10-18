
add_executable(test_kuka_iiwa7_r800 src/test_kuka_iiwa7_r800.cpp)
target_link_libraries(test_kuka_iiwa7_r800 ikt6)

install(TARGETS test_kuka_iiwa7_r800
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
