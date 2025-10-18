cmake_minimum_required(VERSION 3.14)
project(ikt6)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(GTest REQUIRED NO_MODULE)

include_directories(${EIGEN3_INCLUDE_DIR} ${GTest_INCLUDE_DIR} include)

add_library(ikt6 src/util.cpp src/ikt6.cpp)

# Tests {{{

enable_testing()

add_executable(hello_test tests/hello_test.cpp)
target_link_libraries(hello_test GTest::gtest_main)

add_executable(ikt6_test tests/util_test.cpp)
target_link_libraries(ikt6_test ikt6 GTest::gtest_main )

add_executable(melfa_rv6s_test tests/melfa_rv6s_test.cpp)
target_link_libraries(melfa_rv6s_test ikt6 GTest::gtest_main )

add_executable(test_melfa tests/test_melfa_rv6s.cpp)
target_link_libraries(test_melfa ikt6)

add_executable(motoman_qp180_test tests/motoman_qp180_test.cpp)
target_link_libraries(motoman_qp180_test ikt6)

add_executable(motoman_gp88_test tests/motoman_gp88_test.cpp)
target_link_libraries(motoman_gp88_test ikt6)

add_executable(my_custom_test tests/my_custom_test.cpp)
target_link_libraries(my_custom_test ikt6)

include(GoogleTest)
gtest_discover_tests(hello_test)
gtest_discover_tests(ikt6_test)
gtest_discover_tests(melfa_rv6s_test)

# }}}
