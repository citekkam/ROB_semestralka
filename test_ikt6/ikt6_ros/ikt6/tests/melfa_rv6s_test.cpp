#include "ikt6.h"
#include <gtest/gtest.h>


class MelfaRV6sTest : public ::testing::Test {
 protected:
  void SetUp() override {
    Vector6d lengths, offsets, limits_max, directions, limits_min;

    lengths <<     350,      85,     280,  100,   315,     85;     // Lenghts
    offsets <<       0,       0,       0,    0,     0,      0;     // Joint offsets in Rad
    directions <<    1,       1,       1,    1,     1,      1;     // Joint directions
    limits_max <<  170,     135,     166,  160,   120,    360;     // Joint limits max in Rad
    limits_min << -170,     -92,    -107, -160,  -120,   -360;     // Joint limits min in Rad
    limits_max = limits_max * M_PI/180;
    limits_min = limits_min * M_PI/180;
    Isometry3d base = Isometry3d::Identity();
    Isometry3d tool = Isometry3d::Identity();

    robot = ikt6_robot_init( "Mitsubishi Melfa RV6S", lengths, offsets, directions, limits_max, limits_min, base, tool);
    pos_zero << lengths[1] - lengths[3], 0, lengths[0] + lengths[2] + lengths[4] + lengths[5], 0, 0, 0;
    pos_l_shape << lengths[1] + lengths[4] + lengths[5], 0, lengths[0] + lengths[2] + lengths[3], 0,  M_PI/2, 0;
  }

  Robot robot;
  Vector6d pos_zero;
  Vector6d pos_l_shape;
};


TEST_F(MelfaRV6sTest, dkt_zero) {
  Vector6d j;
  j << 0, 0, 0, 0, 0, 0;
  Vector6d pos_test = ikt6_dkt(&robot, j);
  std::cerr << "[          ] pos_test " << pos_test.transpose() << std::endl;
  std::cerr << "[          ] pos_zero " << pos_zero.transpose() << std::endl;
  ASSERT_LE(pos_test(0) - pos_zero(0), eps);
  ASSERT_LE(pos_test(1) - pos_zero(1), eps);
  ASSERT_LE(pos_test(2) - pos_zero(2), eps);
  ASSERT_LE(pos_test(3) - pos_zero(3), eps);
  ASSERT_LE(pos_test(4) - pos_zero(4), eps);
  ASSERT_LE(pos_test(5) - pos_zero(5), eps);
}

TEST_F(MelfaRV6sTest, dkt_l_shape) {
  Vector6d j;
  j << 0, 0, M_PI/2, 0, 0, 0;
  Vector6d pos_test = ikt6_dkt(&robot, j);
  std::cerr << "[          ] pos_test " << pos_test.transpose() << std::endl;
  std::cerr << "[          ] pos_l_shape " << pos_l_shape.transpose() << std::endl;
  ASSERT_LE((pos_test - pos_l_shape).norm(), eps);
}

TEST_F(MelfaRV6sTest, ikt_zero) {
  Vector6d j;
  j << 0, 0, 0, 0, 0, 0;
  Matrix<double, 6, Dynamic> x = ikt6_ikt(&robot, pos_l_shape);
  std::cerr << "[          ] test   " << x << std::endl;
  std::cerr << "[          ] target " << j.transpose() << std::endl;
}
