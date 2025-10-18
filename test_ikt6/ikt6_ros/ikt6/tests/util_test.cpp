#include "util.h"
#include "gtest/gtest.h"

TEST(UtilTest, pmp) {
  EXPECT_EQ(pmp(M_PI * 2), 0);
  EXPECT_EQ(pmp(-M_PI * 2), 0);
}

TEST(UtilTest, mtx_translate) {
  Vector3d t;
  t << 1, 2, 3;
  Isometry3d T_test = mtx_translate(t);
  Matrix4d T_orig_m;
  T_orig_m << 1, 0, 0, t(0),
              0, 1, 0, t(1),
              0, 0, 1, t(2),
              0, 0, 0, 1;

  std::cerr << "[          ] T_orig_m \n" << T_orig_m << std::endl;
  std::cerr << "[          ] T_test_m \n" << T_test.matrix() << std::endl;

  EXPECT_LE((T_test.matrix() - Isometry3d(T_orig_m).matrix()).norm(), eps);

}

