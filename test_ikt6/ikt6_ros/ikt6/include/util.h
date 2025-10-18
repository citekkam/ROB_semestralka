#ifndef UTIL_H_
#define UTIL_H_

#include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;

/** Largest value that is assumed zero.
  */
const double eps = numeric_limits<double>::epsilon() * 10000;

/** Column vector with 6 elements for joints and cartesian coordinates;
  */
typedef Matrix<double, 6, 1> Vector6d;

/** Sign
 */
template <typename T> int sgn(T val);

/** Plus minus PI */
double pmp(double angle);

/** 4x4 transformation matrix representing translation given as 3x1 vector */
Isometry3d mtx_translate(Vector3d t);

/** 4x4 transformation matrix representing rotation around X axis by given angle */
Isometry3d mtx_rotate_x(double angle);

/** 4x4 transformation matrix representing rotation around Y axis by given angle */
Isometry3d mtx_rotate_y(double angle);

/** 4x4 transformation matrix representing rotation around Z axis by given angle */
Isometry3d mtx_rotate_z(double angle);

/** 4x4 transormation matrix representing DH parameters given as 4x1 vector, [alfa, a, theta d] */
Isometry3d mtx_dh(Vector4d dhcol);

/** Rotation matrix to Euler angles */
Vector3d mtx_to_angles(Isometry3d T);



#endif /* ifndef UTIL_H_ */
