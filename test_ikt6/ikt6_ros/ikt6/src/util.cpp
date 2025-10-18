
#include "util.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double pmp(double angle) {

    if (angle <= -M_PI) {
        angle = angle + 2*M_PI;
    }

    if (angle > M_PI) {
        angle = angle - 2*M_PI;
    }

    return angle;
}

Isometry3d mtx_translate(Vector3d t) {
  Isometry3d T = Isometry3d::Identity();
  T.translate(t);
  return T;
}

Isometry3d mtx_rotate_x(double angle) {
  Isometry3d R = Isometry3d::Identity();
  R.rotate(AngleAxisd(angle, Vector3d::UnitX()));
  return R;
}

Isometry3d mtx_rotate_y(double angle) {
  Isometry3d R = Isometry3d::Identity();
  R.rotate(AngleAxisd(angle, Vector3d::UnitY()));
  return R;
}

Isometry3d mtx_rotate_z(double angle) {
  Isometry3d R = Isometry3d::Identity();
  R.rotate(AngleAxisd(angle, Vector3d::UnitZ()));
  return R;
}

Isometry3d mtx_dh(Vector4d dhcol) {

    double alpha = dhcol(0);
    double a     = dhcol(1);
    double theta = dhcol(2);
    double d     = dhcol(3);

    Matrix4d T;

    T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
                  0,             sin(alpha),             cos(alpha),            d,
                  0,                      0,                      0,            1;

    return Isometry3d(T);
}

/** Rotation matrix to Euler angles */
Vector3d mtx_to_angles(Isometry3d T) {


    double s2 = -T(2, 0);
    double A,B,C,c2;

    if (fabs(s2 - 1) < eps) {
        A = 0;
        B = M_PI/2;
        C = atan2(T(1, 2), T(0, 2));
    } else if (fabs(s2 + 1) < eps) {
        A = 0;
        B = -M_PI/2;
        C = atan2(-T(1, 2), T(0, 2));
    } else {
        s2 = min(max(s2, -1.0), 1.0);
        B = asin(s2);
        c2 = sgn(cos(B));
        C = atan2(T(1, 0) * c2, T(0, 0) * c2);
        A = atan2(T(2, 1) * c2, T(2, 2) * c2);
    }

    Vector3d angles(A, B, C);

    return angles;
}
