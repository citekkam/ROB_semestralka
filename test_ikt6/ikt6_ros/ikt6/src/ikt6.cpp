#include "ikt6.h"


Robot ikt6_robot_init(
        string name,
        Vector6d lengths,
        Vector6d offsets,
        Vector6d directions,
        Vector6d limits_max,
        Vector6d limits_min,
        Isometry3d base,
        Isometry3d tool)
{
    Robot robot;
    robot.name = name;
    robot.lengths = lengths;
    robot.offsets = offsets;
    robot.directions = directions;
    robot.limits_max = limits_max;
    robot.limits_min = limits_min;
    robot.base = base;
    robot.tool = tool;

    // robot.DH <<    -M_PI/2,          0,    -M_PI/2,     M_PI/2,    -M_PI/2,          0, // alfa
    //             lengths(1), lengths(2), lengths(3),          0,          0,          0, // a
    //             offsets(0), offsets(1), offsets(2), offsets(3), offsets(4), offsets(5), // theta
    //             lengths(0),          0,          0, lengths(4),          0, lengths(5); // d

    robot.DH <<    -M_PI/2,          0,    -M_PI/2,     M_PI/2,    -M_PI/2,          0, // alfa
                lengths(1), lengths(2), lengths(3),          0,          0,          0, // a
                        0,     -M_PI/2,    -M_PI/2,          0,          0,       M_PI, // theta
                lengths(0),          0,          0, lengths(4),          0, lengths(5); // d

    robot.DH_Param << 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0,
                      1, 1, 1, 1, 1, 1,
                      0, 0, 0, 0, 0, 0;

    robot.A76 = mtx_translate(Vector3d(0, 0, lengths(5)));

    return robot;
}

double apply_offset_and_direction(const Robot *robot, double j, size_t i) {
    return (j - robot->offsets(i)) * robot->directions(i);
}

Vector6d apply_offset_and_direction(const Robot *robot, Vector6d J) {
    Vector6d J_new;
    for (size_t i = 0; i < 6; i++) {
        J_new(i) = apply_offset_and_direction(robot, J(i), i);
    }

    return J_new;
}

double remove_offset_and_direction(const Robot *robot, double j, size_t i) {
    return j*robot->directions(i) + robot->offsets(i);
}

Vector6d remove_offset_and_direction(const Robot *robot, Vector6d J) {
    Vector6d J_new;
    for (size_t i = 0; i < 6; i++) {
        J_new(i) = apply_offset_and_direction(robot, J(i), i);
    }
    return J_new;
}

/**
  * Test limits of one joint, the input j is assumed to include offset and direction change applied, i is the joint number (staring from 0).
  */
bool test_limits(const Robot *robot, double j, size_t i) {
    j = remove_offset_and_direction(robot, j, i);
    return j > robot->limits_min(i) && j < robot->limits_max(i);
}

Isometry3d ikt6_dkt_T(const Robot * robot, Vector6d J) {

    Isometry3d T = robot->base;
    Vector6d P;

    // Apply offsets and direction changes
    J = apply_offset_and_direction(robot, J);

    size_t n = min(J.size(), robot->DH.cols());
    for (size_t i = 0; i < n; i++) {
        T = T * mtx_dh(robot->DH.col(i) + (J(i) * robot->DH_Param.col(i)));
    }

    T = T * robot->tool;

    return T;
}

Vector6d ikt6_dkt(const Robot * robot, Vector6d J) {

    Vector6d P;
    Isometry3d T = ikt6_dkt_T(robot, J);

    Vector4d t = T * Vector4d(0, 0, 0, 1);
    P << t.head(3), mtx_to_angles(T);

    return P;
}

Matrix<double, 5, 2> solve_two_circles(Vector2d c1, double r1, Vector2d c2, double r2, double theta1) {

    Matrix<double, 5, 2> s;
    s.fill(nan(""));

    double a = sqrt(pow(c2.array() - c1.array(), 2).sum()); // a is distance between circle centers (c1, c2)
    double x = (pow(r1, 2) + pow(a,2) - pow(r2, 2)) / (2*a);
    double sqr_y = pow(r1,2) - pow(x,2);
    double phi = nan("");
    double y = nan("");

    if (abs(sqr_y) < eps) {
        phi = atan2(c2(1) - c1(1), c2(0) - c1(0));

        s.col(0) << c1(0) + cos(phi)*x, c1(0) + sin(phi)*x, theta1, c2(0), c2(1);
        s.col(1) << c1(0) + cos(phi)*x, c1(0) + sin(phi)*x, theta1, c2(0), c2(1);
    } else if (sqr_y > 0) {
        y = sqrt(sqr_y);
        phi = atan2(c2(1) - c1(1), c2(0) - c1(0));

        s.col(0) << c1(0) + cos(phi)*x - sin(phi)*y, c1(1) +  sin(phi)*x + cos(phi)*y, theta1, c2(0), c2(1),
        s.col(1) << c1(0) + cos(phi)*x - sin(phi)*(-y), c1(1) +  sin(phi)*x + cos(phi)*(-y), theta1, c2(0), c2(1);
    }
    return s;
}

Matrix<double, 6, Dynamic> ikt6_ikt(const Robot * robot, const Vector6d & P) {
    const Isometry3d T = mtx_translate(P.head(3)) * mtx_rotate_z(P(5)) * mtx_rotate_y(P(4)) * mtx_rotate_x(P(3));
    return ikt6_ikt(robot, T);
}

Matrix<double, 6, Dynamic> ikt6_ikt(const Robot * robot, Isometry3d T) {

    Matrix<double, 6, 8> J;
    J.fill(nan(""));

    // Remove tool from the
    T = T * robot->tool.inverse();
    T = T * robot->A76.inverse();
    T = robot->base.inverse() * T;
    Vector4d W = T * Vector4d(0,0,0,1);


    double theta1a = atan2(W(1), W(0));
    double theta1b = theta1a + M_PI;
    if (theta1b > M_PI) {
        theta1b = theta1b - 2*M_PI;
    }

    // two circles with center c1, c2 and radius r1, r2
    double r1 = robot->lengths(2);
    double r2 = sqrt(pow(robot->lengths(4), 2) + pow(robot->lengths(3), 2));

    Vector2d c1(robot->lengths(1), 0);
    double absW = sqrt(pow(W(0), 2) + pow(W(1), 2));

    Vector2d c2a( absW, W(2) - robot->lengths(0));
    Vector2d c2b(-absW, W(2) - robot->lengths(0));

    Matrix<double, 5, 2> s1 = solve_two_circles(c1, r1, c2a, r2, theta1a);
    Matrix<double, 5, 2> s2 = solve_two_circles(c1, r1, c2b, r2, theta1b);

    Matrix<double, 5, 4> s;
    s << s1, s2;

    // If no solution return empty solution set
    if (s.cols() <= 0) {
        J.resize(NoChange, 0);
        return J;
    }

    // First three joints
    Matrix<double, 3, 4> J3;
    J3.fill(nan(""));

    for (size_t i = 0; i < s.cols(); i++) {

        if (s.col(i).hasNaN()) {
            continue;
        }

        double theta1 = s(2, i);
        double theta2 = atan2(s(0,i) - c1(0), s(1,i) - c1(1));
        double theta3 = atan2(s(3,i) - s(0,i), s(4,i) - s(1,i)) - theta2 + atan2(robot->lengths(3), robot->lengths(4));

        // <+pi, -pi>
        theta3 = pmp(theta3);

        // check solution agains limits
        if (test_limits(robot, theta1, 0) &&
            test_limits(robot, theta2, 1) &&
            test_limits(robot, theta3, 2))
        {
            J3.col(i) << theta1, theta2, theta3;
        }
    }


    size_t n_sol_J = 0;
    for (size_t i = 0; i < J3.cols(); i++) {

        if (J3.col(i).hasNaN()) {
            n_sol_J+=2;
            continue;
        }

        Isometry3d M = robot->base;

        for (size_t j = 0; j < 3; j++) {
            M = M * mtx_dh(robot->DH.col(j) + (J3(j, i) * robot->DH_Param.col(j)));
        }

        Isometry3d mtx = M.inverse() *  T;
        double c5 = mtx(2,2);
        double theta4 = nan("");
        double theta5 = nan("");
        double theta6 = nan("");

        // c5 > 1
        if (c5 > (1 - eps)) {
            theta4 = 0;
            theta5 = 0;
            theta6 = atan2(-mtx(0,1), mtx(1,1)) - M_PI;
            theta6 = pmp(theta6);

            if (test_limits(robot, theta6, 5)) {
                J.col(n_sol_J) << J3.col(i), theta4, theta5, theta6;
                J.col(n_sol_J+1) << J3.col(i), theta4, theta5, theta6;
            }

            n_sol_J+=2;

        // c5 < -1
        } else if (c5 < (-1 + eps)) {
            n_sol_J+=2;
            continue;
        } else {
            double theta5a = acos(c5);
            double theta5b = -theta5a;

            double sg5a = sgn(sin(theta5a));
            double sg5b = -sg5a;

            double theta4a = atan2(-mtx(1,2) * sg5a, -mtx(0,2) * sg5a);
            double theta6a = atan2(-mtx(2,1) * sg5a, mtx(2,0) * sg5a) - M_PI;

            // <+pi, -pi>
            theta6a = pmp(theta6a);

            if (test_limits(robot, theta4a, 3) &&
                test_limits(robot, theta5a, 4) &&
                test_limits(robot, theta6a, 5))
            {
                J.col(n_sol_J) << J3.col(i), theta4a, theta5a, theta6a;
            }
            n_sol_J++;

            double theta4b = atan2(-mtx(1,2) * sg5b, -mtx(0,2) * sg5b);
            double theta6b = atan2(-mtx(2,1) * sg5b, mtx(2,0) * sg5b) - M_PI;

            // <+pi, -pi>
            theta6b = pmp(theta6b);

            if (test_limits(robot, theta4b, 3) &&
                test_limits(robot, theta5b, 4) &&
                test_limits(robot, theta6b, 5))
            {
                J.col(n_sol_J) << J3.col(i), theta4b, theta5b, theta6b;
            }
            n_sol_J++;
        }
    }

    for (size_t i = 0; i < 8; i++) {
        J.col(i) = remove_offset_and_direction(robot, J.col(i));
    }

    return J;
}


Matrix<double, 6, 1> ikt_solve_theta46(const Robot * robot, Isometry3d mtx, double theta1, double theta2, double theta3, double theta5);

Matrix<double, 6, 2> solve_theta456(const Robot * robot, Isometry3d T, double theta1, double theta2, double theta3) {

  Matrix<double, 6, 2> J;
  J.fill(nan(""));

  Isometry3d M = robot->base;
  M = M * mtx_dh(robot->DH.col(0) + (theta1 * robot->DH_Param.col(0)));
  M = M * mtx_dh(robot->DH.col(1) + (theta2 * robot->DH_Param.col(1)));
  M = M * mtx_dh(robot->DH.col(2) + (theta3 * robot->DH_Param.col(2)));

  Isometry3d mtx = M.inverse() *  T;
  double c5 = mtx(2,2);
  double theta4 = nan("");
  double theta5 = nan("");
  double theta6 = nan("");

  if (abs(c5 - 1) < eps) {
    theta4 = 0;
    theta5 = 0;
    theta6 = atan2(-mtx(0,1), mtx(1,1)) - M_PI;
    theta6 = pmp(theta6);

    if (test_limits(robot, theta6, 5)) {
      J.col(0) << theta1, theta2, theta3, theta4, theta5, theta6;
      J.col(1) << theta1, theta2, theta3, theta4, theta5, theta6;
    }
  } else if (abs(c5 + 1) < eps) {
    // No solution
  } else {
    c5 = min(max(c5, -1.0), 1.0);

    double theta5a = acos(c5);
    double theta5b = -theta5a;

    J.col(0) << ikt_solve_theta46(robot, mtx, theta1, theta2, theta3, theta5a);
    J.col(1) << ikt_solve_theta46(robot, mtx, theta1, theta2, theta3, theta5b);
  }

  return J;
}

Matrix<double, 6, 1> ikt_solve_theta46(const Robot * robot, Isometry3d mtx, double theta1, double theta2, double theta3, double theta5) {


  Matrix<double, 6, 1> J;
  double sg5 = sgn(theta5);

  double theta4 = atan2(-mtx(1,2) * sg5, -mtx(0,2) * sg5);
  double theta6 = atan2(-mtx(2,1) * sg5, mtx(2,0) * sg5) - M_PI;

  // <+pi, -pi>
  theta6 = pmp(theta6);

  if (test_limits(robot, theta4, 3) &&
      test_limits(robot, theta5, 4) &&
      test_limits(robot, theta6, 5))
  {
    J.col(0) << theta1, theta2, theta3, theta4, theta5, theta6;
  }

  return J;
}



