#ifndef IKT6_H_
#define IKT6_H_
// Defines {{{

#ifndef DEBUG
#define DEBUG 0
#endif /* ifndef DEBUG */

// }}}
// Includes {{{

#include <eigen3/Eigen/Dense>
#include "util.h"

#if DEBUG
#include <iostream>
Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
#endif

using namespace Eigen;
using namespace std;

// }}}
// Typedefs {{{

/** Robot parameters
  */
typedef struct {

    // Robot parameters, angles are in radians
    string name;
    Vector6d lengths;
    Vector6d offsets;
    Vector6d directions;
    Vector6d limits_max;
    Vector6d limits_min;

    // Calculated
    Matrix<double, 4, 6> DH;
    Matrix<double, 4, 6> DH_Param;
    Isometry3d A76;
    Isometry3d base;
    Isometry3d tool;

} Robot;

// }}}
// Function prototypes {{{

/** Computes direct kinematics chain from given robot parameters and joint angles.
 *
 *  Parameters:
 *    robot - robot parameters
 *    J - joint angles
 *
 *  Returns
 *    Cartesian position and orientation of the end of the tool
 *
 */
Vector6d ikt6_dkt(const Robot * robot, Vector6d J);

/** Computes direct kinematics chain from given robot parameters and joint angles.
 *
 *  Parameters:
 *    robot - robot parameters
 *    J - joint angles
 *
 *  Returns
 *    Transformation from world to tool.
 *
 */
Isometry3d ikt6_dkt_T(const Robot * robot, Vector6d J);

/** Computes inverse kinematics task for given robot parameters and target position.
 *
 * Parameters
 *  robot - robot parameters
 *  P - Cartesian position and orientation of the tool, in the world coordinate system.
 *
 * Returns
 *  Eight solutions of the inverse kinematics as an column vectors.
 *
 */
Matrix<double, 6, Dynamic> ikt6_ikt(const Robot * robot, const Vector6d & P);


/** Computes inverse kinematics task for given robot parameters and target position.
 *
 * Parameters
 *  robot - robot parameters
 *  T - transformation from world coordinate system to the desired tool coordinate system.
 *
 * Returns
 *  Eight solutions of the inverse kinematics as an column vectors.
 *
 */
Matrix<double, 6, Dynamic> ikt6_ikt(const Robot * robot, Isometry3d T);

/** Initialize Robot parameters.
 *
 * Parameters:
 *   name: Name of the robot.
 *   lengths: (l1, l2, l3, l4, l5, l6) see image in documentation
 *   offsets: Offset of each joint in radians.
 *   directions: Direction of each joint [1, -1]
 *   limits_max: Maximal allowed angle of each joint in radians.
 *   limits_min: Minimal allowed angle of each joint in radians.
 *   base: Transformation from the wold to the robot base.
 *   tool: Transformation from the end of the robot to the tool.
 *
 * Returns:
 *   robot: Robot parameters structure.
 *
 */
Robot ikt6_robot_init(
        string name,
        Vector6d lengths,
        Vector6d offsets,
        Vector6d directions,
        Vector6d limits_max,
        Vector6d limits_min,
        Isometry3d base,
        Isometry3d tool);

// }}}
#endif /* ifndef IKT6_H_ */
