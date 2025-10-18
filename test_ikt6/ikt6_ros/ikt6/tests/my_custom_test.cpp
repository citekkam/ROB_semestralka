#include "test.h"

/**
 * Simple example of how to use the test.h framework
 * to test a robot configuration
 */

int main() {

    std::cout << "===============================================================================" << std::endl;
    std::cout << "Custom Robot Test Example" << std::endl;
    std::cout << "===============================================================================" << std::endl;

    // Define robot parameters
    Vector6d lengths, offsets, directions, limits_max, limits_min;
    
    // Example: CRS93 parameters
    lengths <<     440,   0,  305,  0,  330,   211;      // Link lengths in mm
    offsets <<       0,    0,    0,    0,    0,    0;      // Joint offsets in radians
    directions <<    1,    -1,    -1,    1,    -1,    1;      // Joint directions (+1 or -1)
    limits_max <<  175,  90,  110,  180,  105,  180;      // Max limits in degrees
    limits_min << -175,  -90, -110, -180, -105, -180;      // Min limits in degrees
    
    // Convert limits to radians
    limits_max = limits_max * M_PI/180;
    limits_min = limits_min * M_PI/180;
    
    // Define base and tool transforms (identity for now)
    Isometry3d base = Isometry3d::Identity();
    Isometry3d tool = Isometry3d::Identity();
    
    // Initialize robot
    Robot robot = ikt6_robot_init(
        "CRS93", 
        lengths, 
        offsets, 
        directions, 
        limits_max, 
        limits_min, 
        base, 
        tool
    );

    // Test 1: Normal operation (no singularities)
    // std::cout << "\n=== Test 1: Normal Operation ===" << std::endl;
    // Vector6i zerocheck1, zeroset1;
    // zerocheck1 << 0, 0, 0, 0, 0, 0;  // Don't check for singularities
    // zeroset1 << 0, 0, 0, 0, 0, 0;    // Don't force any joints to zero
    
    size_t ntests = 1000;  // Number of random configurations to test
    bool debug = true;     // Set true to see detailed failure messages
    
    // TestStats_TypeDef stats1 = run_test(&robot, zerocheck1, zeroset1, ntests, debug);
    // print_test_stats(stats1);

    // Test 2: With J5 singularity
    // std::cout << "\n=== Test 2: J5 Singularity (J5 = 0) ===" << std::endl;
    // Vector6i zerocheck2, zeroset2;
    // zerocheck2 << 0, 0, 0, 0, 1, 0;  // Check if J5 is near zero
    // zeroset2 << 0, 0, 0, 0, 1, 0;    // Force J5 to zero
    
    // TestStats_TypeDef stats2 = run_test(&robot, zerocheck2, zeroset2, ntests, debug);
    // print_test_stats(stats2);

    // Test 3: Manual single test with known values
    std::cout << "\n=== Test 3: Manual Single Configuration ===" << std::endl;
    
    Vector6d J_test;
    J_test << 1.28114148e-01, -1.05680035e+00, -1.94032616e+00, -6.65644384e-03, -1.38230077e-01,  1.34653016e-01;  // L-shape configuratio
    std::cout << "Input joints: " << J_test.transpose() << std::endl;
    
    // Forward kinematics
    Vector6d P_test = ikt6_dkt(&robot, J_test);
    std::cout << "Forward kinematics result (x,y,z,r,p,y): " << P_test.transpose() << std::endl;

    Isometry3d T_test = ikt6_dkt_T(&robot, J_test);
    std::cout << "Forward kinematics transformation matrix T:\n" << T_test.matrix() << std::endl;
    
    // Inverse kinematics
    P_test(1) -= 100;  // Slightly perturb to avoid singularity issues
    Matrix<double, 6, Dynamic> J_solutions = ikt6_ikt(&robot, P_test);
    std::cout << "Number of IK solutions found: " << J_solutions.cols() << std::endl;
    
    // Verify each solution
    for (int i = 0; i < J_solutions.cols(); i++) {
        if (!J_solutions.col(i).hasNaN()) {
            Vector6d P_verify = ikt6_dkt(&robot, J_solutions.col(i));
            double error = (P_test - P_verify).norm();
            std::cout << "Solution " << i+1 << ": " << J_solutions.col(i).transpose() 
                      << " (error: " << error << ")" << std::endl;
        }
    }

    std::cout << "\n===============================================================================" << std::endl;
    std::cout << "Tests completed!" << std::endl;
    std::cout << "===============================================================================" << std::endl;

    return 0;
}
