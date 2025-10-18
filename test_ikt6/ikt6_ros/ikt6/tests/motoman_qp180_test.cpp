#include "ikt6.h"
#include "test.h"

int main() {

    Vector6d lengths, offsets, limits_max, directions, limits_min;

    lengths <<     650,     325,    1150,  300,  1590,    225;     // Lenghts
    offsets <<       0,       0,  M_PI/2,    0,     0, M_PI/2;     // Joint offsets in Rad
    directions <<    1,       1,      -1,   -1,    -1,     -1;     // Joint directions
    limits_max <<  180,      76,      90,  360,   130,    360;     // Joint limits max in Rad
    limits_min << -180,     -60,     -86, -360,  -130,   -360;     // Joint limits min in Rad
    limits_max = limits_max * M_PI/180;
    limits_min = limits_min * M_PI/180;
    Isometry3d base = Isometry3d::Identity();
    Isometry3d tool = Isometry3d::Identity();

    auto robot = ikt6_robot_init( "Mitsubishi Melfa RV6S", lengths, offsets, directions, limits_max, limits_min, base, tool);

    // robot.DH << -M_PI/2,       0, -M_PI/2, M_PI/2, -M_PI/2,     0,
    //                 325,    1150,     300,      0,       0,     0,
    //                   0, -M_PI/2,       0,      0,       0,  M_PI,
    //                 650,       0,       0,   1590,       0,   225;

    // robot.DH_Param << 0, 0, 0, 0, 0, 0,
    //                   0, 0, 0, 0, 0, 0,
    //                   1, 1, -1, -1, -1, -1,
    //                   0, 0, 0, 0, 0, 0;


    bool debug = true;
    size_t ntests = 2000;

    Vector6i no_zero;
    Vector6i set_zero;

    printf("===============================================================================\n");
    printf("Testing inverse kinematics for %s\n", robot.name.c_str());
    printf("\nNo singularities\n");
    printf("===============================================================================\n");

    no_zero << 0, 0, 1, 0, 1, 0;
    set_zero << 0, 0, 0, 0, 0, 0;
    TestStats_TypeDef stats_ns = run_test(&robot, no_zero, set_zero, ntests, debug);
    print_test_stats(stats_ns);

    printf("\nSingularity on J5\n");

    no_zero << 0, 0, 1, 0, 0, 0;
    set_zero << 0, 0, 0, 0, 1, 0;
    printf("===============================================================================\n");
    TestStats_TypeDef stats_s5 = run_test(&robot, no_zero, set_zero, ntests, debug);
    print_test_stats(stats_s5);

    printf("\nSingularity on J3\n");
    no_zero << 0, 0, 0, 0, 1, 0;
    set_zero << 0, 0, 1, 0, 0, 0;
    printf("===============================================================================\n");
    TestStats_TypeDef stats_s3 = run_test(&robot, no_zero, set_zero, ntests, debug);
    print_test_stats(stats_s3);

    printf("\nSingularity on J5 and J3\n");
    printf("===============================================================================\n");
    no_zero << 0, 0, 0, 0, 0, 0;
    set_zero << 0, 0, 1, 0, 1, 0;
    TestStats_TypeDef stats_s5s3 = run_test(&robot, no_zero, set_zero, ntests, debug);
    print_test_stats(stats_s5s3);

    Vector6d J_in;
    J_in << 0,0,0,0,0,0;

    Vector6d P = ikt6_dkt(&robot, J_in);
    cout << "P = \n" << P << endl;
    P << 2140, 0, 2100, M_PI/2, 0, M_PI/2;
    cout << "P = \n" << P << endl;
    cout << "J = \n" << ikt6_ikt(&robot, P) << endl;

}
