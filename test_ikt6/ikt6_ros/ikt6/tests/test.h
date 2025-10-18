#ifndef IKT6_TEST_H_
#define IKT6_TEST_H_
// Includes {{{

#include <iostream>
#include <stdio.h>
#include <chrono>
#include "ikt6.h"

using std::chrono::high_resolution_clock;
using std::chrono::time_point;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

// }}}
// Typedefs and constants {{{

typedef struct {

    int total_tested;
    int total_solutions;
    int correct;
    int incorrect;
    int nosolution;
    double dkt_time_ms;
    double ikt_time_ms;

} TestStats_TypeDef;

typedef Matrix<int, 6, 1> Vector6i;

IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

// }}}
// Function prototypes {{{

/**
 * Test robot configuration
 *
 */
TestStats_TypeDef run_test(Robot *robot, Vector6i zerocheck, Vector6i zeroset, size_t ntests, bool debug);
bool check_limits(Robot *robot, Vector6d J);
TestStats_TypeDef test_stats_init();
void print_test_stats(TestStats_TypeDef stats);

// }}}
// Implementation {{{


TestStats_TypeDef test_stats_init() {

    TestStats_TypeDef stats;
    stats.total_tested = 0;
    stats.total_solutions = 0;
    stats.correct = 0;
    stats.incorrect = 0;
    stats.dkt_time_ms = 0;
    stats.ikt_time_ms = 0;
    stats.nosolution = 0;

    return stats;
}

bool check_limits(Robot *robot, Vector6d J) {

    for (size_t i = 0; i < 6; i++) {
        if (J(i) < robot->limits_min(i) || J(i) > robot->limits_max(i)) {
            return false;
        }
    }
    return true;
}



TestStats_TypeDef run_test(Robot *robot, Vector6i zerocheck, Vector6i zeroset, size_t ntests, bool debug) {

    Vector6d J_in;
    Matrix<double, 6, Dynamic> J_out;
    Vector6d P_in, P_out;

    TestStats_TypeDef stats = test_stats_init();
    while (stats.total_tested < ntests) {

        // generate random joint state
        // TODO: investigate why it generates points outside the limits
        for (size_t j = 0; j < 6; j++) {
            J_in(j) = ((double) rand()/ (double)RAND_MAX - 0.5) * (robot->limits_max[j] - robot->limits_min[j]);
        }

        // check for singularities or set perposefully
        for (size_t j = 0; j < 6; j++) {
            if (zerocheck(j) == 1 && abs(J_in(3)) < eps) {
                continue;
            }

            if (zeroset(j) == 1) {
                J_in(j) = 0;
            }
        }

        // check limits
        if (! check_limits(robot, J_in)) {
            continue;
        }

        // update number of tested points
        stats.total_tested++;

        // record start of  dkt
        auto dkt_t0 = high_resolution_clock::now();

        // calculate direct kinematics
        P_in = ikt6_dkt(robot, J_in);


        // record end of dkt
        auto dkt_t1 = high_resolution_clock::now();
        duration<double, std::milli> dkt_dur = dkt_t1 - dkt_t0;
        stats.dkt_time_ms += dkt_dur.count();


        // record start of ikt
        auto ikt_t0 = high_resolution_clock::now();

        // calculate inverse kinematics
        J_out = ikt6_ikt(robot, P_in);

        // record end of ikt
        auto ikt_t1 = high_resolution_clock::now();
        duration<double, std::milli> ikt_dur = ikt_t1 - ikt_t0;
        stats.ikt_time_ms += ikt_dur.count();


        // check number of solutions
        if (J_out.cols() <= 0) {
            stats.nosolution++;
            if (debug) {
                std::cout << "!!! No solution !!!" << std::endl;
                std::cout << "J_in  = " << J_in.transpose() << std::endl;
                std::cout << "P_in  = " << P_in.transpose() << std::endl;
            }
        }

        // check solutions
        for (size_t j = 0; j < J_out.cols(); j++) {

            if (J_out.col(j).hasNaN()) {
                stats.nosolution++;
                continue;
            }

            stats.total_solutions++;

            P_out = ikt6_dkt(robot, J_out.col(j));

            double diff = (abs(P_in.array() - P_out.array())).sum();

            if (diff < eps * 10000) {
                stats.correct++;
            } else {
                stats.incorrect++;
                if (debug) {
                    std::cout << "!!! Solution out of limits !!!" << std::endl;
                    std::cout << "J_in  = " << J_in.transpose() << std::endl;
                    std::cout << "P_in  = " << P_in.transpose() << std::endl;
                    std::cout << "J_out  = " << J_out.col(j).transpose() << std::endl;
                    std::cout << "P_out = " << P_out.transpose() << std::endl;
                }
            }

        }
    }

    return stats;
}


void print_test_stats(TestStats_TypeDef stats) {
    printf("Total tested configurations                % 8d\n", stats.total_tested);
    printf("Total solutions                            % 8d\n", stats.total_solutions);
    printf("No solution count                          % 8d\n", stats.nosolution);
    printf("Correct count                              % 8d\n", stats.correct);
    printf("Incorrect count                            % 8d\n", stats.incorrect);
    if (stats.total_solutions > 0) {
        printf("Successfulness:                            % 9.2f %%\n", stats.correct / (stats.total_solutions) * 100.0);
    }
    printf("Total dkt time:                            % 9.4f ms\n", stats.dkt_time_ms);
    printf("Single dkt time (average):                 % 9.4f ms\n", stats.dkt_time_ms / (double)stats.total_tested);
    printf("Total ikt time:                            % 9.4f ms\n", stats.ikt_time_ms);
    printf("Single ikt time (average):                 % 9.4f ms\n", stats.ikt_time_ms / (double)stats.total_tested);
}

// }}}
#endif
