# IKT6 Testing Guide

## Overview

This document explains what the `test.h` testing framework does and how to test the IKT6 inverse kinematics library for 6-DOF robotic manipulators.

## What Does `test.h` Do?

### Purpose
The `test.h` file provides a **comprehensive validation framework** for testing inverse kinematics solutions by verifying that:
1. Forward kinematics (DKT) produces correct cartesian positions from joint angles
2. Inverse kinematics (IKT) finds all valid joint angle solutions for a given cartesian position
3. The IKT solutions, when fed back through DKT, reproduce the original cartesian position

### How It Works

```
Random Joint Angles (J_in) 
    ↓ [Forward Kinematics: ikt6_dkt()]
Cartesian Position (P_in)
    ↓ [Inverse Kinematics: ikt6_ikt()]
Solution Set (J_out) [up to 8 solutions]
    ↓ [Verify: Forward Kinematics on each solution]
Cartesian Positions (P_out)
    ↓ [Compare: |P_in - P_out| < epsilon]
PASS or FAIL
```

### Key Components

#### 1. `TestStats_TypeDef` Structure
Tracks comprehensive statistics:
- **total_tested**: Number of random configurations tested
- **total_solutions**: Total IK solutions found across all tests
- **correct**: Solutions that correctly reproduce the target position
- **incorrect**: Solutions that fail to match the target
- **nosolution**: Cases where IK couldn't find a solution
- **dkt_time_ms**: Total time spent in forward kinematics
- **ikt_time_ms**: Total time spent in inverse kinematics

#### 2. `run_test()` Function
The main testing loop:
- Generates random joint angles within robot limits
- Optionally sets specific joints to zero (for singularity testing)
- Validates each solution by round-trip (DKT → IKT → DKT)
- Measures computation time
- Collects statistics

#### 3. `check_limits()` Function
Ensures generated joint angles are within robot's physical limits

#### 4. `print_test_stats()` Function
Displays formatted test results including success rate and timing

## How to Test IKT6 Functions

### Method 1: Run Pre-built Tests (Quick Start)

Build and run all tests:

```bash
cd /home/david/School/rob/test_ikt6/ikt6_ros/ikt6
mkdir -p build && cd build
cmake ..
make
ctest --verbose
```

**What this tests:**
- Basic assertions (hello_test)
- Utility functions (mtx_translate, etc.)
- Melfa RV6S robot DKT and IKT

### Method 2: Run Comprehensive Robot Tests

Run the full validation suite for a specific robot:

```bash
cd /home/david/School/rob/test_ikt6/ikt6_ros/ikt6/build

# Test Mitsubishi Melfa RV6S
./test_melfa

# Or run individual robot tests
./motoman_gp88_test
./motoman_qp180_test
```

**What this outputs:**
```
Total tested configurations                    2000
Total solutions                                9009
No solution count                              6991
Correct count                                  9009
Incorrect count                                   0
Successfulness:                               100.00 %
Total dkt time:                              95.7360 ms
Single dkt time (average):                    0.0479 ms
Total ikt time:                             321.0084 ms
Single ikt time (average):                    0.1605 ms
```

### Method 3: Write Your Own Test

Create a new test file following the pattern:

```cpp
#include "test.h"

int main() {
    // 1. Define robot parameters
    Vector6d lengths, offsets, directions, limits_max, limits_min;
    
    lengths <<     350,   85,  280,  100,  315,   85;      // Link lengths (mm)
    offsets <<       0,    0,    0,    0,    0,    0;      // Joint offsets (rad)
    directions <<    1,    1,    1,    1,    1,    1;      // Joint directions
    limits_max <<  170,  135,  166,  160,  120,  360;      // Max limits (degrees)
    limits_min << -170,  -92, -107, -160, -120, -360;      // Min limits (degrees)
    
    // Convert to radians
    limits_max = limits_max * M_PI/180;
    limits_min = limits_min * M_PI/180;
    
    // Define base and tool transforms
    Isometry3d base = Isometry3d::Identity();
    Isometry3d tool = Isometry3d::Identity();
    
    // 2. Initialize robot
    Robot robot = ikt6_robot_init(
        "My Robot", 
        lengths, 
        offsets, 
        directions, 
        limits_max, 
        limits_min, 
        base, 
        tool
    );
    
    // 3. Run tests
    Vector6i zerocheck, zeroset;
    zerocheck << 0, 0, 0, 0, 0, 0;  // Don't check for singularities
    zeroset << 0, 0, 0, 0, 0, 0;    // Don't force zeros
    
    size_t ntests = 2000;  // Number of random configurations
    bool debug = false;     // Set true for detailed output
    
    TestStats_TypeDef stats = run_test(&robot, zerocheck, zeroset, ntests, debug);
    
    // 4. Print results
    print_test_stats(stats);
    
    return 0;
}
```

### Method 4: Google Test Integration (Unit Tests)

For more granular testing using Google Test:

```cpp
#include "ikt6.h"
#include <gtest/gtest.h>

class MyRobotTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize robot parameters
        Vector6d lengths, offsets, directions, limits_max, limits_min;
        // ... (set parameters)
        
        robot = ikt6_robot_init("MyRobot", lengths, offsets, 
                                directions, limits_max, limits_min, 
                                base, tool);
    }
    
    Robot robot;
};

TEST_F(MyRobotTest, TestZeroPosition) {
    Vector6d j;
    j << 0, 0, 0, 0, 0, 0;
    Vector6d pos = ikt6_dkt(&robot, j);
    
    // Assert expected position
    EXPECT_NEAR(pos(0), expected_x, 1e-6);
    EXPECT_NEAR(pos(1), expected_y, 1e-6);
    // ... etc
}

TEST_F(MyRobotTest, TestInverseKinematics) {
    Vector6d target_pos;
    target_pos << x, y, z, roll, pitch, yaw;
    
    Matrix<double, 6, Dynamic> solutions = ikt6_ikt(&robot, target_pos);
    
    EXPECT_GT(solutions.cols(), 0);  // At least one solution
    
    // Verify each solution
    for (int i = 0; i < solutions.cols(); i++) {
        Vector6d result_pos = ikt6_dkt(&robot, solutions.col(i));
        EXPECT_NEAR((target_pos - result_pos).norm(), 0, 1e-6);
    }
}
```

## Testing Strategies

### 1. Normal Operation Testing
Test with no singularities:
```cpp
Vector6i zerocheck, zeroset;
zerocheck << 0, 0, 0, 0, 0, 0;
zeroset << 0, 0, 0, 0, 0, 0;
run_test(&robot, zerocheck, zeroset, 2000, false);
```

### 2. Singularity Testing
Force specific joints to zero (e.g., J5 singularity):
```cpp
Vector6i zerocheck, zeroset;
zerocheck << 0, 0, 0, 0, 1, 0;  // Check J5
zeroset << 0, 0, 0, 0, 1, 0;    // Set J5 = 0
run_test(&robot, zerocheck, zeroset, 2000, false);
```

### 3. Debug Mode
Enable detailed output for failures:
```cpp
run_test(&robot, zerocheck, zeroset, 100, true);  // debug = true
```

## Key Functions Reference

### Core Functions

```cpp
// Initialize robot with parameters
Robot ikt6_robot_init(
    string name,
    Vector6d lengths,        // Link lengths
    Vector6d offsets,        // Joint angle offsets
    Vector6d directions,     // Joint rotation directions (+1 or -1)
    Vector6d limits_max,     // Max joint limits (radians)
    Vector6d limits_min,     // Min joint limits (radians)
    Isometry3d base,         // Base transform
    Isometry3d tool          // Tool transform
);

// Forward kinematics: Joint angles → Cartesian position
Vector6d ikt6_dkt(const Robot *robot, Vector6d J);
// Returns: [x, y, z, roll, pitch, yaw]

// Inverse kinematics: Cartesian position → Joint angles
Matrix<double, 6, Dynamic> ikt6_ikt(const Robot *robot, const Vector6d &P);
// Returns: Up to 8 solution columns [j1, j2, j3, j4, j5, j6]
```

## Interpreting Results

### Good Results
```
Successfulness:                               100.00 %
Correct count                                  9009
Incorrect count                                   0
```
✅ All solutions correctly reproduce the target position

### Warnings
```
No solution count                              6991
```
⚠️ Some configurations couldn't be solved (may be unreachable or in singularity)

### Problems
```
Incorrect count                                 123
Successfulness:                                98.65 %
```
❌ Some solutions don't match the target - possible numerical issues or bugs

## Common Issues

1. **High "No solution" count**: 
   - May indicate singularities or unreachable positions
   - Check robot limits and DH parameters

2. **Incorrect solutions**:
   - Numerical precision issues
   - Bug in IK algorithm
   - Enable debug mode to see specific failures

3. **NaN values**:
   - Singularity configurations
   - Division by zero in IK solver

## Additional Resources

- **Main README**: `/home/david/School/rob/test_ikt6/ikt6_ros/ikt6/README.md`
- **Test Examples**: `/home/david/School/rob/test_ikt6/ikt6_ros/ikt6/tests/`
- **Robot Parameters Figure**: `./fig/robot_parameters.png`

## Quick Command Reference

```bash
# Build everything
cd ikt6 && mkdir -p build && cd build && cmake .. && make

# Run all unit tests
ctest --verbose

# Run specific robot test
./test_melfa
./motoman_gp88_test

# Run with different number of tests (edit source and recompile)
# In test file, change: size_t ntests = 10000;
```
