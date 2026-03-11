#include <gtest/gtest.h>

#include <cmath>

#include "Modules/Robot/DriveKinematics/DriveKinematics/Vehicle_Drive.h"

namespace
{
bool nearly_equal(double a, double b, double eps = 1e-5)
{
    return std::fabs(a - b) <= eps;
}
}

TEST(DriveKinematicsTests, TankDriveForwardSetsEqualWheelVelocities)
{
    Module::Robot::Tank_Drive tank;
    tank.Init();
    tank.UpdateVelocities(1.0, 0.0);

    EXPECT_TRUE(nearly_equal(tank.GetLeftVelocity(), 1.0));
    EXPECT_TRUE(nearly_equal(tank.GetRightVelocity(), 1.0));
}

TEST(DriveKinematicsTests, TankDriveRotateProducesDifferentialWheels)
{
    Module::Robot::Tank_Drive tank;
    tank.Init();
    tank.UpdateVelocities(0.0, 1.0);

    EXPECT_GT(tank.GetLeftVelocity(), tank.GetRightVelocity());
}
