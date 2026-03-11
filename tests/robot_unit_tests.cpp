#include <cmath>
#include <cstdio>

#include "../Source/Modules/Robot/Entity1D/Entity1D/Entity1D.h"
#include "../Source/Modules/Robot/Entity2D/Entity2D/Entity2D.h"
#include "../Source/Modules/Robot/DriveKinematics/DriveKinematics/Vehicle_Drive.h"

namespace
{
    bool nearly_equal(double a, double b, double eps = 1e-6)
    {
        return std::fabs(a - b) <= eps;
    }

    int run_entity1d_tests()
    {
        Module::Localization::Entity1D entity;
        entity.Reset(0.0);
        entity.SetVelocity(1.0);
        for (int i = 0; i < 100; ++i)
        {
            entity.TimeSlice(0.01);
        }

        if (!(entity.GetCurrentPosition() > 0.0))
        {
            std::printf("Entity1D test failed: expected positive position\n");
            return 1;
        }
        return 0;
    }

    int run_entity2d_tests()
    {
        Module::Localization::Entity2D entity;
        entity.Reset(0.0, 0.0, 0.0);
        entity.SetLinearVelocity_global(1.0, 0.0);
        for (int i = 0; i < 100; ++i)
        {
            entity.TimeSlice(0.01);
        }

        const auto pos = entity.GetCurrentPosition();
        if (!(pos.y > 0.0))
        {
            std::printf("Entity2D test failed: expected positive north displacement\n");
            return 1;
        }
        return 0;
    }

    int run_drivekinematics_tests()
    {
        Module::Robot::Tank_Drive tank;
        tank.Init();
        tank.UpdateVelocities(1.0, 0.0);

        if (!nearly_equal(tank.GetLeftVelocity(), 1.0, 1e-5) || !nearly_equal(tank.GetRightVelocity(), 1.0, 1e-5))
        {
            std::printf("DriveKinematics test failed: left/right velocity mismatch\n");
            return 1;
        }
        return 0;
    }
}

int main()
{
    if (run_entity1d_tests() != 0)
    {
        return 1;
    }
    if (run_entity2d_tests() != 0)
    {
        return 1;
    }
    if (run_drivekinematics_tests() != 0)
    {
        return 1;
    }

    std::printf("robot_unit_tests: all tests passed\n");
    return 0;
}
