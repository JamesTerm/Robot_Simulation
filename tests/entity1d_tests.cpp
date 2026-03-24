#include <gtest/gtest.h>

#include "Modules/Robot/Entity1D/Entity1D/Entity1D.h"

TEST(Entity1DTests, PositiveVelocityAdvancesPosition)
{
    Module::Localization::Entity1D entity;
    entity.Reset(0.0);
    entity.SetVelocity(1.0);

    for (int i = 0; i < 100; ++i)
    {
        entity.TimeSlice(0.01);
    }

    EXPECT_GT(entity.GetCurrentPosition(), 0.0);
}

TEST(Entity1DTests, ZeroVelocityHoldsPosition)
{
    Module::Localization::Entity1D entity;
    entity.Reset(3.0);
    entity.SetVelocity(0.0);

    for (int i = 0; i < 100; ++i)
    {
        entity.TimeSlice(0.01);
    }

    EXPECT_DOUBLE_EQ(entity.GetCurrentPosition(), 3.0);
}
