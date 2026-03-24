#include <gtest/gtest.h>

#include "Modules/Robot/Entity2D/Entity2D/Entity2D.h"

TEST(Entity2DTests, GlobalVelocityMovesNorth)
{
    Module::Localization::Entity2D entity;
    entity.Reset(0.0, 0.0, 0.0);
    entity.SetLinearVelocity_global(1.0, 0.0);

    for (int i = 0; i < 100; ++i)
    {
        entity.TimeSlice(0.01);
    }

    const auto pos = entity.GetCurrentPosition();
    EXPECT_GT(pos.y, 0.0);
}

TEST(Entity2DTests, ResetRestoresState)
{
    Module::Localization::Entity2D entity;
    entity.Reset(2.0, -1.0, 0.5);
    const auto pos = entity.GetCurrentPosition();

    EXPECT_DOUBLE_EQ(pos.x, 2.0);
    EXPECT_DOUBLE_EQ(pos.y, -1.0);
    EXPECT_DOUBLE_EQ(entity.GetCurrentHeading(), 0.5);
}
