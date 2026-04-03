#include <gtest/gtest.h>

#include "Modules/Input/AI_Input/AutonChooser.h"
#include "Modules/Input/AI_Input/AutonSelection.h"

#include <chrono>
#include <vector>

using Module::Input::AutonChooserOption;
using Module::Input::ResolveAutonSelectionFromLabel;
using Module::Input::ResolveAutonIndex;

TEST(AutonSelectionTests, UsesImmediateValueWhenAvailable)
{
    auto reader = [](double& value) -> bool
    {
        value = 1.0;
        return true;
    };

    const int index = ResolveAutonIndex(reader, 0, 6, 20, std::chrono::milliseconds(0));
    EXPECT_EQ(index, 1);
}

TEST(AutonSelectionTests, RetriesAndUsesLaterArrival)
{
    int calls = 0;
    auto reader = [&calls](double& value) -> bool
    {
        ++calls;
        if (calls < 4)
        {
            return false;
        }
        value = 3.0;
        return true;
    };

    const int index = ResolveAutonIndex(reader, 0, 6, 5, std::chrono::milliseconds(0));
    EXPECT_EQ(index, 3);
    EXPECT_GE(calls, 4);
}

TEST(AutonSelectionTests, DefaultsWhenMissing)
{
    auto reader = [](double&) -> bool
    {
        return false;
    };

    const int index = ResolveAutonIndex(reader, 0, 6, 3, std::chrono::milliseconds(0));
    EXPECT_EQ(index, 0);
}

TEST(AutonSelectionTests, ClampsOutOfRangeToDefault)
{
    auto reader = [](double& value) -> bool
    {
        value = 99.0;
        return true;
    };

	const int index = ResolveAutonIndex(reader, 0, 6, 2, std::chrono::milliseconds(0));
	EXPECT_EQ(index, 0);
}

// Ian: The ResolveAutonIndex tests above use maxExclusive=6 as an arbitrary
// bound for the generic numeric fallback -- they are not tied to the actual
// auton enum size (which is now 2).  They remain valid as protocol tests.

TEST(AutonSelectionTests, ChooserSelectionMapsLabelToIndex_Auton)
{
	// Matches the 2-option auton chooser layout
	const AutonChooserOption options[] =
	{
		{0, "Do Nothing"},
		{1, "Just Move Forward"}
	};

	EXPECT_EQ(ResolveAutonSelectionFromLabel("Just Move Forward", options, 2, 0), 1);
	EXPECT_EQ(ResolveAutonSelectionFromLabel("Do Nothing", options, 2, 0), 0);
	// Unknown label falls back to defaultIndex
	EXPECT_EQ(ResolveAutonSelectionFromLabel("Unknown", options, 2, 0), 0);
}

TEST(AutonSelectionTests, ChooserSelectionMapsLabelToIndex_TestDrive)
{
	// Matches the drive-test portion of the test chooser layout
	const AutonChooserOption options[] =
	{
		{0, "Do Nothing"},
		{1, "Just Move Forward"},
		{2, "Just Rotate"},
		{3, "Move Rotate Sequence"},
		{4, "Box Waypoints"},
		{5, "Smart Waypoints"}
	};

	EXPECT_EQ(ResolveAutonSelectionFromLabel("Just Rotate", options, 6, 0), 2);
	EXPECT_EQ(ResolveAutonSelectionFromLabel("Smart Waypoints", options, 6, 0), 5);
	EXPECT_EQ(ResolveAutonSelectionFromLabel("Unknown", options, 6, 0), 0);
}
