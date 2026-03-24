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

TEST(AutonSelectionTests, ChooserSelectionMapsLabelToIndex)
{
	const AutonChooserOption options[] =
	{
		{0, "Do Nothing"},
		{1, "Just Move Forward"},
		{2, "Just Rotate"}
	};

	EXPECT_EQ(ResolveAutonSelectionFromLabel("Just Move Forward", options, 3, 2), 1);
	EXPECT_EQ(ResolveAutonSelectionFromLabel("Unknown", options, 3, 2), 2);
}
