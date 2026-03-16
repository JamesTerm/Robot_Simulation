#pragma once

#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#include <string>
#include <vector>

namespace Module
{
namespace Input
{
struct AutonChooserOption
{
    int index = 0;
    const char* label = "";
};

inline std::string FindAutonChooserSelection(
    const char* chooserBaseKey,
    const char* legacyFlatKey,
    const char* defaultLabel)
{
    const std::string chooserBase = chooserBaseKey ? chooserBaseKey : "";
    const std::string selectedKey = chooserBase + "/selected";
    const std::string activeKey = chooserBase + "/active";
    const std::string scopedLegacyKey = std::string("Test/") + (legacyFlatKey ? legacyFlatKey : "");

    try
    {
        const std::string selected = SmartDashboard::GetString(selectedKey);
        if (!selected.empty())
            return selected;
    }
    catch (...)
    {
    }

    try
    {
        const std::string active = SmartDashboard::GetString(activeKey);
        if (!active.empty())
            return active;
    }
    catch (...)
    {
    }

    auto tryLegacyString = [](const std::string& key, std::string& out) -> bool
    {
        try
        {
            out = SmartDashboard::GetString(key);
            return !out.empty();
        }
        catch (...)
        {
        }
        return false;
    };

    std::string legacySelection;
    if (tryLegacyString(scopedLegacyKey, legacySelection) || tryLegacyString(legacyFlatKey ? legacyFlatKey : "", legacySelection))
        return legacySelection;

    return defaultLabel ? std::string(defaultLabel) : std::string();
}

inline int ResolveAutonSelectionFromChooser(
    const char* chooserBaseKey,
    const char* legacyFlatKey,
    const AutonChooserOption* options,
    size_t optionCount,
    int defaultIndex)
{
    const char* defaultLabel = "";
    for (size_t i = 0; i < optionCount; ++i)
    {
        if (options[i].index == defaultIndex)
        {
            defaultLabel = options[i].label;
            break;
        }
    }

    const std::string selectedLabel = FindAutonChooserSelection(chooserBaseKey, legacyFlatKey, defaultLabel);
    for (size_t i = 0; i < optionCount; ++i)
    {
        if (selectedLabel == options[i].label)
            return options[i].index;
    }

    return defaultIndex;
}

inline int ResolveAutonSelectionFromLabel(
    const std::string& selectedLabel,
    const AutonChooserOption* options,
    size_t optionCount,
    int defaultIndex)
{
    for (size_t i = 0; i < optionCount; ++i)
    {
        if (selectedLabel == options[i].label)
            return options[i].index;
    }

    return defaultIndex;
}

inline void PublishAutonChooser(
    const char* chooserBaseKey,
    const AutonChooserOption* options,
    size_t optionCount,
    int defaultIndex,
    int activeIndex,
    int selectedIndex)
{
    const std::string chooserBase = chooserBaseKey ? chooserBaseKey : "";
    std::vector<std::string> labels;
    labels.reserve(optionCount);

    std::string defaultLabel;
    std::string activeLabel;
    std::string selectedLabel;

    for (size_t i = 0; i < optionCount; ++i)
    {
        labels.push_back(options[i].label);
        if (options[i].index == defaultIndex)
            defaultLabel = options[i].label;
        if (options[i].index == activeIndex)
            activeLabel = options[i].label;
        if (options[i].index == selectedIndex)
            selectedLabel = options[i].label;
    }

    if (activeLabel.empty())
        activeLabel = defaultLabel;
    if (selectedLabel.empty())
        selectedLabel = activeLabel.empty() ? defaultLabel : activeLabel;

    SmartDashboard::PutString(chooserBase + "/.type", "String Chooser");
    SmartDashboard::PutStringArray(chooserBase + "/options", labels);
    SmartDashboard::PutString(chooserBase + "/default", defaultLabel);
    SmartDashboard::PutString(chooserBase + "/active", activeLabel);
    SmartDashboard::PutString(chooserBase + "/selected", selectedLabel);
}
}
}
