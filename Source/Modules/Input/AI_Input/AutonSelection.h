#pragma once

#include <chrono>
#include <functional>
#include <thread>

namespace Module
{
namespace Input
{
inline int ResolveAutonIndex(
    const std::function<bool(double&)>& tryReadSelection,
    int defaultIndex,
    int maxExclusive,
    size_t retryCount,
    std::chrono::milliseconds retryDelay)
{
    double selectionValue = static_cast<double>(defaultIndex);
    bool hasSelection = tryReadSelection(selectionValue);

    if (!hasSelection)
    {
        for (size_t i = 0; i < retryCount && !hasSelection; ++i)
        {
            if (retryDelay.count() > 0)
            {
                std::this_thread::sleep_for(retryDelay);
            }
            hasSelection = tryReadSelection(selectionValue);
        }
    }

    if (!hasSelection)
    {
        return defaultIndex;
    }

    int index = static_cast<int>(selectionValue);
    if (index < defaultIndex)
    {
        index = defaultIndex;
    }
    if (index >= maxExclusive)
    {
        index = defaultIndex;
    }
    return index;
}
}
}
