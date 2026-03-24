#pragma once

#include <fstream>
#include <mutex>
#include <string>

namespace Module
{
namespace Input
{
inline void AppendDirectAutonChainLog(const std::string& line)
{
    static std::mutex logMutex;
    static std::ofstream log("D:/code/Robot_Simulation/.debug/direct_auton_chain_log.txt", std::ios::out | std::ios::app);
    std::lock_guard<std::mutex> lock(logMutex);
    log << line << '\n';
    log.flush();
}
}
}
