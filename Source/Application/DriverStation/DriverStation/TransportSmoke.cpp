#include "stdafx.h"

#include "Robot_Tester.h"
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#include <Windows.h>

#include <thread>

int main()
{
    SmartDashboard::PutNumber("AutonTest", 1.0);

    RobotTester tester;
    tester.RobotTester_create();
    tester.SetConnectionMode(ConnectionMode::eDirectConnect);
    tester.RobotTester_init();

    tester.SetGameMode(0); // auton
    tester.StartStreaming();

    Sleep(500);

    tester.StopStreaming();
    tester.Shutdown();

    return 0;
}
