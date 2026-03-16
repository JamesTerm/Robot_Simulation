#include "stdafx.h"

#include "Robot_Tester.h"
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#include <Windows.h>

#include <thread>

int main()
{
	SmartDashboard::PutString("Test/AutoChooser/.type", "String Chooser");
	SmartDashboard::PutString("Test/AutoChooser/options", "Do Nothing,Just Move Forward,Just Rotate,Move Rotate Sequence,Box Waypoints,Smart Waypoints");
	SmartDashboard::PutString("Test/AutoChooser/default", "Do Nothing");
	SmartDashboard::PutString("Test/AutoChooser/active", "Do Nothing");
	SmartDashboard::PutString("Test/AutoChooser/selected", "Just Move Forward");
	SmartDashboard::PutString("AutonTest", "Just Move Forward");

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
