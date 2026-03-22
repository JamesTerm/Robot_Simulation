#include "stdafx.h"

#include "NativeLink.h"
#include "Robot_Tester.h"
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#include <Windows.h>

#include <cstdlib>
#include <string>
#include <thread>

int main(int argc, char** argv)
{
	const NativeLink::ServerConfig nativeLinkConfig = NativeLink::LoadServerConfigFromEnvironment();
	DWORD runMs = 500;
	DWORD startupDelayMs = 0;
	double testMove = 3.5;
	if (argc > 1)
	{
		const long parsed = std::strtol(argv[1], nullptr, 10);
		if (parsed > 0)
			runMs = static_cast<DWORD>(parsed);
	}
	for (int i = 2; i < argc; ++i)
	{
		const std::string arg = argv[i] ? argv[i] : "";
		if (arg == "--startup-delay-ms" && (i + 1) < argc)
		{
			const long parsed = std::strtol(argv[++i], nullptr, 10);
			if (parsed > 0)
				startupDelayMs = static_cast<DWORD>(parsed);
		}
		else if (arg == "--test-move" && (i + 1) < argc)
		{
			const double parsed = std::strtod(argv[++i], nullptr);
			if (parsed > 0.0)
				testMove = parsed;
		}
	}

	RobotTester tester;
	tester.RobotTester_create();
	tester.SetConnectionMode(ConnectionMode::eNativeLink);
	tester.RobotTester_init();
	printf("[TransportSmoke] native_link carrier=%s channel=%s host=%s port=%u\n",
		NativeLink::ToString(nativeLinkConfig.carrierKind),
		nativeLinkConfig.channelId.c_str(),
		nativeLinkConfig.host.c_str(),
		static_cast<unsigned>(nativeLinkConfig.port));

	if (startupDelayMs > 0)
	{
		printf("[TransportSmoke] startup_delay_ms=%lu\n", static_cast<unsigned long>(startupDelayMs));
		Sleep(startupDelayMs);
	}

	SmartDashboard::PutString("Test/Auton_Selection/AutoChooser/.type", "String Chooser");
	std::vector<std::string> chooserOptions;
	chooserOptions.push_back("Do Nothing");
	chooserOptions.push_back("Just Move Forward");
	chooserOptions.push_back("Just Rotate");
	chooserOptions.push_back("Move Rotate Sequence");
	chooserOptions.push_back("Box Waypoints");
	chooserOptions.push_back("Smart Waypoints");
	SmartDashboard::PutStringArray("Test/Auton_Selection/AutoChooser/options", chooserOptions);
	SmartDashboard::PutString("Test/Auton_Selection/AutoChooser/default", "Do Nothing");
	SmartDashboard::PutString("Test/Auton_Selection/AutoChooser/active", "Do Nothing");
	SmartDashboard::PutString("Test/Auton_Selection/AutoChooser/selected", "Just Move Forward");
	SmartDashboard::PutNumber("TestMove", testMove);
	printf("[TransportSmoke] seeded chooser selected='Just Move Forward' TestMove=%g run_ms=%lu\n", testMove, static_cast<unsigned long>(runMs));

	tester.SetGameMode(0); // auton
	tester.StartStreaming();

	Sleep(runMs);

	tester.StopStreaming();
	tester.Shutdown();

    return 0;
}
