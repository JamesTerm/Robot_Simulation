// Ian: TransportSmoke is a headless smoke test for transport backends.
// Usage:
//   DriverStation_TransportSmoke [run_ms] [--mode nativelink|nt4|direct|legacy]
//                                         [--startup-delay-ms N] [--test-move N]
//
// Default mode is nativelink (original behavior).
// Adding --mode nt4 starts the NT4 WebSocket server on port 5810
// so any NT4 dashboard (Shuffleboard, Glass, etc.) can connect and verify data flow.

#include "stdafx.h"

#include "NativeLink.h"
#include "Robot_Tester.h"
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#include <Windows.h>

#include <cstdlib>
#include <cmath>
#include <string>
#include <thread>

static ConnectionMode ParseConnectionMode(const std::string& modeStr)
{
	if (modeStr == "nt4" || modeStr == "shuffleboard" || modeStr == "shuffle")
		return ConnectionMode::eNetworkTablesV4;
	if (modeStr == "direct")
		return ConnectionMode::eDirectConnect;
	if (modeStr == "legacy")
		return ConnectionMode::eLegacySmartDashboard;
	// Default: nativelink
	return ConnectionMode::eNativeLink;
}

static const char* ConnectionModeLabel(ConnectionMode mode)
{
	switch (mode)
	{
	case ConnectionMode::eLegacySmartDashboard: return "legacy";
	case ConnectionMode::eDirectConnect:        return "direct";
	case ConnectionMode::eNetworkTablesV4:     return "nt4";
	case ConnectionMode::eNativeLink:           return "nativelink";
	default:                                    return "unknown";
	}
}

int smoke_main(int argc, char** argv);

// Ian: SEH filter to catch access violations and other structured exceptions
// that don't go through C++ exception handling.
static LONG WINAPI UnhandledExceptionFilter_Smoke(EXCEPTION_POINTERS* pExInfo)
{
	fprintf(stderr, "[TransportSmoke] FATAL: Unhandled exception code=0x%08lX at address=%p\n",
		pExInfo->ExceptionRecord->ExceptionCode,
		pExInfo->ExceptionRecord->ExceptionAddress);
	fflush(stderr);
	return EXCEPTION_EXECUTE_HANDLER;
}

int main(int argc, char** argv)
{
	SetUnhandledExceptionFilter(UnhandledExceptionFilter_Smoke);
	try
	{
		return smoke_main(argc, argv);
	}
	catch (const std::exception& ex)
	{
		fprintf(stderr, "[TransportSmoke] FATAL: Unhandled C++ exception: %s\n", ex.what());
		fflush(stderr);
		return 1;
	}
	catch (...)
	{
		fprintf(stderr, "[TransportSmoke] FATAL: Unhandled unknown exception\n");
		fflush(stderr);
		return 1;
	}
}

int smoke_main(int argc, char** argv)
{
	const NativeLink::ServerConfig nativeLinkConfig = NativeLink::LoadServerConfigFromEnvironment();
	DWORD runMs = 500;
	DWORD startupDelayMs = 0;
	double testMove = 3.5;
	ConnectionMode connectionMode = ConnectionMode::eNativeLink;

	// Ian: Parse all args uniformly — named flags can appear in any order,
	// first bare positional number is treated as run_ms for backward compat.
	bool runMsSet = false;
	for (int i = 1; i < argc; ++i)
	{
		const std::string arg = argv[i] ? argv[i] : "";
		if (arg == "--mode" && (i + 1) < argc)
		{
			connectionMode = ParseConnectionMode(argv[++i]);
		}
		else if (arg == "--startup-delay-ms" && (i + 1) < argc)
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
		else if (!runMsSet && arg[0] != '-')
		{
			// Ian: Bare positional number = run duration (backward compat)
			const long parsed = std::strtol(arg.c_str(), nullptr, 10);
			if (parsed > 0)
			{
				runMs = static_cast<DWORD>(parsed);
				runMsSet = true;
			}
		}
	}

	// Ian: For NT4 mode, default to 60 seconds so there's time for
	// dashboards to connect and see data.  The 500ms default is fine for
	// automated tests against the other backends.
	if (!runMsSet && connectionMode == ConnectionMode::eNetworkTablesV4)
		runMs = 60000;

	RobotTester tester;
	tester.RobotTester_create();
	tester.SetConnectionMode(connectionMode);
	// Ian: SetConnectionMode() already calls DashboardTransportRouter::Initialize()
	// via SetMode(), which starts the transport backend (e.g. NT4 server on port
	// 5810 for NT4 mode).  RobotTester_init() then calls Initialize() again,
	// but DashboardTransportRouter safely deduplicates via m_is_initialized flag —
	// no second NT4 server is created, no port conflict.
	// The init() call is needed for TeleAuton_V2::init() which creates the
	// simulation model, AI goal system, and OSG viewer.  This requires a display
	// session (interactive or RDP) — will crash if truly headless.
	tester.RobotTester_init();

	printf("[TransportSmoke] mode=%s run_ms=%lu\n",
		ConnectionModeLabel(connectionMode),
		static_cast<unsigned long>(runMs));

	if (connectionMode == ConnectionMode::eNativeLink)
	{
		printf("[TransportSmoke] native_link carrier=%s channel=%s host=%s port=%u\n",
			NativeLink::ToString(nativeLinkConfig.carrierKind),
			nativeLinkConfig.channelId.c_str(),
			nativeLinkConfig.host.c_str(),
			static_cast<unsigned>(nativeLinkConfig.port));
	}

	if (startupDelayMs > 0)
	{
		printf("[TransportSmoke] startup_delay_ms=%lu\n", static_cast<unsigned long>(startupDelayMs));
		Sleep(startupDelayMs);
	}

	// Ian: Seed the chooser and test values — these exercise the full publish path
	// regardless of which backend is active.
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
	printf("[TransportSmoke] seeded chooser selected='Just Move Forward' TestMove=%g\n", testMove);
	fflush(stdout);

	// Ian: Start the auton sequence.  SetGameMode(0) latches auton mode,
	// StartStreaming() launches the OSG viewer render loop on an async thread
	// and activates the auton goal (which reads the chooser selection and
	// TestMove).  TeleAuton publishes real sim telemetry at ~60fps.
	tester.SetGameMode(0); // auton
	tester.StartStreaming();

	// Ian: Monitor loop — TeleAuton publishes real sim telemetry at ~60fps on
	// the viewer thread, so we just read back and print to observe the robot.
	printf("[TransportSmoke] running for %lu ms ...\n", static_cast<unsigned long>(runMs));
	fflush(stdout);
	const DWORD loopIntervalMs = 100;  // 10 Hz
	DWORD elapsed = 0;
	int loopCount = 0;
	while (elapsed < runMs)
	{
		Sleep(loopIntervalMs);
		elapsed += loopIntervalMs;
		loopCount++;

		// Ian: Print telemetry readback every second so we can observe the
		// simulation state from the console.
		if (loopCount % 10 == 0)
		{
			double y_ft = 0.0;
			double testMoveRead = 0.0;
			SmartDashboard::TryGetNumber("Y_ft", y_ft);
			SmartDashboard::TryGetNumber("TestMove", testMoveRead);
			const std::string selected = SmartDashboard::GetString(
				"Test/Auton_Selection/AutoChooser/selected", "(none)");
			printf("[TransportSmoke] t=%.1fs  Y_ft=%.3f  TestMove=%.1f  selected='%s'\n",
				elapsed / 1000.0, y_ft, testMoveRead, selected.c_str());
			fflush(stdout);
		}
	}
	printf("[TransportSmoke] loop finished (%d iterations)\n", loopCount);
	fflush(stdout);

	tester.StopStreaming();
	tester.Shutdown();

	printf("[TransportSmoke] done\n");
	return 0;
}
