// Ian: Transport.h is the central transport abstraction for the simulator.
// Each new dashboard integration (Legacy, Direct, Shuffleboard, Glass, etc.)
// adds a ConnectionMode enum value and a corresponding IConnectionBackend
// implementation in Transport.cpp.  The enum value also gates chooser and
// query-source behavior in AI_Input_Example.cpp — remember to update
// IsChooserEnabledForCurrentConnection() when adding a new mode.
#pragma once

#include <memory>

// Ian: Adding a new transport mode (e.g. Glass) requires:
//  1. New enum value here
//  2. New IConnectionBackend subclass in Transport.cpp
//  3. EnsureBackend() factory case in Transport.cpp
//  4. UsesLegacyTransportPath() updated (return false for new NT4-style transports)
//  5. IsChooserEnabledForCurrentConnection() in AI_Input_Example.cpp
//  6. DriverStation.cpp hotkey and menu entry
//  7. SmartDashboard-side plugin in plugins/<NewTransport>/
enum class ConnectionMode
{
	eLegacySmartDashboard = 0,
	eDirectConnect = 1,
	eShuffleboard = 2,
	eNativeLink = 3
};

const wchar_t* GetConnectionModeName(ConnectionMode mode);

class IConnectionBackend
{
public:
	virtual ~IConnectionBackend() = default;
	virtual void Initialize() = 0;
	virtual void Shutdown() = 0;
	virtual const wchar_t* GetBackendName() const = 0;
};

class DashboardTransportRouter
{
public:
	DashboardTransportRouter();
	void Initialize(ConnectionMode initial_mode);
	void SetMode(ConnectionMode mode);
	ConnectionMode GetMode() const;
	const wchar_t* GetActiveBackendName() const;
	void Shutdown();
private:
	ConnectionMode m_mode = ConnectionMode::eLegacySmartDashboard;
	bool m_is_initialized = false;
	std::unique_ptr<IConnectionBackend> m_backend;
	void EnsureBackend();
	static bool UsesLegacyTransportPath(ConnectionMode mode);
};
