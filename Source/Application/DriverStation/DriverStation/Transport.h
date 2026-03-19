#pragma once

#include <memory>

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
