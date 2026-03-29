// Ian: Transport.h is the central transport abstraction for the simulator.
// Each new dashboard integration (Legacy, Direct, NetworkTables V4, etc.)
// adds a ConnectionMode enum value and a corresponding IConnectionBackend
// implementation in Transport.cpp.  The enum value also gates chooser and
// query-source behavior in AI_Input_Example.cpp — remember to update
// IsChooserEnabledForCurrentConnection() when adding a new mode.
#pragma once

#include <memory>
#include <string>

// Ian: Adding a new transport mode requires:
//  1. New enum value here
//  2. New IConnectionBackend subclass in Transport.cpp
//  3. EnsureBackend() factory case in Transport.cpp
//  4. UsesLegacyTransportPath() updated (return false for new NT4-style transports)
//  5. HasDirectTransport() AND UsesNetworkTablesTransport() in SmartDashboard.cpp
//     — any mode with a DirectPublishSink must be listed or PutNumber/etc. will
//       also start the legacy NT2 server on port 1735
//  6. IsChooserEnabledForCurrentConnection() in AI_Input_Example.cpp
//  7. DriverStation.cpp hotkey and menu entry
//  8. SmartDashboard-side plugin in plugins/<NewTransport>/
enum class ConnectionMode
{
	eLegacySmartDashboard = 0,
	eDirectConnect = 1,
	eNetworkTablesV4 = 2,    // Ian: was eShuffleboard — renamed because Shuffleboard, Glass,
	                          // and any NT4-speaking dashboard all use the same backend
	eNativeLink = 3
};

// Ian: VideoSourceMode controls which frame source feeds the MJPEG server.
// The MJPEG server (port 1181) stays alive for any non-Off mode; only the
// source pushing frames into it changes.  "Off" tears down the server entirely
// and publishes CameraPublisher/connected = false.
//
// Ian: OWNERSHIP CHANGE — as of the transport-agnostic-video refactor, the
// MJPEG server and frame sources are owned by DashboardTransportRouter, NOT
// by individual backends.  This means video streaming works on ANY transport
// mode (Direct, NativeLink, NT4, Legacy).  Backends only participate in
// camera discovery key publication via PublishCameraDiscoveryKeys().
enum class VideoSourceMode
{
	eOff = 0,             // No camera stream — MJPEG server shut down
	eCamera = 1,          // USB webcam via Video for Windows (VFW)
	eSyntheticRadar = 2,  // Existing SimCameraSource radar sweep pattern
	eVirtualField = 3     // "The Grid" — Tron-style first-person virtual field (TronGridSource)
};

const wchar_t* GetVideoSourceModeName(VideoSourceMode mode);

const wchar_t* GetConnectionModeName(ConnectionMode mode);

// Forward declarations for video infrastructure (owned by DashboardTransportRouter)
class MjpegServer;
class SimCameraSource;
class WebCameraSource;
class TronGridSource;

class IConnectionBackend
{
public:
	virtual ~IConnectionBackend() = default;
	virtual void Initialize() = 0;
	virtual void Shutdown() = 0;
	virtual const wchar_t* GetBackendName() const = 0;

	// Ian: Camera discovery key publication.  The router calls these after
	// starting/stopping a video source so that dashboards (SmartDashboard,
	// Shuffleboard, Glass) can auto-discover the MJPEG stream.
	//
	// Each backend publishes keys in its own protocol:
	//   - NT4Backend: publishes /CameraPublisher/SimCamera/streams etc. via NT4
	//   - DirectConnectBackend: publishes via shared-memory ring buffer
	//   - NativeLinkBackend: publishes via NativeLink topics
	//   - LegacyBackend: no-op (legacy dashboards don't support CameraPublisher)
	//
	// Ian: LESSON LEARNED — these are "best effort" notifications.  The MJPEG
	// server is already listening on port 1181 before these are called.  A
	// dashboard that knows the URL can connect even without discovery keys.
	virtual void PublishCameraDiscoveryKeys(const std::string& /*sourceDescription*/) {}
	virtual void ClearCameraDiscoveryKeys() {}
};

class DashboardTransportRouter
{
public:
	DashboardTransportRouter();
	~DashboardTransportRouter();
	void Initialize(ConnectionMode initial_mode);
	void SetMode(ConnectionMode mode);
	ConnectionMode GetMode() const;
	const wchar_t* GetActiveBackendName() const;
	void Shutdown();

	// Ian: Video source is now owned by the router — works on any transport mode.
	// The router manages the MJPEG server and frame sources directly, and delegates
	// camera discovery key publication to the active backend.
	void SetVideoSource(VideoSourceMode mode);
	VideoSourceMode GetVideoSource() const;

	// Ian: Value query — used by TronGridSource's position callback to read
	// robot position from whichever backend is active.  Delegates to the
	// backend's SmartDashboardDirectQuerySource interface via dynamic_cast.
	// Returns false if the backend doesn't support queries or the key is missing.
	bool TryGetNumber(const std::string& key, double& value) const;

private:
	ConnectionMode m_mode = ConnectionMode::eLegacySmartDashboard;
	bool m_is_initialized = false;
	std::unique_ptr<IConnectionBackend> m_backend;
	void EnsureBackend();
	static bool UsesLegacyTransportPath(ConnectionMode mode);

	// Ian: Video infrastructure — previously owned by NT4Backend, now owned by
	// the router so video works on any transport mode.
	//
	// Ian: LESSON LEARNED — these must be unique_ptrs, not inline members, because
	// MjpegServer inherits from ix::SocketServer which is not movable/copyable.
	std::unique_ptr<MjpegServer> m_mjpegServer;
	std::unique_ptr<SimCameraSource> m_syntheticSource;
	std::unique_ptr<WebCameraSource> m_webCamSource;
	std::unique_ptr<TronGridSource> m_tronGridSource;
	VideoSourceMode m_videoMode = VideoSourceMode::eOff;

	// Ian: Video lifecycle helpers — extracted from NT4Backend.
	bool StartMjpegServer();
	void StopMjpegServer();
	void StopCurrentSource();
};
