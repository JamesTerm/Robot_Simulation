#pragma once
// Ian: MjpegServer is a minimal MJPEG-over-HTTP streaming server on port 1181.
// It uses ix::SocketServer (from IXWebSocket) for the TCP accept loop and
// per-connection threading, but does NOT use ix::HttpServer because that class
// is strictly request-response (callback returns a complete HttpResponsePtr).
// MJPEG requires a long-lived connection with an infinite multipart/x-mixed-replace
// response, so we subclass SocketServer directly and own the raw socket.
//
// Architecture:
//   - Each HTTP client connection gets its own thread (SocketServer does this).
//   - handleConnection() reads the HTTP request line, sends the MJPEG response
//     headers (200 OK, multipart/x-mixed-replace; boundary=<X>), then enters a
//     blocking loop waiting for frames from the producer (SimCameraSource).
//   - PushFrame() is called by the producer on its own thread.  It copies the
//     JPEG data into a shared buffer and signals all waiting client threads via
//     a condition variable.
//   - Each client thread wakes up, copies the JPEG data, and writes it as an
//     MJPEG boundary-delimited part over the socket.  If the write fails
//     (client disconnected), the thread exits and SocketServer cleans it up.
//
// Protocol compatibility notes:
//   - SmartDashboard's MjpegStreamSource expects:
//       Content-Type: multipart/x-mixed-replace; boundary=<X>
//     with each part delimited by "--<X>\r\n" + part headers + "\r\n\r\n" + data.
//   - The boundary is sent WITHOUT leading dashes in the Content-Type header;
//     the client strips leading dashes and then searches for "--<boundary>".
//   - Each part includes Content-Type: image/jpeg and Content-Length headers.
//   - The URL path is "/?action=stream" (matching mjpg-streamer convention).

#include <ixwebsocket/IXSocketServer.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

class MjpegServer final : public ix::SocketServer
{
public:
	// Ian: Port 1181 is the standard FRC camera stream port (cscore default).
	// The host "0.0.0.0" binds to all interfaces so that both localhost and
	// LAN connections work.
	explicit MjpegServer(int port = 1181,
	                     const std::string& host = "0.0.0.0");
	~MjpegServer() override;

	// Ian: Signal all client handler threads to exit immediately.
	// This sets m_stopping and wakes every thread blocked on m_frameCondition
	// so they break out of the frame-push loop without waiting for the next
	// 1-second timeout.  Call this BEFORE ix::SocketServer::stop() to ensure
	// client threads are already unwinding when stop() tries to join them.
	// Without this, shutdown can stall for up to 1 second per connected client.
	void SignalStop();

	// Ian: Push a new JPEG frame to all connected clients.
	// Called by the producer (SimCameraSource) on its own thread.
	// The data is copied internally; the caller can reuse the buffer immediately.
	// Thread-safe: uses m_frameMutex + m_frameCondition to synchronize with
	// client handler threads.
	void PushFrame(const std::vector<uint8_t>& jpegData);
	void PushFrame(const uint8_t* jpegData, size_t jpegSize);

	// Ian: Number of currently connected and streaming clients.
	size_t GetClientCount() const;

private:
	// ix::SocketServer pure virtuals
	void handleConnection(std::unique_ptr<ix::Socket> socket,
	                      std::shared_ptr<ix::ConnectionState> connectionState) override;
	size_t getConnectedClientsCount() override;

	// Ian: The MJPEG boundary string.  Chosen to be distinctive and unlikely
	// to appear in JPEG data.  The Content-Type header sends this WITHOUT
	// leading dashes; each part delimiter is "--" + boundary.
	static constexpr const char* kBoundary = "mjpegstream";

	// Ian: Shared frame buffer.  The producer writes here, client threads read.
	// m_frameSequence is a monotonically increasing counter so that client threads
	// can detect new frames (they compare their local sequence to m_frameSequence
	// and wait on m_frameCondition if equal).
	mutable std::mutex m_frameMutex;
	std::condition_variable m_frameCondition;
	std::vector<uint8_t> m_currentFrame;
	uint64_t m_frameSequence = 0;

	// Ian: Client count tracking.  Incremented/decremented in handleConnection.
	mutable std::mutex m_clientMutex;
	size_t m_clientCount = 0;

	// Ian: Shutdown signal.  When set to true, client handler threads break out
	// of the frame-push loop immediately instead of waiting for the next timeout
	// or write failure.  This is set by SignalStop() and checked in handleConnection().
	std::atomic<bool> m_stopping{false};
};
