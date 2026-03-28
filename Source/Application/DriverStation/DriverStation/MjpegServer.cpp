// Ian: MjpegServer.cpp — MJPEG-over-HTTP streaming server implementation.
// See MjpegServer.h for architecture and protocol compatibility notes.
//
// The core idea: ix::SocketServer gives us a TCP accept loop with one thread
// per connection.  handleConnection() receives the raw ix::Socket after accept.
// We parse the minimal HTTP request, send MJPEG response headers, then enter
// a frame-push loop driven by a condition variable.
//
// Ian: LESSON LEARNED — ix::HttpServer is strictly request-response (the callback
// must return a complete HttpResponsePtr).  MJPEG requires an infinite streaming
// response, so we bypass HttpServer entirely and subclass SocketServer directly.
// This gives us raw socket access for the long-lived connection.

#include "MjpegServer.h"

#include <ixwebsocket/IXSocket.h>

#include <cstdio>
#include <sstream>

#ifdef _WIN32
#include <Windows.h>  // OutputDebugStringA
#endif

namespace
{
	void DebugLog(const char* fmt, ...)
	{
		char buf[512];
		va_list args;
		va_start(args, fmt);
		vsnprintf(buf, sizeof(buf), fmt, args);
		va_end(args);
#ifdef _WIN32
		OutputDebugStringA(buf);
#endif
	}
}

MjpegServer::MjpegServer(int port, const std::string& host)
	: ix::SocketServer(port, host)
{
}

MjpegServer::~MjpegServer()
{
	SignalStop();
	stop();
}

// Ian: Wake all client handler threads so they exit immediately.
// The sequence is:  set m_stopping → notify condition variable → threads
// wake up, see m_stopping, break out of the loop.  This must happen BEFORE
// ix::SocketServer::stop() which joins the handler threads — otherwise those
// threads can block on wait_for for up to 1 second per timeout cycle.
void MjpegServer::SignalStop()
{
	m_stopping.store(true);
	m_frameCondition.notify_all();
}

void MjpegServer::PushFrame(const std::vector<uint8_t>& jpegData)
{
	PushFrame(jpegData.data(), jpegData.size());
}

void MjpegServer::PushFrame(const uint8_t* jpegData, size_t jpegSize)
{
	if (jpegData == nullptr || jpegSize == 0)
		return;

	{
		std::lock_guard<std::mutex> lock(m_frameMutex);
		m_currentFrame.assign(jpegData, jpegData + jpegSize);
		++m_frameSequence;
	}
	m_frameCondition.notify_all();
}

size_t MjpegServer::GetClientCount() const
{
	std::lock_guard<std::mutex> lock(m_clientMutex);
	return m_clientCount;
}

size_t MjpegServer::getConnectedClientsCount()
{
	return GetClientCount();
}

// Ian: handleConnection runs on a dedicated thread per client (managed by ix::SocketServer).
// The thread lifetime IS the connection lifetime — when this function returns, the socket
// closes and SocketServer's GC thread joins this thread.
//
// Protocol flow:
//   1. Read the HTTP request (we only care about GET /?action=stream)
//   2. Send HTTP 200 with Content-Type: multipart/x-mixed-replace; boundary=<kBoundary>
//   3. Loop forever: wait for a new frame, write it as an MJPEG part
//   4. On write failure → client disconnected → return
void MjpegServer::handleConnection(
	std::unique_ptr<ix::Socket> socket,
	std::shared_ptr<ix::ConnectionState> connectionState)
{
	// Ian: Track client count for diagnostics.
	{
		std::lock_guard<std::mutex> lock(m_clientMutex);
		++m_clientCount;
	}
	DebugLog("[MjpegServer] Client connected (%zu total)\n", GetClientCount());

	// --- Step 1: Read the HTTP request ---
	// Ian: We do a minimal HTTP/1.x request parse.  We only need to consume the
	// request line and headers so we can send our response.  We don't actually
	// validate the method/path because we serve the same MJPEG stream for any
	// GET request — this matches mjpg-streamer behavior.
	//
	// Read line-by-line until we see the blank line ending the headers.
	// ix::Socket doesn't have a readLine that works without a CancellationRequest,
	// so we use the one with a cancellation that checks the stopping flag.
	// Ian: This lambda serves double duty — it's the ix::Socket CancellationRequest
	// for readLine/writeBytes AND gives us an early exit during shutdown instead of
	// blocking until the next I/O timeout.
	auto shouldCancel = [this]() -> bool { return m_stopping.load(); };

	bool requestValid = false;
	while (true)
	{
		auto [ok, line] = socket->readLine(shouldCancel);
		if (!ok)
		{
			DebugLog("[MjpegServer] Failed to read HTTP request line\n");
			break;
		}
		// Blank line (just \r\n or \n) marks end of headers
		if (line.empty() || line == "\r" || line == "\r\n")
		{
			requestValid = true;
			break;
		}
	}

	if (!requestValid)
	{
		std::lock_guard<std::mutex> lock(m_clientMutex);
		--m_clientCount;
		DebugLog("[MjpegServer] Client disconnected during request parse (%zu remaining)\n", m_clientCount);
		return;
	}

	// --- Step 2: Send the MJPEG response headers ---
	// Ian: The response format must match what SmartDashboard's MjpegStreamSource expects:
	//   HTTP/1.1 200 OK\r\n
	//   Content-Type: multipart/x-mixed-replace; boundary=<kBoundary>\r\n
	//   Cache-Control: no-cache\r\n
	//   Connection: close\r\n
	//   Pragma: no-cache\r\n
	//   \r\n
	//
	// Ian: The boundary in the Content-Type header is sent WITHOUT leading dashes.
	// MjpegStreamSource::ParseBoundaryFromContentType strips any leading dashes
	// and then searches for "--<boundary>" in the stream.
	std::ostringstream responseHeaders;
	responseHeaders << "HTTP/1.1 200 OK\r\n"
	                << "Content-Type: multipart/x-mixed-replace; boundary=" << kBoundary << "\r\n"
	                << "Cache-Control: no-cache\r\n"
	                << "Connection: close\r\n"
	                << "Pragma: no-cache\r\n"
	                << "\r\n";

	const std::string headerStr = responseHeaders.str();
	if (!socket->writeBytes(headerStr, shouldCancel))
	{
		DebugLog("[MjpegServer] Failed to send response headers\n");
		std::lock_guard<std::mutex> lock(m_clientMutex);
		--m_clientCount;
		return;
	}

	// --- Step 3: Frame push loop ---
	// Ian: Each MJPEG part looks like:
	//   --<boundary>\r\n
	//   Content-Type: image/jpeg\r\n
	//   Content-Length: <size>\r\n
	//   \r\n
	//   <JPEG data>\r\n
	//
	// We wait on m_frameCondition for new frames and push them out.
	// The loop exits when a write fails (client disconnected) or the server stops.

	uint64_t lastSequence = 0;

	while (!m_stopping.load())
	{
		std::vector<uint8_t> frameCopy;

		{
			std::unique_lock<std::mutex> lock(m_frameMutex);
			// Ian: Wait for a new frame.  Use wait_for with a timeout so we can
			// periodically check if the connection is still alive even if no frames
			// are being produced.  1 second timeout is generous — at 15fps we'd
			// normally wake every ~66ms.  The predicate also checks m_stopping so
			// that SignalStop() → notify_all() wakes us immediately for shutdown.
			m_frameCondition.wait_for(lock, std::chrono::seconds(1), [&]()
			{
				return m_stopping.load() || m_frameSequence > lastSequence;
			});

			if (m_stopping.load())
				break;

			if (m_frameSequence == lastSequence)
			{
				// Timeout — no new frame.  Check if the socket is still writable.
				// We'll just try to continue; the next write attempt will catch
				// a dead connection.
				continue;
			}

			lastSequence = m_frameSequence;
			frameCopy = m_currentFrame;
		}

		if (frameCopy.empty())
			continue;

		// Build the MJPEG part
		std::ostringstream partHeader;
		partHeader << "--" << kBoundary << "\r\n"
		           << "Content-Type: image/jpeg\r\n"
		           << "Content-Length: " << frameCopy.size() << "\r\n"
		           << "\r\n";

		const std::string partHeaderStr = partHeader.str();

		// Write part header
		if (!socket->writeBytes(partHeaderStr, shouldCancel))
		{
			DebugLog("[MjpegServer] Client disconnected (header write failed)\n");
			break;
		}

		// Write JPEG data
		// Ian: ix::Socket::writeBytes takes a std::string, so we need to convert.
		// This is a copy, but at 320x240 JPEG (~10-30KB) it's negligible.
		const std::string jpegStr(reinterpret_cast<const char*>(frameCopy.data()), frameCopy.size());
		if (!socket->writeBytes(jpegStr, shouldCancel))
		{
			DebugLog("[MjpegServer] Client disconnected (data write failed)\n");
			break;
		}

		// Write trailing CRLF after the JPEG data (before the next boundary)
		if (!socket->writeBytes("\r\n", shouldCancel))
		{
			DebugLog("[MjpegServer] Client disconnected (trailing CRLF write failed)\n");
			break;
		}
	}

	// --- Step 4: Cleanup ---
	{
		std::lock_guard<std::mutex> lock(m_clientMutex);
		--m_clientCount;
	}
	DebugLog("[MjpegServer] Client disconnected (%zu remaining)\n", GetClientCount());

	// Ian: CRITICAL — tell ix::SocketServer's GC thread that this handler is done.
	// Without this call, the GC thread's closeTerminatedThreads() never joins our
	// thread (it only joins threads whose ConnectionState::isTerminated() is true).
	// That means getConnectionsThreadsCount() never reaches 0, the GC loop in
	// runGC() spins forever, and SocketServer::stop() deadlocks on _gcThread.join().
	// This was the root cause of the zombie DriverStation process — the window
	// closed, Shutdown() called stop(), but stop() hung waiting for the GC thread.
	connectionState->setTerminated();
}
