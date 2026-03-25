// Ian: NT4ClientTest — a standalone WebSocket client that mimics ntcore's exact
// connection sequence (as read from allwpilib NetworkClient.cpp).  Purpose:
// diagnose why NT4 dashboards connect but never send subscribe messages.
//
// This test client does exactly what ntcore does:
//   1. Connect to ws://localhost:5810/nt/testclient  with subprotocol header
//   2. Log the 101 response headers (specifically Sec-WebSocket-Protocol)
//   3. On Open: log protocol, then immediately send a subscribe JSON message
//   4. On Text: log incoming text (should be announce messages)
//   5. On Binary: log incoming binary data length (should be value frames)
//   6. On Close/Error: log reason
//
// Usage:
//   NT4ClientTest.exe [--port 5810] [--duration-ms 10000]
//
// Run this while DriverStation_TransportSmoke is running in NT4 mode.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>

// Ian: vcpkg's ixwebsocket package adds include/ixwebsocket as the include dir,
// so we include without the ixwebsocket/ prefix (unlike NT4Server.cpp which gets
// the parent include/ dir from transitive deps).
#include <IXNetSystem.h>
#include <IXWebSocket.h>

static void Log(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	// Timestamp
	auto now = std::chrono::steady_clock::now();
	static auto start = now;
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
	printf("[%6lld ms] ", static_cast<long long>(ms));
	vprintf(fmt, args);
	printf("\n");
	fflush(stdout);
	va_end(args);
}

int main(int argc, char** argv)
{
	int port = 5810;
	int durationMs = 15000;

	for (int i = 1; i < argc; ++i)
	{
		std::string arg = argv[i] ? argv[i] : "";
		if (arg == "--port" && (i + 1) < argc)
			port = std::atoi(argv[++i]);
		else if (arg == "--duration-ms" && (i + 1) < argc)
			durationMs = std::atoi(argv[++i]);
	}

	Log("NT4ClientTest starting — connecting to ws://localhost:%d/nt/testclient", port);
	Log("Duration: %d ms", durationMs);

	// Ian: IXWebSocket requires WSAStartup on Windows
	ix::initNetSystem();

	ix::WebSocket ws;
	std::string url = "ws://localhost:" + std::to_string(port) + "/nt/testclient";
	ws.setUrl(url);

	// Ian: This is the critical part — ntcore sends two subprotocols.
	// The server MUST echo exactly one back, or ntcore drops the connection.
	// IXWebSocket client sets the Sec-WebSocket-Protocol header from this.
	ws.addSubProtocol("v4.1.networktables.first.wpi.edu");
	ws.addSubProtocol("networktables.first.wpi.edu");

	// Ian: Disable per-message deflate — ntcore doesn't use it and it complicates debugging
	ix::WebSocketPerMessageDeflateOptions deflateOpts(false);
	ws.setPerMessageDeflateOptions(deflateOpts);

	// Ian: Disable automatic reconnection — we want to see exactly one
	// connection attempt's lifecycle.
	ws.disableAutomaticReconnection();

	std::atomic<bool> gotOpen{false};
	std::atomic<bool> gotClose{false};
	std::atomic<int> textCount{0};
	std::atomic<int> binaryCount{0};

	ws.setOnMessageCallback([&](const ix::WebSocketMessagePtr& msg) {
		switch (msg->type)
		{
		case ix::WebSocketMessageType::Open:
		{
			gotOpen = true;
			Log("=== OPEN ===");
			Log("  URI:      %s", msg->openInfo.uri.c_str());
			Log("  Protocol: '%s'", msg->openInfo.protocol.c_str());
			Log("  Headers from server 101 response:");
			for (auto& kv : msg->openInfo.headers)
			{
				Log("    %s: %s", kv.first.c_str(), kv.second.c_str());
			}

			// Ian: Check for the critical Sec-WebSocket-Protocol header
			bool foundProtocolHeader = false;
			for (auto& kv : msg->openInfo.headers)
			{
				// IXWebSocket lowercases header names
				if (kv.first == "sec-websocket-protocol")
				{
					foundProtocolHeader = true;
					Log("  >>> Sec-WebSocket-Protocol header IS present: '%s'", kv.second.c_str());
				}
			}
			if (!foundProtocolHeader)
			{
				Log("  >>> WARNING: Sec-WebSocket-Protocol header is MISSING from 101 response!");
				Log("  >>> This is likely the root cause — ntcore will Terminate(1002, \"invalid response\")");
			}

			// Ian: Now do exactly what ntcore does after open:
			// 1. Send an RTT ping (binary frame with id=-1, timestamp)
			// 2. Send subscribe for all topics (prefix match on empty string = everything)
			//
			// Step 1: RTT ping — ntcore sends MsgPack [int(-1), int(0), int(2), int(timestamp)]
			// For our diagnostic purposes, we skip this — ntcore blocks on RTT before
			// sending control messages, but we're testing if the server even receives text.
			// Let's send the subscribe immediately.

			// Step 2: Subscribe to everything (matches ntcore's LocalStorage initial subscribe)
			std::string subscribeMsg =
				R"([{"method":"subscribe","params":{"topics":[""],"options":{"prefix":true},"subuid":1}}])";
			Log("  Sending subscribe: %s", subscribeMsg.c_str());
			ws.send(subscribeMsg, false);  // false = text frame
			Log("  Subscribe sent successfully");
			break;
		}

		case ix::WebSocketMessageType::Message:
		{
			if (msg->binary)
			{
				int count = ++binaryCount;
				if (count <= 10 || count % 50 == 0)
				{
					Log("<<< Binary frame: %zu bytes (total binary frames: %d)",
						msg->str.size(), count);
				}
			}
			else
			{
				int count = ++textCount;
				// Log all text messages — these should be announce/unannounce
				std::string preview = msg->str.substr(0, 500);
				Log("<<< Text frame #%d (%zu bytes): %s%s",
					count, msg->str.size(),
					preview.c_str(),
					msg->str.size() > 500 ? "..." : "");
			}
			break;
		}

		case ix::WebSocketMessageType::Close:
		{
			gotClose = true;
			Log("=== CLOSE === code=%d reason='%s'",
				msg->closeInfo.code, msg->closeInfo.reason.c_str());
			break;
		}

		case ix::WebSocketMessageType::Error:
		{
			Log("=== ERROR === retries=%d wait=%d reason='%s' http_status=%d",
				msg->errorInfo.retries,
				msg->errorInfo.wait_time,
				msg->errorInfo.reason.c_str(),
				msg->errorInfo.http_status);
			break;
		}

		case ix::WebSocketMessageType::Ping:
			Log("<<< Ping (%zu bytes)", msg->str.size());
			break;

		case ix::WebSocketMessageType::Pong:
			Log("<<< Pong (%zu bytes)", msg->str.size());
			break;

		case ix::WebSocketMessageType::Fragment:
			Log("<<< Fragment (%zu bytes)", msg->str.size());
			break;
		}
	});

	Log("Connecting...");
	ws.start();

	// Wait for connection or timeout
	auto startTime = std::chrono::steady_clock::now();
	auto deadline = startTime + std::chrono::milliseconds(durationMs);

	while (std::chrono::steady_clock::now() < deadline)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// Print periodic status
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now() - startTime).count();
		if (elapsed > 0 && elapsed % 2000 < 100)
		{
			auto state = ws.getReadyState();
			const char* stateStr = "unknown";
			switch (state)
			{
			case ix::ReadyState::Connecting: stateStr = "CONNECTING"; break;
			case ix::ReadyState::Open:       stateStr = "OPEN"; break;
			case ix::ReadyState::Closing:    stateStr = "CLOSING"; break;
			case ix::ReadyState::Closed:     stateStr = "CLOSED"; break;
			}
			Log("Status: state=%s open=%d text=%d binary=%d close=%d",
				stateStr,
				gotOpen.load() ? 1 : 0,
				textCount.load(),
				binaryCount.load(),
				gotClose.load() ? 1 : 0);
		}

		if (gotClose.load())
		{
			Log("Connection closed — exiting early");
			break;
		}
	}

	Log("Shutting down...");
	ws.stop();
	ix::uninitNetSystem();

	Log("=== SUMMARY ===");
	Log("  Connection opened: %s", gotOpen.load() ? "YES" : "NO");
	Log("  Text frames received: %d", textCount.load());
	Log("  Binary frames received: %d", binaryCount.load());
	Log("  Connection closed: %s", gotClose.load() ? "YES" : "NO");

	if (!gotOpen.load())
	{
		Log("  DIAGNOSIS: Connection never opened — server may not be running or port is wrong");
	}
	else if (textCount.load() == 0 && binaryCount.load() == 0)
	{
		Log("  DIAGNOSIS: Connected but received no data — server may not be sending announces/values");
	}
	else if (textCount.load() > 0 && binaryCount.load() == 0)
	{
		Log("  DIAGNOSIS: Got announces but no binary data — server may have no values to send");
	}
	else
	{
		Log("  DIAGNOSIS: Full data flow working — %d announces, %d value frames",
			textCount.load(), binaryCount.load());
	}

	return 0;
}
