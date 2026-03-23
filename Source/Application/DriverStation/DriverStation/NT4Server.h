#pragma once
// Ian: NT4Server is the NetworkTables 4.x WebSocket protocol server for Shuffleboard
// compatibility.  It runs an IXWebSocket server on port 5810 and speaks the NT4 binary
// (MessagePack) + JSON control protocol that official WPILib dashboards expect.
//
// Design notes:
//   - This is *server-side only* — the simulator publishes topics and Shuffleboard
//     subscribes to them.  We do NOT need to subscribe to remote topics yet (iteration 1).
//   - The MessagePack encoder is custom / minimal (~100 lines) because we only need
//     to *encode* a small subset of types: bool, double, int, float, string, string[].
//   - IXWebSocket handles WebSocket framing, PING/PONG, and connection lifecycle.
//   - NT4 subprotocol negotiation is handled in the Open callback by checking the
//     Sec-WebSocket-Protocol header from the client request.
//
// Usage:
//   NT4Server server;
//   server.Start();                  // begins listening on port 5810
//   server.Publish("Velocity", 3.14);
//   server.PublishBool("SafetyLock_Drive", true);
//   server.PublishString("status", "ok");
//   server.PublishStringArray("options", {"A","B","C"});
//   server.Stop();

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

// Forward-declare IXWebSocket types to avoid header pollution.
namespace ix { class WebSocket; class WebSocketServer; }

class NT4Server
{
public:
	NT4Server();
	~NT4Server();

	// Ian: Start/Stop control the WebSocket server lifecycle.
	// Start() binds to 0.0.0.0:5810 and begins accepting connections.
	// Stop() gracefully shuts down all connections and the listener.
	bool Start(int port = 5810);
	void Stop();
	bool IsRunning() const;

	// --- Publishing API (server → all connected clients) ---
	// These queue values and broadcast as NT4 binary frames.
	// Topic names should use NT4 convention: "/SmartDashboard/<key>"
	// The server auto-announces topics to new clients on connect.
	void PublishDouble(const std::string& topicName, double value);
	void PublishBoolean(const std::string& topicName, bool value);
	void PublishString(const std::string& topicName, const std::string& value);
	void PublishStringArray(const std::string& topicName, const std::vector<std::string>& values);
	void PublishInt(const std::string& topicName, int64_t value);

private:
	// Ian: NT4 data type codes per the spec.
	// These appear in both the announce JSON and in the binary MessagePack frame.
	enum class NT4Type : uint8_t
	{
		Boolean     = 0,
		Double      = 1,
		Int         = 2,
		Float       = 3,
		String      = 4,
		Raw         = 5,   // also msgpack, protobuf
		BooleanArr  = 16,
		DoubleArr   = 17,
		IntArr      = 18,
		FloatArr    = 19,
		StringArr   = 20
	};

	static const char* NT4TypeString(NT4Type type);

	struct TopicInfo
	{
		int32_t id = -1;           // server-assigned topic ID
		std::string name;          // e.g. "/SmartDashboard/Velocity"
		NT4Type type = NT4Type::Double;
		bool announced = false;    // true once first announce sent to any client
	};

	// Ian: Retained value cache — we keep the latest value for each topic so that
	// newly connecting clients get a full snapshot (Shuffleboard expects this).
	struct RetainedValue
	{
		NT4Type type = NT4Type::Double;
		bool boolVal = false;
		double doubleVal = 0.0;
		int64_t intVal = 0;
		std::string stringVal;
		std::vector<std::string> stringArrayVal;
	};

	// Core server
	std::unique_ptr<ix::WebSocketServer> m_server;
	bool m_running = false;

	// Topic management (protected by m_topicsMutex)
	mutable std::mutex m_topicsMutex;
	std::unordered_map<std::string, TopicInfo> m_topics;          // name → info
	int32_t m_nextTopicId = 0;

	// Retained values (protected by m_topicsMutex — same lock)
	std::unordered_map<std::string, RetainedValue> m_retained;

	// --- Internal helpers ---
	TopicInfo& GetOrCreateTopic(const std::string& name, NT4Type type);
	void AnnounceAllTopicsTo(void* ws);  // ws is ix::WebSocket*
	void SendValueToAllClients(const TopicInfo& topic, const RetainedValue& value);
	std::string BuildAnnounceJson(const TopicInfo& topic) const;

	// --- Minimal MessagePack encoder ---
	// Ian: These produce raw bytes per the MessagePack spec. We only implement
	// the subset needed for NT4 value frames: fixarray(4), integers, doubles,
	// booleans, strings, and string arrays.
	static void MsgPackWriteArrayHeader(std::string& buf, uint32_t count);
	static void MsgPackWriteInt(std::string& buf, int64_t value);
	static void MsgPackWriteUInt(std::string& buf, uint64_t value);
	static void MsgPackWriteDouble(std::string& buf, double value);
	static void MsgPackWriteBool(std::string& buf, bool value);
	static void MsgPackWriteString(std::string& buf, const std::string& str);
	static void MsgPackWriteStringArray(std::string& buf, const std::vector<std::string>& arr);

	// Build a complete NT4 binary frame: [topicID, timestamp_us, typeCode, value]
	std::string BuildValueFrame(const TopicInfo& topic, const RetainedValue& value) const;
	static uint64_t GetTimestampUs();

	// Handle incoming JSON control messages from clients
	void HandleClientTextMessage(ix::WebSocket& ws, const std::string& text);
};
