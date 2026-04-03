#pragma once
// Ian: NT4Server is the NetworkTables 4.x WebSocket protocol server for dashboard
// compatibility (Shuffleboard, Glass, and any NT4-speaking client).
// It runs an IXWebSocket server on port 5810 and speaks the NT4 binary
// (MessagePack) + JSON control protocol that official WPILib dashboards expect.
//
// Design notes:
//   - This is *server-side only* — the simulator publishes topics and dashboards
//     subscribe to them.  We do NOT need to subscribe to remote topics yet (iteration 1).
//   - The MessagePack encoder is custom / minimal (~100 lines) because we only need
//     to *encode* a small subset of types: bool, double, int, float, string, string[].
//   - IXWebSocket handles WebSocket framing, PING/PONG, and connection lifecycle.
//   - NT4 subprotocol negotiation is handled in the Open callback by checking the
//     Sec-WebSocket-Protocol header from the client request.
//
// Ian: LESSON LEARNED — NT4 is subscription-driven!  The server must NOT send announce
// or value messages until the client sends a subscribe message.  Our first attempt
// sent announces on connect, which ntcore silently ignored.  The correct flow is:
//   1. Client connects (Open)
//   2. Client sends subscribe with topic patterns
//   3. Server sends announce for matching topics
//   4. Server sends cached values for matching topics
//   5. Future publishes only go to clients with matching subscriptions
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
#include <set>
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

	// --- Query API (for SmartDashboardDirectQuerySource) ---
	// Ian: These let the NT4Backend read back values that clients wrote.
	// They check the retained value cache under m_topicsMutex.
	bool TryGetBoolean(const std::string& topicName, bool& value) const;
	bool TryGetNumber(const std::string& topicName, double& value) const;
	bool TryGetString(const std::string& topicName, std::string& value) const;

	// --- Publishing API (server → subscribed clients) ---
	// These queue values and broadcast as NT4 binary frames to clients
	// whose subscriptions match the topic name.
	// Topic names should use NT4 convention: "/SmartDashboard/<key>"
	void PublishDouble(const std::string& topicName, double value);
	void PublishBoolean(const std::string& topicName, bool value);
	void PublishString(const std::string& topicName, const std::string& value);
	void PublishStringArray(const std::string& topicName, const std::vector<std::string>& values);
	void PublishInt(const std::string& topicName, int64_t value);

	// --- Topic properties API ---
	// Ian: Set the properties JSON for a topic's announce message.  WPILib dashboards
	// use this to identify Sendable types (Scheduler, Subsystem, Command, etc.).
	// The propertiesJson string must be valid JSON, e.g. "{\"SmartDashboard\":\"Scheduler\"}".
	// If the topic doesn't exist yet, it is created with the given type.
	// If the topic already exists, its properties are updated (type is ignored).

	// Ian: Type hints for SetTopicProperties — lets callers specify the NT4 type without
	// depending on the private NT4Type enum.  Maps 1:1 to NT4Type values.
	enum class NT4TypeHint : uint8_t
	{
		Boolean = 0,
		Double  = 1,
		String  = 4,
		StringArray = 20
	};

	void SetTopicProperties(const std::string& topicName, NT4TypeHint typeHint, const std::string& propertiesJson);

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
		// Ian: Topic properties are emitted in the announce JSON's "properties" field.
		// WPILib dashboards use this to identify Sendable types:
		//   {"SmartDashboard":"Scheduler"}, {"SmartDashboard":"Subsystem"}, etc.
		// Stored as pre-serialized JSON string so NT4Server.h doesn't need nlohmann.
		// Default is empty string → announce emits {} (backward-compatible).
		std::string propertiesJson;
	};

	// Ian: Retained value cache — we keep the latest value for each topic so that
	// newly subscribing clients get a full snapshot (NT4 dashboards expect this).
	struct RetainedValue
	{
		NT4Type type = NT4Type::Double;
		bool boolVal = false;
		double doubleVal = 0.0;
		int64_t intVal = 0;
		std::string stringVal;
		std::vector<std::string> stringArrayVal;
	};

	// Ian: Per-client subscription tracking.
	// LESSON LEARNED: NT4 is subscription-driven.  The server must track what each
	// client has subscribed to and only send announces/values for matching topics.
	// Without this, ntcore silently ignores all our data.
	struct ClientSubscription
	{
		int32_t subuid = -1;                  // client-generated subscription UID
		std::vector<std::string> topics;      // topic names or prefixes
		bool prefix = false;                  // if true, topics[] are prefixes
	};

	struct ClientState
	{
		std::vector<ClientSubscription> subscriptions;
		std::set<int32_t> announcedTopics;    // topic IDs already announced to this client
		// Ian: Per-client publish tracking. When a client sends a "publish" JSON message,
		// we map their chosen pubuid → server-assigned topic ID. Binary value frames from
		// the client use the pubuid, NOT the server topic ID, so we need this reverse map
		// to route incoming values to the correct topic and retained cache entry.
		std::unordered_map<int32_t, int32_t> pubuidToTopicId;  // client pubuid → server topic ID
	};

	// Core server
	std::unique_ptr<ix::WebSocketServer> m_server;
	bool m_running = false;

	// Topic management (protected by m_topicsMutex)
	mutable std::mutex m_topicsMutex;
	std::unordered_map<std::string, TopicInfo> m_topics;          // name → info
	std::unordered_map<int32_t, std::string> m_topicIdToName;    // id → name (reverse lookup)
	int32_t m_nextTopicId = 0;

	// Retained values (protected by m_topicsMutex — same lock)
	std::unordered_map<std::string, RetainedValue> m_retained;

	// Ian: Per-client state, keyed by WebSocket pointer address.
	// Protected by m_topicsMutex (same lock — simpler than a second mutex).
	std::unordered_map<uintptr_t, ClientState> m_clientState;

	// --- Internal helpers ---
	TopicInfo& GetOrCreateTopic(const std::string& name, NT4Type type);
	std::string BuildAnnounceJson(const TopicInfo& topic) const;

	// Ian: Check if a topic name matches any subscription for a given client.
	bool TopicMatchesClientSubscriptions(const std::string& topicName, const ClientState& cs) const;

	// Ian: Send announce + cached value for all matching topics to a client that
	// just subscribed.  Only sends topics not yet announced to this client.
	void SendMatchingTopicsToClient(ix::WebSocket& ws, ClientState& cs);

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

	// Handle incoming binary messages from clients (timestamp sync, value updates)
	void HandleClientBinaryMessage(ix::WebSocket& ws, const std::string& data);

	// Ian: Minimal MessagePack reader — just enough to decode incoming NT4 value frames
	// from clients. Mirrors the writer side: reads ints, doubles, bools, strings.
	struct MsgPackCursor
	{
		const uint8_t* data;
		size_t size;
		size_t pos;
		MsgPackCursor(const uint8_t* d, size_t s) : data(d), size(s), pos(0) {}
		bool HasRemaining(size_t n) const { return pos + n <= size; }
		bool ReadArrayHeader(uint32_t& count);
		bool ReadInt(int64_t& out);
		bool ReadUInt(uint64_t& out);
		bool ReadDouble(double& out);
		bool ReadBool(bool& out);
		bool ReadString(std::string& out);
		bool SkipElement();
	};

	// Ian: Process a decoded client value update: update retained cache and re-broadcast
	// to other subscribed clients (excluding the sender).
	void HandleClientValueUpdate(ix::WebSocket& sender, int32_t topicId, const RetainedValue& value);
};
