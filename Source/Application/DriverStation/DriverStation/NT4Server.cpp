// Ian: NT4Server.cpp — NetworkTables 4 WebSocket server implementation.
// This is the heart of NT4 dashboard compatibility (Shuffleboard, Glass, etc.).
// See NT4Server.h for design overview.
//
// Protocol reference: https://github.com/wpilibsuite/allwpilib/blob/main/ntcore/doc/networktables4.adoc
//
// Key protocol details implemented here:
//   - Server listens on ws://0.0.0.0:5810
//   - Clients connect to /nt/<clientname>  (we accept any path starting with /nt/)
//   - Subprotocol negotiation: prefer v4.1.networktables.first.wpi.edu, fall back to
//     networktables.first.wpi.edu (v4.0)
//   - Control messages (JSON text frames): subscribe, publish, setproperties from client;
//     announce, unannounce, properties from server.  Each text frame is a JSON array.
//   - Value updates (MessagePack binary frames): each message is [topicID, timestamp_us, typeCode, value]
//   - Server assigns topic IDs via announce messages
//
// Ian: LESSON LEARNED — NT4 is subscription-driven!  The flow is:
//   1. Client connects
//   2. Client sends subscribe (e.g. [""] with prefix=true for "all topics")
//   3. ONLY THEN does server send announce messages for matching topics
//   4. Server sends cached values for those topics
//   5. Future value updates only go to clients whose subscriptions match
//
// Our first implementation sent announces immediately on connect, BEFORE the client
// subscribed.  ntcore silently ignored all of them, which is why dashboards showed
// nothing despite receiving all the data.

#include "stdafx.h"
#include "NT4Server.h"

#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXWebSocketServer.h>

// Ian: nlohmann/json is already a vcpkg dependency (installed for the project).
// We use it for NT4 control message JSON encoding/decoding.
#include <nlohmann/json.hpp>

#include <Windows.h>

#include <algorithm>
#include <chrono>
#include <cstring>

using json = nlohmann::json;

// ============================================================================
// Timestamp helper
// ============================================================================

uint64_t NT4Server::GetTimestampUs()
{
	// Ian: NT4 spec says timestamps are microseconds since an arbitrary epoch.
	// We use steady_clock for monotonicity (same as the rest of the transport layer).
	return static_cast<uint64_t>(
		std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count());
}

// ============================================================================
// NT4 type code → string name (for JSON announce messages)
// ============================================================================

const char* NT4Server::NT4TypeString(NT4Type type)
{
	switch (type)
	{
	case NT4Type::Boolean:    return "boolean";
	case NT4Type::Double:     return "double";
	case NT4Type::Int:        return "int";
	case NT4Type::Float:      return "float";
	case NT4Type::String:     return "string";
	case NT4Type::Raw:        return "raw";
	case NT4Type::BooleanArr: return "boolean[]";
	case NT4Type::DoubleArr:  return "double[]";
	case NT4Type::IntArr:     return "int[]";
	case NT4Type::FloatArr:   return "float[]";
	case NT4Type::StringArr:  return "string[]";
	default:                  return "raw";
	}
}

// ============================================================================
// MessagePack minimal encoder
// ============================================================================
// Ian: These implement just enough of the MessagePack spec to encode NT4 value
// frames.  The spec is at https://github.com/msgpack/msgpack/blob/master/spec.md
// We need: fixarray, positive/negative fixint, uint8/16/32/64, int8/16/32/64,
// float64, bool, fixstr/str8/str16/str32, and arrays of strings.

void NT4Server::MsgPackWriteArrayHeader(std::string& buf, uint32_t count)
{
	if (count <= 15)
	{
		// fixarray: 1001xxxx
		buf.push_back(static_cast<char>(0x90 | count));
	}
	else if (count <= 0xFFFF)
	{
		// array 16: 0xdc + 2 bytes big-endian
		buf.push_back(static_cast<char>(0xDC));
		buf.push_back(static_cast<char>((count >> 8) & 0xFF));
		buf.push_back(static_cast<char>(count & 0xFF));
	}
	else
	{
		// array 32: 0xdd + 4 bytes big-endian
		buf.push_back(static_cast<char>(0xDD));
		buf.push_back(static_cast<char>((count >> 24) & 0xFF));
		buf.push_back(static_cast<char>((count >> 16) & 0xFF));
		buf.push_back(static_cast<char>((count >> 8) & 0xFF));
		buf.push_back(static_cast<char>(count & 0xFF));
	}
}

void NT4Server::MsgPackWriteInt(std::string& buf, int64_t value)
{
	if (value >= 0)
	{
		MsgPackWriteUInt(buf, static_cast<uint64_t>(value));
		return;
	}
	// Negative integers
	if (value >= -32)
	{
		// negative fixint: 111xxxxx
		buf.push_back(static_cast<char>(static_cast<int8_t>(value)));
	}
	else if (value >= -128)
	{
		buf.push_back(static_cast<char>(0xD0)); // int 8
		buf.push_back(static_cast<char>(static_cast<int8_t>(value)));
	}
	else if (value >= -32768)
	{
		buf.push_back(static_cast<char>(0xD1)); // int 16
		int16_t v = static_cast<int16_t>(value);
		buf.push_back(static_cast<char>((v >> 8) & 0xFF));
		buf.push_back(static_cast<char>(v & 0xFF));
	}
	else if (value >= INT32_MIN)
	{
		buf.push_back(static_cast<char>(0xD2)); // int 32
		int32_t v = static_cast<int32_t>(value);
		buf.push_back(static_cast<char>((v >> 24) & 0xFF));
		buf.push_back(static_cast<char>((v >> 16) & 0xFF));
		buf.push_back(static_cast<char>((v >> 8) & 0xFF));
		buf.push_back(static_cast<char>(v & 0xFF));
	}
	else
	{
		buf.push_back(static_cast<char>(0xD3)); // int 64
		buf.push_back(static_cast<char>((value >> 56) & 0xFF));
		buf.push_back(static_cast<char>((value >> 48) & 0xFF));
		buf.push_back(static_cast<char>((value >> 40) & 0xFF));
		buf.push_back(static_cast<char>((value >> 32) & 0xFF));
		buf.push_back(static_cast<char>((value >> 24) & 0xFF));
		buf.push_back(static_cast<char>((value >> 16) & 0xFF));
		buf.push_back(static_cast<char>((value >> 8) & 0xFF));
		buf.push_back(static_cast<char>(value & 0xFF));
	}
}

void NT4Server::MsgPackWriteUInt(std::string& buf, uint64_t value)
{
	if (value <= 127)
	{
		// positive fixint: 0xxxxxxx
		buf.push_back(static_cast<char>(value));
	}
	else if (value <= 0xFF)
	{
		buf.push_back(static_cast<char>(0xCC)); // uint 8
		buf.push_back(static_cast<char>(value));
	}
	else if (value <= 0xFFFF)
	{
		buf.push_back(static_cast<char>(0xCD)); // uint 16
		buf.push_back(static_cast<char>((value >> 8) & 0xFF));
		buf.push_back(static_cast<char>(value & 0xFF));
	}
	else if (value <= 0xFFFFFFFF)
	{
		buf.push_back(static_cast<char>(0xCE)); // uint 32
		buf.push_back(static_cast<char>((value >> 24) & 0xFF));
		buf.push_back(static_cast<char>((value >> 16) & 0xFF));
		buf.push_back(static_cast<char>((value >> 8) & 0xFF));
		buf.push_back(static_cast<char>(value & 0xFF));
	}
	else
	{
		buf.push_back(static_cast<char>(0xCF)); // uint 64
		buf.push_back(static_cast<char>((value >> 56) & 0xFF));
		buf.push_back(static_cast<char>((value >> 48) & 0xFF));
		buf.push_back(static_cast<char>((value >> 40) & 0xFF));
		buf.push_back(static_cast<char>((value >> 32) & 0xFF));
		buf.push_back(static_cast<char>((value >> 24) & 0xFF));
		buf.push_back(static_cast<char>((value >> 16) & 0xFF));
		buf.push_back(static_cast<char>((value >> 8) & 0xFF));
		buf.push_back(static_cast<char>(value & 0xFF));
	}
}

void NT4Server::MsgPackWriteDouble(std::string& buf, double value)
{
	// float 64: 0xcb + 8 bytes IEEE 754 big-endian
	buf.push_back(static_cast<char>(0xCB));
	uint64_t bits = 0;
	static_assert(sizeof(double) == sizeof(uint64_t), "double must be 8 bytes");
	std::memcpy(&bits, &value, sizeof(bits));
	// Write big-endian
	buf.push_back(static_cast<char>((bits >> 56) & 0xFF));
	buf.push_back(static_cast<char>((bits >> 48) & 0xFF));
	buf.push_back(static_cast<char>((bits >> 40) & 0xFF));
	buf.push_back(static_cast<char>((bits >> 32) & 0xFF));
	buf.push_back(static_cast<char>((bits >> 24) & 0xFF));
	buf.push_back(static_cast<char>((bits >> 16) & 0xFF));
	buf.push_back(static_cast<char>((bits >> 8) & 0xFF));
	buf.push_back(static_cast<char>(bits & 0xFF));
}

void NT4Server::MsgPackWriteBool(std::string& buf, bool value)
{
	buf.push_back(value ? static_cast<char>(0xC3) : static_cast<char>(0xC2));
}

void NT4Server::MsgPackWriteString(std::string& buf, const std::string& str)
{
	const uint32_t len = static_cast<uint32_t>(str.size());
	if (len <= 31)
	{
		// fixstr: 101xxxxx
		buf.push_back(static_cast<char>(0xA0 | len));
	}
	else if (len <= 0xFF)
	{
		buf.push_back(static_cast<char>(0xD9)); // str 8
		buf.push_back(static_cast<char>(len));
	}
	else if (len <= 0xFFFF)
	{
		buf.push_back(static_cast<char>(0xDA)); // str 16
		buf.push_back(static_cast<char>((len >> 8) & 0xFF));
		buf.push_back(static_cast<char>(len & 0xFF));
	}
	else
	{
		buf.push_back(static_cast<char>(0xDB)); // str 32
		buf.push_back(static_cast<char>((len >> 24) & 0xFF));
		buf.push_back(static_cast<char>((len >> 16) & 0xFF));
		buf.push_back(static_cast<char>((len >> 8) & 0xFF));
		buf.push_back(static_cast<char>(len & 0xFF));
	}
	buf.append(str);
}

void NT4Server::MsgPackWriteStringArray(std::string& buf, const std::vector<std::string>& arr)
{
	MsgPackWriteArrayHeader(buf, static_cast<uint32_t>(arr.size()));
	for (const auto& s : arr)
		MsgPackWriteString(buf, s);
}

// ============================================================================
// Build a complete NT4 binary value frame
// ============================================================================
// Format: [topicID (int), timestamp_us (int), typeCode (int), value]
// Ian: Per the spec, each binary WebSocket frame can contain multiple concatenated
// MessagePack messages.  For simplicity in iteration 1, we send one message per frame.

std::string NT4Server::BuildValueFrame(const TopicInfo& topic, const RetainedValue& value) const
{
	std::string buf;
	buf.reserve(64);

	// 4-element array header
	MsgPackWriteArrayHeader(buf, 4);

	// [0] topic ID
	MsgPackWriteInt(buf, topic.id);

	// [1] timestamp in microseconds
	MsgPackWriteUInt(buf, GetTimestampUs());

	// [2] data type code
	MsgPackWriteInt(buf, static_cast<int64_t>(static_cast<uint8_t>(value.type)));

	// [3] value
	switch (value.type)
	{
	case NT4Type::Boolean:
		MsgPackWriteBool(buf, value.boolVal);
		break;
	case NT4Type::Double:
		MsgPackWriteDouble(buf, value.doubleVal);
		break;
	case NT4Type::Int:
		MsgPackWriteInt(buf, value.intVal);
		break;
	case NT4Type::String:
		MsgPackWriteString(buf, value.stringVal);
		break;
	case NT4Type::StringArr:
		MsgPackWriteStringArray(buf, value.stringArrayVal);
		break;
	default:
		// Ian: For unsupported types, send an empty string as a safe fallback.
		MsgPackWriteString(buf, "");
		break;
	}

	return buf;
}

// ============================================================================
// Build JSON announce message for a topic (single, not wrapped in array)
// ============================================================================
// Ian: Returns the JSON object for one announce message.
// The caller is responsible for wrapping in a JSON array.

std::string NT4Server::BuildAnnounceJson(const TopicInfo& topic) const
{
	json msg;
	msg["method"] = "announce";
	msg["params"]["name"] = topic.name;
	msg["params"]["id"] = topic.id;
	msg["params"]["type"] = NT4TypeString(topic.type);
	// Ian: pubuid is optional in the spec for server-initiated announces.
	// Omitting it is fine per spec: "If this message was sent in response to a
	// publish message, the Publisher UID. Otherwise absent."
	// However, some ntcore implementations may expect it. We omit for now.
	msg["params"]["properties"] = json::object();
	// Ian: Wrap in array — NT4 spec says each text frame is a JSON array of messages.
	json arr = json::array();
	arr.push_back(msg);
	return arr.dump();
}

// ============================================================================
// Topic management
// ============================================================================

NT4Server::TopicInfo& NT4Server::GetOrCreateTopic(const std::string& name, NT4Type type)
{
	// Caller must hold m_topicsMutex
	auto it = m_topics.find(name);
	if (it != m_topics.end())
		return it->second;

	TopicInfo info;
	info.id = m_nextTopicId++;
	info.name = name;
	info.type = type;
	m_topics[name] = info;
	m_topicIdToName[info.id] = name;
	return m_topics[name];
}

// ============================================================================
// Subscription matching
// ============================================================================
// Ian: Check if a topic name matches any of a client's subscriptions.
// Per NT4 spec, if prefix=true, a topic matches if it starts with the subscription
// string.  If prefix=false, it must be an exact match.  An empty string with
// prefix=true matches ALL non-meta topics (topics not starting with '$').

bool NT4Server::TopicMatchesClientSubscriptions(const std::string& topicName,
                                                 const ClientState& cs) const
{
	for (const auto& sub : cs.subscriptions)
	{
		for (const auto& pattern : sub.topics)
		{
			if (sub.prefix)
			{
				// Prefix match: topic name starts with pattern
				if (topicName.compare(0, pattern.size(), pattern) == 0)
					return true;
			}
			else
			{
				// Exact match
				if (topicName == pattern)
					return true;
			}
		}
	}
	return false;
}

// ============================================================================
// Send matching topics to a client after subscribe
// ============================================================================
// Ian: When a client sends subscribe, we need to send announce + cached value
// for every existing topic that matches the subscription AND hasn't been
// announced to this client yet.

void NT4Server::SendMatchingTopicsToClient(ix::WebSocket& ws, ClientState& cs)
{
	// Caller must hold m_topicsMutex

	// Build a batch of announces
	json announcements = json::array();
	std::vector<std::pair<TopicInfo, RetainedValue>> valuesToSend;

	for (const auto& kv : m_topics)
	{
		const TopicInfo& topic = kv.second;

		// Skip if already announced to this client
		if (cs.announcedTopics.count(topic.id))
			continue;

		// Check if any subscription matches
		if (!TopicMatchesClientSubscriptions(topic.name, cs))
			continue;

		// Build announce
		json msg;
		msg["method"] = "announce";
		msg["params"]["name"] = topic.name;
		msg["params"]["id"] = topic.id;
		msg["params"]["type"] = NT4TypeString(topic.type);
		msg["params"]["properties"] = json::object();
		announcements.push_back(msg);

		cs.announcedTopics.insert(topic.id);

		// Queue cached value if available
		auto retIt = m_retained.find(topic.name);
		if (retIt != m_retained.end())
			valuesToSend.push_back({topic, retIt->second});
	}

	// Send all announces in one text frame
	if (!announcements.empty())
	{
		const std::string jsonText = announcements.dump();
		ws.sendText(jsonText);
	}

	// Send cached values as binary frames
	for (const auto& entry : valuesToSend)
	{
		const std::string frame = BuildValueFrame(entry.first, entry.second);
		ws.sendBinary(frame);
	}
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

NT4Server::NT4Server() = default;

NT4Server::~NT4Server()
{
	Stop();
}

// ============================================================================
// Start / Stop
// ============================================================================

bool NT4Server::Start(int port)
{
	if (m_running)
		return true;

	// Ian: IXWebSocket requires ix::initNetSystem() on Windows (calls WSAStartup).
	// Safe to call multiple times.
	ix::initNetSystem();

	// Ian: Create the WebSocket server.
	// Port 5810 is the standard NT4 port. Bind to 0.0.0.0 to accept connections
	// from any interface (important for remote dashboards on another machine).
	// Backlog 5, max 8 connections, handshake timeout 3s.
	m_server = std::make_unique<ix::WebSocketServer>(
		port,
		"0.0.0.0",
		5,     // backlog
		8,     // maxConnections
		3,     // handshakeTimeoutSecs
		AF_INET,
		0      // pingIntervalSeconds — we handle our own pings per NT4 spec
	);

	// Ian: Disable per-message deflate — NT4 doesn't use it and it adds overhead
	// for the small messages we send.
	m_server->disablePerMessageDeflate();

	// Ian: Enable PONG responses so clients can use WebSocket PING for keepalive.
	// NT4 v4.1 spec says server should PING every 200ms, timeout 1s.
	// IXWebSocket handles PONG automatically when pong is enabled.
	m_server->enablePong();

	m_server->setOnClientMessageCallback(
		[this](std::shared_ptr<ix::ConnectionState> connectionState,
		       ix::WebSocket& ws,
		       const ix::WebSocketMessagePtr& msg)
		{
			switch (msg->type)
			{
			case ix::WebSocketMessageType::Open:
			{
				char dbg[512] = {};
				sprintf_s(dbg, "[NT4Server] Client connected: %s (protocol: %s, uri: %s)\n",
					connectionState->getRemoteIp().c_str(),
					msg->openInfo.protocol.c_str(),
					msg->openInfo.uri.c_str());
				OutputDebugStringA(dbg);
				printf("%s", dbg);

				// Ian: LESSON LEARNED — Do NOT send announces here!
				// The NT4 protocol is subscription-driven. We must wait for the client
				// to send a subscribe message before we announce anything. Sending
				// announces before subscribe causes ntcore to silently ignore them.
				{
					std::lock_guard<std::mutex> lock(m_topicsMutex);
					uintptr_t key = reinterpret_cast<uintptr_t>(&ws);
					m_clientState[key] = ClientState{};
				}
				break;
			}

			case ix::WebSocketMessageType::Close:
			{
				char dbg[256] = {};
				sprintf_s(dbg, "[NT4Server] Client disconnected: %s (code: %d)\n",
					connectionState->getRemoteIp().c_str(),
					msg->closeInfo.code);
				OutputDebugStringA(dbg);
				printf("%s", dbg);

				// Clean up client state
				{
					std::lock_guard<std::mutex> lock(m_topicsMutex);
					uintptr_t key = reinterpret_cast<uintptr_t>(&ws);
					m_clientState.erase(key);
				}
				break;
			}

			case ix::WebSocketMessageType::Message:
			{
				if (msg->binary)
				{
					// Ian: Binary frames from client are value updates or timestamp
					// sync (topic ID -1). Handle timestamp sync for v4.1 compliance.
					HandleClientBinaryMessage(ws, msg->str);
				}
				else
				{
					// Ian: Text frames are JSON control messages.
					HandleClientTextMessage(ws, msg->str);
				}
				break;
			}

			case ix::WebSocketMessageType::Error:
			{
				char dbg[512] = {};
				sprintf_s(dbg, "[NT4Server] WebSocket error: %s\n",
					msg->errorInfo.reason.c_str());
				OutputDebugStringA(dbg);
				break;
			}

			case ix::WebSocketMessageType::Ping:
			{
				// Ian: IXWebSocket auto-responds with PONG. Nothing to do.
				break;
			}

			case ix::WebSocketMessageType::Pong:
			{
				// Received a PONG — client is alive. Nothing to do.
				break;
			}

			default:
				break;
			}
		}
	);

	// Start listening
	auto listenResult = m_server->listen();
	if (!listenResult.first)
	{
		char dbg[512] = {};
		sprintf_s(dbg, "[NT4Server] Failed to listen on port %d: %s\n",
			port, listenResult.second.c_str());
		OutputDebugStringA(dbg);
		printf("%s", dbg);
		m_server.reset();
		return false;
	}

	m_server->start();
	m_running = true;

	char dbg[128] = {};
	sprintf_s(dbg, "[NT4Server] Listening on port %d\n", port);
	OutputDebugStringA(dbg);
	printf("%s", dbg);

	return true;
}

void NT4Server::Stop()
{
	if (!m_running || !m_server)
		return;

	OutputDebugStringA("[NT4Server] Stopping...\n");
	m_server->stop();
	m_server.reset();
	m_running = false;

	std::lock_guard<std::mutex> lock(m_topicsMutex);
	m_clientState.clear();

	OutputDebugStringA("[NT4Server] Stopped\n");
}

bool NT4Server::IsRunning() const
{
	return m_running;
}

// ============================================================================
// Publishing API
// ============================================================================
// Ian: Each Publish* method:
//   1. Creates the topic if it doesn't exist
//   2. Updates the retained value cache
//   3. Announces to any subscribed client that hasn't seen this topic yet
//   4. Sends the value update to all subscribed clients

void NT4Server::PublishDouble(const std::string& topicName, double value)
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	TopicInfo& topic = GetOrCreateTopic(topicName, NT4Type::Double);

	RetainedValue& rv = m_retained[topicName];
	rv.type = NT4Type::Double;
	rv.doubleVal = value;

	if (!m_running || !m_server)
		return;

	// Send to each subscribed client
	for (auto& client : m_server->getClients())
	{
		if (!client)
			continue;

		uintptr_t key = reinterpret_cast<uintptr_t>(client.get());
		auto csIt = m_clientState.find(key);
		if (csIt == m_clientState.end())
			continue;

		ClientState& cs = csIt->second;

		// Check subscription match
		if (!TopicMatchesClientSubscriptions(topicName, cs))
			continue;

		// Announce if not yet announced to this client
		if (!cs.announcedTopics.count(topic.id))
		{
			cs.announcedTopics.insert(topic.id);
			const std::string announceJson = BuildAnnounceJson(topic);
			client->sendText(announceJson);
		}

		// Send value
		const std::string frame = BuildValueFrame(topic, rv);
		client->sendBinary(frame);
	}
}

void NT4Server::PublishBoolean(const std::string& topicName, bool value)
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	TopicInfo& topic = GetOrCreateTopic(topicName, NT4Type::Boolean);

	RetainedValue& rv = m_retained[topicName];
	rv.type = NT4Type::Boolean;
	rv.boolVal = value;

	if (!m_running || !m_server)
		return;

	for (auto& client : m_server->getClients())
	{
		if (!client) continue;
		uintptr_t key = reinterpret_cast<uintptr_t>(client.get());
		auto csIt = m_clientState.find(key);
		if (csIt == m_clientState.end()) continue;
		ClientState& cs = csIt->second;
		if (!TopicMatchesClientSubscriptions(topicName, cs)) continue;

		if (!cs.announcedTopics.count(topic.id))
		{
			cs.announcedTopics.insert(topic.id);
			client->sendText(BuildAnnounceJson(topic));
		}
		client->sendBinary(BuildValueFrame(topic, rv));
	}
}

void NT4Server::PublishString(const std::string& topicName, const std::string& value)
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	TopicInfo& topic = GetOrCreateTopic(topicName, NT4Type::String);

	RetainedValue& rv = m_retained[topicName];
	rv.type = NT4Type::String;
	rv.stringVal = value;

	if (!m_running || !m_server)
		return;

	for (auto& client : m_server->getClients())
	{
		if (!client) continue;
		uintptr_t key = reinterpret_cast<uintptr_t>(client.get());
		auto csIt = m_clientState.find(key);
		if (csIt == m_clientState.end()) continue;
		ClientState& cs = csIt->second;
		if (!TopicMatchesClientSubscriptions(topicName, cs)) continue;

		if (!cs.announcedTopics.count(topic.id))
		{
			cs.announcedTopics.insert(topic.id);
			client->sendText(BuildAnnounceJson(topic));
		}
		client->sendBinary(BuildValueFrame(topic, rv));
	}
}

void NT4Server::PublishStringArray(const std::string& topicName, const std::vector<std::string>& values)
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	TopicInfo& topic = GetOrCreateTopic(topicName, NT4Type::StringArr);

	RetainedValue& rv = m_retained[topicName];
	rv.type = NT4Type::StringArr;
	rv.stringArrayVal = values;

	if (!m_running || !m_server)
		return;

	for (auto& client : m_server->getClients())
	{
		if (!client) continue;
		uintptr_t key = reinterpret_cast<uintptr_t>(client.get());
		auto csIt = m_clientState.find(key);
		if (csIt == m_clientState.end()) continue;
		ClientState& cs = csIt->second;
		if (!TopicMatchesClientSubscriptions(topicName, cs)) continue;

		if (!cs.announcedTopics.count(topic.id))
		{
			cs.announcedTopics.insert(topic.id);
			client->sendText(BuildAnnounceJson(topic));
		}
		client->sendBinary(BuildValueFrame(topic, rv));
	}
}

void NT4Server::PublishInt(const std::string& topicName, int64_t value)
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	TopicInfo& topic = GetOrCreateTopic(topicName, NT4Type::Int);

	RetainedValue& rv = m_retained[topicName];
	rv.type = NT4Type::Int;
	rv.intVal = value;

	if (!m_running || !m_server)
		return;

	for (auto& client : m_server->getClients())
	{
		if (!client) continue;
		uintptr_t key = reinterpret_cast<uintptr_t>(client.get());
		auto csIt = m_clientState.find(key);
		if (csIt == m_clientState.end()) continue;
		ClientState& cs = csIt->second;
		if (!TopicMatchesClientSubscriptions(topicName, cs)) continue;

		if (!cs.announcedTopics.count(topic.id))
		{
			cs.announcedTopics.insert(topic.id);
			client->sendText(BuildAnnounceJson(topic));
		}
		client->sendBinary(BuildValueFrame(topic, rv));
	}
}

// ============================================================================
// Minimal MessagePack reader (for decoding incoming client value frames)
// ============================================================================
// Ian: This is the read-side counterpart to the MsgPackWrite* encoder above.
// We need it to decode binary value frames from clients writing back chooser
// selections and other control values. Only implements the types we need:
// ints, doubles, bools, strings, and enough to skip unknown elements.

static uint16_t ReadBE16(const uint8_t* p)
{
	return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

static uint32_t ReadBE32(const uint8_t* p)
{
	return (static_cast<uint32_t>(p[0]) << 24)
	     | (static_cast<uint32_t>(p[1]) << 16)
	     | (static_cast<uint32_t>(p[2]) <<  8)
	     |  static_cast<uint32_t>(p[3]);
}

static uint64_t ReadBE64(const uint8_t* p)
{
	return (static_cast<uint64_t>(p[0]) << 56)
	     | (static_cast<uint64_t>(p[1]) << 48)
	     | (static_cast<uint64_t>(p[2]) << 40)
	     | (static_cast<uint64_t>(p[3]) << 32)
	     | (static_cast<uint64_t>(p[4]) << 24)
	     | (static_cast<uint64_t>(p[5]) << 16)
	     | (static_cast<uint64_t>(p[6]) <<  8)
	     |  static_cast<uint64_t>(p[7]);
}

bool NT4Server::MsgPackCursor::ReadArrayHeader(uint32_t& count)
{
	if (!HasRemaining(1)) return false;
	uint8_t b = data[pos++];
	if ((b & 0xF0) == 0x90) { count = b & 0x0F; return true; }
	if (b == 0xDC && HasRemaining(2)) { count = ReadBE16(data + pos); pos += 2; return true; }
	if (b == 0xDD && HasRemaining(4)) { count = ReadBE32(data + pos); pos += 4; return true; }
	return false;
}

bool NT4Server::MsgPackCursor::ReadInt(int64_t& out)
{
	if (!HasRemaining(1)) return false;
	uint8_t b = data[pos++];
	if (b <= 0x7F) { out = static_cast<int64_t>(b); return true; }
	if (b >= 0xE0) { out = static_cast<int64_t>(static_cast<int8_t>(b)); return true; }
	switch (b)
	{
		case 0xCC: if (!HasRemaining(1)) return false; out = data[pos++]; return true;
		case 0xCD: if (!HasRemaining(2)) return false; out = ReadBE16(data + pos); pos += 2; return true;
		case 0xCE: if (!HasRemaining(4)) return false; out = ReadBE32(data + pos); pos += 4; return true;
		case 0xCF: if (!HasRemaining(8)) return false; out = static_cast<int64_t>(ReadBE64(data + pos)); pos += 8; return true;
		case 0xD0: if (!HasRemaining(1)) return false; out = static_cast<int8_t>(data[pos++]); return true;
		case 0xD1: if (!HasRemaining(2)) return false; out = static_cast<int16_t>(ReadBE16(data + pos)); pos += 2; return true;
		case 0xD2: if (!HasRemaining(4)) return false; out = static_cast<int32_t>(ReadBE32(data + pos)); pos += 4; return true;
		case 0xD3: if (!HasRemaining(8)) return false; out = static_cast<int64_t>(ReadBE64(data + pos)); pos += 8; return true;
		default: return false;
	}
}

bool NT4Server::MsgPackCursor::ReadUInt(uint64_t& out)
{
	int64_t sv; if (!ReadInt(sv)) return false;
	out = static_cast<uint64_t>(sv); return true;
}

bool NT4Server::MsgPackCursor::ReadDouble(double& out)
{
	if (!HasRemaining(1)) return false;
	uint8_t b = data[pos];
	if (b == 0xCB)
	{
		pos++;
		if (!HasRemaining(8)) return false;
		uint64_t bits = ReadBE64(data + pos); pos += 8;
		std::memcpy(&out, &bits, sizeof(double));
		return true;
	}
	if (b == 0xCA)
	{
		pos++;
		if (!HasRemaining(4)) return false;
		uint32_t bits = ReadBE32(data + pos); pos += 4;
		float f; std::memcpy(&f, &bits, sizeof(float));
		out = static_cast<double>(f);
		return true;
	}
	int64_t iv; if (ReadInt(iv)) { out = static_cast<double>(iv); return true; }
	return false;
}

bool NT4Server::MsgPackCursor::ReadBool(bool& out)
{
	if (!HasRemaining(1)) return false;
	uint8_t b = data[pos++];
	if (b == 0xC3) { out = true; return true; }
	if (b == 0xC2) { out = false; return true; }
	return false;
}

bool NT4Server::MsgPackCursor::ReadString(std::string& out)
{
	if (!HasRemaining(1)) return false;
	uint8_t b = data[pos++];
	size_t len = 0;
	if ((b & 0xE0) == 0xA0) { len = b & 0x1F; }
	else if (b == 0xD9 && HasRemaining(1)) { len = data[pos++]; }
	else if (b == 0xDA && HasRemaining(2)) { len = ReadBE16(data + pos); pos += 2; }
	else if (b == 0xDB && HasRemaining(4)) { len = ReadBE32(data + pos); pos += 4; }
	else return false;
	if (!HasRemaining(len)) return false;
	out.assign(reinterpret_cast<const char*>(data + pos), len);
	pos += len;
	return true;
}

bool NT4Server::MsgPackCursor::SkipElement()
{
	if (!HasRemaining(1)) return false;
	uint8_t b = data[pos++];
	if (b <= 0x7F || b >= 0xE0) return true;
	if (b == 0xC0 || b == 0xC2 || b == 0xC3) return true;
	if ((b & 0xE0) == 0xA0) { size_t n = b & 0x1F; if (!HasRemaining(n)) return false; pos += n; return true; }
	if ((b & 0xF0) == 0x90) { uint32_t n = b & 0x0F; for (uint32_t i = 0; i < n; ++i) if (!SkipElement()) return false; return true; }
	if ((b & 0xF0) == 0x80) { uint32_t n = b & 0x0F; for (uint32_t i = 0; i < n * 2; ++i) if (!SkipElement()) return false; return true; }
	switch (b)
	{
		case 0xCC: case 0xD0: if (!HasRemaining(1)) return false; pos += 1; return true;
		case 0xCD: case 0xD1: if (!HasRemaining(2)) return false; pos += 2; return true;
		case 0xCE: case 0xD2: case 0xCA: if (!HasRemaining(4)) return false; pos += 4; return true;
		case 0xCF: case 0xD3: case 0xCB: if (!HasRemaining(8)) return false; pos += 8; return true;
		case 0xD9: { if (!HasRemaining(1)) return false; size_t n = data[pos++]; if (!HasRemaining(n)) return false; pos += n; return true; }
		case 0xDA: { if (!HasRemaining(2)) return false; size_t n = ReadBE16(data + pos); pos += 2; if (!HasRemaining(n)) return false; pos += n; return true; }
		case 0xDB: { if (!HasRemaining(4)) return false; size_t n = ReadBE32(data + pos); pos += 4; if (!HasRemaining(n)) return false; pos += n; return true; }
		case 0xDC: { if (!HasRemaining(2)) return false; uint32_t n = ReadBE16(data + pos); pos += 2; for (uint32_t i = 0; i < n; ++i) if (!SkipElement()) return false; return true; }
		case 0xDD: { if (!HasRemaining(4)) return false; uint32_t n = ReadBE32(data + pos); pos += 4; for (uint32_t i = 0; i < n; ++i) if (!SkipElement()) return false; return true; }
		default: return false;
	}
}

// ============================================================================
// Handle binary messages from clients
// ============================================================================
// Ian: In NT4, clients send binary frames for:
//   - Timestamp sync (topic ID = -1): client sends its local time, server
//     responds with server time + echoed client time
//   - Value updates for topics the client published: the first array element
//     is the client's pubuid (NOT the server topic ID). We resolve it via
//     the per-client pubuidToTopicId map stored during the "publish" handshake.

void NT4Server::HandleClientBinaryMessage(ix::WebSocket& ws, const std::string& data)
{
	if (data.size() < 3)
		return;

	const auto* rawData = reinterpret_cast<const uint8_t*>(data.data());
	MsgPackCursor cursor(rawData, data.size());

	// Ian: A single binary frame may contain multiple concatenated MsgPack arrays.
	while (cursor.pos < cursor.size)
	{
		uint32_t arrayLen = 0;
		if (!cursor.ReadArrayHeader(arrayLen) || arrayLen < 4)
			break;

		int64_t topicIdOrPubuid = 0;
		if (!cursor.ReadInt(topicIdOrPubuid))
			break;

		uint64_t timestamp = 0;
		if (!cursor.ReadUInt(timestamp))
			break;

		int64_t typeCodeRaw = 0;
		if (!cursor.ReadInt(typeCodeRaw))
			break;

		// Topic ID -1 = timestamp sync (RTT ping)
		if (topicIdOrPubuid == -1)
		{
			// Ian: LESSON LEARNED — ntcore's ClientImpl blocks ALL outgoing messages
			// (subscribe, publish, etc.) until it gets a valid RTT response.  The response
			// MUST have:
			//   - type code 2 (Integer, not Double)
			//   - value = the client's original timestamp echoed back
			// Without this, m_haveTimeOffset stays false and SendOutgoing() returns early,
			// which means the client NEVER sends subscribe messages.  This was the root
			// cause of Shuffleboard/Glass connecting but appearing silent.
			//
			// Client sends: [-1, 0, type=2(int), clientTimestamp]
			// Server must respond: [-1, serverTimestamp, type=2(int), clientTimestamp]
			int64_t clientTimestamp = 0;
			cursor.ReadInt(clientTimestamp);  // 4th element: client's timestamp
			for (uint32_t i = 4; i < arrayLen; ++i) cursor.SkipElement();

			std::string response;
			response.reserve(32);
			MsgPackWriteArrayHeader(response, 4);
			MsgPackWriteInt(response, -1);                      // topic ID -1 = timestamp sync
			MsgPackWriteUInt(response, GetTimestampUs());        // server timestamp (becomes value.server_time())
			MsgPackWriteInt(response, 2);                        // type code 2 = Integer (NOT 1/Double!)
			MsgPackWriteInt(response, clientTimestamp);           // echo client's original timestamp
			ws.sendBinary(response);
			continue;
		}

		// Ian: For real value frames, the client sends its pubuid (the ID it chose
		// in the "publish" JSON message) as the first element. We resolve this to
		// the server-assigned topic ID via the per-client mapping.
		int32_t serverTopicId = -1;
		{
			std::lock_guard<std::mutex> lock(m_topicsMutex);
			uintptr_t clientKey = reinterpret_cast<uintptr_t>(&ws);
			auto csIt = m_clientState.find(clientKey);
			if (csIt != m_clientState.end())
			{
				int32_t pubuid = static_cast<int32_t>(topicIdOrPubuid);
				auto pubIt = csIt->second.pubuidToTopicId.find(pubuid);
				if (pubIt != csIt->second.pubuidToTopicId.end())
					serverTopicId = pubIt->second;
			}
		}

		if (serverTopicId < 0)
		{
			// Unknown pubuid — skip remaining elements
			cursor.SkipElement();
			for (uint32_t i = 4; i < arrayLen; ++i) cursor.SkipElement();
			continue;
		}

		// Decode the value based on type code
		NT4Type ntType = static_cast<NT4Type>(static_cast<uint8_t>(typeCodeRaw));
		RetainedValue rv;
		rv.type = ntType;
		bool decoded = false;

		switch (ntType)
		{
			case NT4Type::Boolean:
			{
				bool bv = false;
				if (cursor.ReadBool(bv)) { rv.boolVal = bv; decoded = true; }
				break;
			}
			case NT4Type::Double:
			case NT4Type::Float:
			{
				double dv = 0.0;
				if (cursor.ReadDouble(dv)) { rv.type = NT4Type::Double; rv.doubleVal = dv; decoded = true; }
				break;
			}
			case NT4Type::Int:
			{
				int64_t iv = 0;
				if (cursor.ReadInt(iv)) { rv.intVal = iv; rv.doubleVal = static_cast<double>(iv); decoded = true; }
				break;
			}
			case NT4Type::String:
			{
				std::string sv;
				if (cursor.ReadString(sv)) { rv.stringVal = std::move(sv); decoded = true; }
				break;
			}
			default:
				cursor.SkipElement();
				break;
		}

		for (uint32_t i = 4; i < arrayLen; ++i) cursor.SkipElement();

		if (decoded)
		{
			HandleClientValueUpdate(ws, serverTopicId, rv);
		}
	}
}

// ============================================================================
// Handle a decoded client value update
// ============================================================================
// Ian: Update the retained cache and re-broadcast to ALL subscribed clients
// (including the sender). This is how dashboard write-back reaches the simulator:
//   1. Client sends binary value frame → decoded above
//   2. We update the retained cache (m_retained) with the new value
//   3. ALL subscribed clients see the update (including the sender — the server
//      is the single source of truth, and echoing back confirms acceptance)
//   4. The simulator reads the value via TryGet* queries on the retained cache

void NT4Server::HandleClientValueUpdate(ix::WebSocket& /*sender*/, int32_t topicId, const RetainedValue& value)
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);

	// Find the topic name from the ID
	auto nameIt = m_topicIdToName.find(topicId);
	if (nameIt == m_topicIdToName.end())
		return;

	const std::string& topicName = nameIt->second;
	auto topicIt = m_topics.find(topicName);
	if (topicIt == m_topics.end())
		return;

	const TopicInfo& topic = topicIt->second;

	// Update the retained cache
	m_retained[topicName] = value;

	// Re-broadcast to all other subscribed clients
	if (!m_running || !m_server)
		return;

	// Ian: We intentionally do NOT skip the sender. A client that publishes a
	// value should see it echoed back — the server is the single source of truth.
	// This lets a dashboard update its own UI tiles from the authoritative echo,
	// and also means multiple dashboards connected to the same server all converge
	// on the same retained state. (Matches the design principle: one robot, many
	// clients, every client sees every update including its own.)
	for (auto& client : m_server->getClients())
	{
		if (!client) continue;
		uintptr_t clientKey = reinterpret_cast<uintptr_t>(client.get());

		auto csIt = m_clientState.find(clientKey);
		if (csIt == m_clientState.end()) continue;
		ClientState& cs = csIt->second;

		if (!TopicMatchesClientSubscriptions(topicName, cs)) continue;

		// Announce if not yet announced to this client
		if (!cs.announcedTopics.count(topic.id))
		{
			cs.announcedTopics.insert(topic.id);
			client->sendText(BuildAnnounceJson(topic));
		}

		client->sendBinary(BuildValueFrame(topic, value));
	}
}

// ============================================================================
// Query API — let NT4Backend read retained values
// ============================================================================
// Ian: These are called by the simulator via SmartDashboardDirectQuerySource.
// They read from the retained value cache, which includes both server-published
// values AND client-written values (after HandleClientValueUpdate).

bool NT4Server::TryGetBoolean(const std::string& topicName, bool& value) const
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	auto it = m_retained.find(topicName);
	if (it == m_retained.end()) return false;
	if (it->second.type != NT4Type::Boolean) return false;
	value = it->second.boolVal;
	return true;
}

bool NT4Server::TryGetNumber(const std::string& topicName, double& value) const
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	auto it = m_retained.find(topicName);
	if (it == m_retained.end()) return false;
	if (it->second.type == NT4Type::Double || it->second.type == NT4Type::Float)
	{
		value = it->second.doubleVal;
		return true;
	}
	if (it->second.type == NT4Type::Int)
	{
		value = static_cast<double>(it->second.intVal);
		return true;
	}
	return false;
}

bool NT4Server::TryGetString(const std::string& topicName, std::string& value) const
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	auto it = m_retained.find(topicName);
	if (it == m_retained.end()) return false;
	if (it->second.type != NT4Type::String) return false;
	value = it->second.stringVal;
	return true;
}

// ============================================================================
// Handle JSON control messages from clients
// ============================================================================
// Ian: NT4 text frames are JSON arrays of message objects.
// Critical methods we handle:
//   - subscribe: Client wants to receive topic data — THIS is what triggers announces
//   - publish: Client wants to publish a topic to us
//   - unsubscribe / unpublish / setproperties: Acknowledged

void NT4Server::HandleClientTextMessage(ix::WebSocket& ws, const std::string& text)
{
	try
	{
		auto messages = json::parse(text);
		if (!messages.is_array())
			return;

		for (const auto& msg : messages)
		{
			if (!msg.contains("method") || !msg.contains("params"))
				continue;

			const std::string method = msg["method"].get<std::string>();

			if (method == "subscribe")
			{
				// Ian: THIS is the critical message. When a dashboard subscribes,
				// we record the subscription and THEN send matching announces + values.
				const auto& params = msg["params"];

				ClientSubscription sub;
				if (params.contains("subuid"))
					sub.subuid = params["subuid"].get<int32_t>();
				if (params.contains("topics") && params["topics"].is_array())
				{
					for (const auto& t : params["topics"])
						sub.topics.push_back(t.get<std::string>());
				}
				if (params.contains("options") && params["options"].is_object())
				{
					const auto& opts = params["options"];
					if (opts.contains("prefix"))
						sub.prefix = opts["prefix"].get<bool>();
				}

				// Store subscription and send matching topics
				{
					std::lock_guard<std::mutex> lock(m_topicsMutex);
					uintptr_t key = reinterpret_cast<uintptr_t>(&ws);
					auto csIt = m_clientState.find(key);
					if (csIt != m_clientState.end())
					{
						// Check if this subuid already exists (update)
						bool found = false;
						for (auto& existing : csIt->second.subscriptions)
						{
							if (existing.subuid == sub.subuid)
							{
								existing = sub; // Replace
								found = true;
								break;
							}
						}
						if (!found)
							csIt->second.subscriptions.push_back(sub);

						// Now send announces + cached values for all matching topics
						SendMatchingTopicsToClient(ws, csIt->second);
					}
					else
					{
						OutputDebugStringA("[NT4Server] WARNING: subscribe received but no ClientState found!\n");
						printf("[NT4Server] WARNING: subscribe received but no ClientState found!\n");
					}
				}
			}
			else if (method == "publish")
			{
				// Ian: Client wants to publish a topic to the server.
				// We create/find the topic, respond with an announce (echoing pubuid),
				// and store the pubuid → topic ID mapping so we can route incoming
				// binary value frames from this client to the correct topic.

				if (msg["params"].contains("name") && msg["params"].contains("type"))
				{
					const std::string name = msg["params"]["name"].get<std::string>();
					const std::string typeStr = msg["params"]["type"].get<std::string>();
					int32_t pubuid = 0;
					if (msg["params"].contains("pubuid"))
						pubuid = msg["params"]["pubuid"].get<int32_t>();

					// Determine NT4Type from string
					NT4Type type = NT4Type::Raw;
					if (typeStr == "boolean") type = NT4Type::Boolean;
					else if (typeStr == "double") type = NT4Type::Double;
					else if (typeStr == "int") type = NT4Type::Int;
					else if (typeStr == "float") type = NT4Type::Float;
					else if (typeStr == "string") type = NT4Type::String;
					else if (typeStr == "boolean[]") type = NT4Type::BooleanArr;
					else if (typeStr == "double[]") type = NT4Type::DoubleArr;
					else if (typeStr == "int[]") type = NT4Type::IntArr;
					else if (typeStr == "float[]") type = NT4Type::FloatArr;
					else if (typeStr == "string[]") type = NT4Type::StringArr;

					std::lock_guard<std::mutex> lock(m_topicsMutex);
					TopicInfo& topic = GetOrCreateTopic(name, type);

					// Ian: Store the pubuid → topic ID mapping for this client.
					// When binary value frames arrive, the client sends the pubuid
					// (NOT the server topic ID) as the first array element. We need
					// this map to resolve it back to the topic.
					uintptr_t clientKey = reinterpret_cast<uintptr_t>(&ws);
					auto csIt = m_clientState.find(clientKey);
					if (csIt != m_clientState.end())
					{
						csIt->second.pubuidToTopicId[pubuid] = topic.id;
					}

					// Respond with announce (include pubuid since this is in response to publish)
					json announceMsg;
					announceMsg["method"] = "announce";
					announceMsg["params"]["name"] = topic.name;
					announceMsg["params"]["id"] = topic.id;
					announceMsg["params"]["type"] = NT4TypeString(topic.type);
					announceMsg["params"]["pubuid"] = pubuid;
					announceMsg["params"]["properties"] = json::object();

					json arr = json::array();
					arr.push_back(announceMsg);
					ws.sendText(arr.dump());
				}
			}
			else if (method == "setproperties")
			{
				// Acknowledged — no action needed in iteration 1
			}
			else if (method == "unsubscribe")
			{
				// Remove the subscription
				if (msg["params"].contains("subuid"))
				{
					int32_t subuid = msg["params"]["subuid"].get<int32_t>();
					std::lock_guard<std::mutex> lock(m_topicsMutex);
					uintptr_t key = reinterpret_cast<uintptr_t>(&ws);
					auto csIt = m_clientState.find(key);
					if (csIt != m_clientState.end())
					{
						auto& subs = csIt->second.subscriptions;
						subs.erase(
							std::remove_if(subs.begin(), subs.end(),
								[subuid](const ClientSubscription& s) { return s.subuid == subuid; }),
							subs.end());
					}
				}
			}
			else if (method == "unpublish")
			{
				// Acknowledged — no action needed in iteration 1
			}
			else
			{
				// Unknown method — ignore
			}
		}
	}
	catch (const json::exception& e)
	{
		char dbg[512] = {};
		sprintf_s(dbg, "[NT4Server] JSON parse error: %s\n", e.what());
		OutputDebugStringA(dbg);
	}
}
