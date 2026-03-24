// Ian: NT4Server.cpp — NetworkTables 4 WebSocket server implementation.
// This is the heart of Shuffleboard compatibility.  See NT4Server.h for design overview.
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
// subscribed.  ntcore silently ignored all of them, which is why Shuffleboard showed
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
	// from any interface (important for remote Shuffleboard on another machine).
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
// Handle binary messages from clients
// ============================================================================
// Ian: In NT4 v4.1, clients send binary frames for:
//   - Timestamp sync (topic ID = -1): client sends its local time, server
//     responds with server time + echoed client time
//   - Value updates for topics the client published (not used in iteration 1)

void NT4Server::HandleClientBinaryMessage(ix::WebSocket& ws, const std::string& data)
{
	// Ian: Minimal MessagePack decoder — just enough to read the topic ID
	// from a 4-element array to detect timestamp sync requests.
	if (data.size() < 3)
		return;

	size_t pos = 0;

	// Read array header
	uint8_t first = static_cast<uint8_t>(data[pos++]);
	uint32_t arrayLen = 0;
	if ((first & 0xF0) == 0x90)
		arrayLen = first & 0x0F;
	else if (first == 0xDC && pos + 2 <= data.size())
	{
		arrayLen = (static_cast<uint8_t>(data[pos]) << 8) | static_cast<uint8_t>(data[pos + 1]);
		pos += 2;
	}
	else
		return;

	if (arrayLen != 4)
		return;

	// Read topic ID (first element) — could be negative fixint for -1
	uint8_t idByte = static_cast<uint8_t>(data[pos]);
	int32_t topicId = 0;

	if (idByte <= 127)
	{
		// positive fixint
		topicId = idByte;
		pos++;
	}
	else if (idByte >= 0xE0)
	{
		// negative fixint: -1 to -32
		topicId = static_cast<int8_t>(idByte);
		pos++;
	}
	else if (idByte == 0xD0 && pos + 2 <= data.size())
	{
		// int 8
		topicId = static_cast<int8_t>(data[pos + 1]);
		pos += 2;
	}
	else
	{
		// Other int formats — for timestamp sync we only care about -1
		return;
	}

	// Ian: Topic ID -1 = timestamp sync request.
	// Per NT4 spec: client sends [−1, 0, type, clientTime].
	// Server must respond with [−1, serverTime, type, clientTime].
	// We need to read the type code and client time value, then echo back.
	if (topicId == -1)
	{
		// Read timestamp (element [1]) — skip it (should be 0)
		// Read type code (element [2]) — need this
		// Read client time (element [3]) — need this to echo back
		// For simplicity, just echo the entire data payload with our timestamp inserted.
		// The safest approach: rebuild the response from scratch.

		// We need to parse out the client's time value.  For a robust implementation
		// we'd need a full msgpack decoder.  For now, use a simplified approach:
		// Respond with [−1, serverTimestamp, 1 (int type code), 0] which at least
		// tells the client we're alive.  This is imperfect but enough for iteration 1.
		// TODO: Properly echo client time for accurate RTT measurement.

		std::string response;
		response.reserve(32);
		MsgPackWriteArrayHeader(response, 4);
		MsgPackWriteInt(response, -1);              // topic ID
		MsgPackWriteUInt(response, GetTimestampUs()); // server timestamp
		MsgPackWriteInt(response, 1);               // type code (double)
		// Ian: We should echo the client's time value, but we'd need to decode it.
		// For now send 0 — this means the client's RTT estimate will be wrong,
		// but it won't break functionality.  Proper implementation in iteration 2.
		MsgPackWriteInt(response, 0);

		ws.sendBinary(response);
	}
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
				// Ian: THIS is the critical message. When Shuffleboard subscribes,
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
				}
			}
			else if (method == "publish")
			{
				// Ian: Client wants to publish a topic to the server.
				// In NT4, the server must respond with an announce message.
				// Not fully supported in iteration 1, but we should at least
				// respond with an announce to keep the client happy.

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
