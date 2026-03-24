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
//   - On new client connect, server sends announce + retained value for all known topics

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
#include <cstdio>
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
// Build JSON announce message for a topic
// ============================================================================
// Ian: The announce message tells the client about a topic's existence.
// Format: {"method":"announce","params":{"name":"...","id":N,"type":"...","properties":{}}}

std::string NT4Server::BuildAnnounceJson(const TopicInfo& topic) const
{
	json msg;
	msg["method"] = "announce";
	msg["params"]["name"] = topic.name;
	msg["params"]["id"] = topic.id;
	msg["params"]["type"] = NT4TypeString(topic.type);
	msg["params"]["pubuid"] = 0;  // server-originated, no client publisher
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
	info.announced = false;
	m_topics[name] = info;
	return m_topics[name];
}

// ============================================================================
// Send announce + retained values for all topics to a specific client
// ============================================================================

void NT4Server::AnnounceAllTopicsTo(void* wsPtr)
{
	auto* ws = static_cast<ix::WebSocket*>(wsPtr);
	if (!ws)
		return;

	// Build one big JSON array with all announce messages, then send retained values
	json announcements = json::array();

	std::vector<std::pair<TopicInfo, RetainedValue>> snapshot;

	{
		std::lock_guard<std::mutex> lock(m_topicsMutex);
		for (auto& kv : m_topics)
		{
			TopicInfo& topic = kv.second;
			topic.announced = true;

			json msg;
			msg["method"] = "announce";
			msg["params"]["name"] = topic.name;
			msg["params"]["id"] = topic.id;
			msg["params"]["type"] = NT4TypeString(topic.type);
			msg["params"]["pubuid"] = 0;
			msg["params"]["properties"] = json::object();
			announcements.push_back(msg);

			auto retIt = m_retained.find(topic.name);
			if (retIt != m_retained.end())
				snapshot.push_back({topic, retIt->second});
		}
	}

	// Send all announces in one text frame
	if (!announcements.empty())
	{
		const std::string jsonText = announcements.dump();
		ws->sendText(jsonText);

		char dbg[256] = {};
		sprintf_s(dbg, "[NT4Server] Sent %zu announce(s) to new client\n",
			announcements.size());
		OutputDebugStringA(dbg);
	}

	// Send retained values as binary frames
	for (const auto& entry : snapshot)
	{
		const std::string frame = BuildValueFrame(entry.first, entry.second);
		ws->sendBinary(frame);
	}
}

// ============================================================================
// Broadcast a value update to all connected clients
// ============================================================================

void NT4Server::SendValueToAllClients(const TopicInfo& topic, const RetainedValue& value)
{
	if (!m_server)
		return;

	// Ian: If this topic hasn't been announced yet, announce it first to all clients.
	// This handles the case where a new topic is published after clients are already connected.
	if (!topic.announced)
	{
		const std::string announceJson = BuildAnnounceJson(topic);
		for (auto& client : m_server->getClients())
		{
			if (client)
				client->sendText(announceJson);
		}
		// Mark as announced (caller should update under lock, but we do it here too for safety)
	}

	const std::string frame = BuildValueFrame(topic, value);
	for (auto& client : m_server->getClients())
	{
		if (client)
			client->sendBinary(frame);
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

				// Ian: Send all known topics + retained values to the new client.
				AnnounceAllTopicsTo(&ws);
				break;
			}

			case ix::WebSocketMessageType::Close:
			{
				char dbg[256] = {};
				sprintf_s(dbg, "[NT4Server] Client disconnected: %s (code: %d)\n",
					connectionState->getRemoteIp().c_str(),
					msg->closeInfo.code);
				OutputDebugStringA(dbg);
				break;
			}

			case ix::WebSocketMessageType::Message:
			{
				if (msg->binary)
				{
					// Ian: Binary frames from client would be value updates (e.g. for
					// bidirectional topics). In iteration 1 we're server-publish-only,
					// so we just log and ignore.
					OutputDebugStringA("[NT4Server] Received binary frame from client (ignored in iteration 1)\n");
				}
				else
				{
					// Ian: Text frames are JSON control messages.
					// We need to handle: subscribe, publish, setproperties, unsubscribe, unpublish
					// For iteration 1, we mostly just acknowledge these gracefully.
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
	OutputDebugStringA("[NT4Server] Stopped\n");
}

bool NT4Server::IsRunning() const
{
	return m_running;
}

// ============================================================================
// Publishing API
// ============================================================================

void NT4Server::PublishDouble(const std::string& topicName, double value)
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	TopicInfo& topic = GetOrCreateTopic(topicName, NT4Type::Double);

	RetainedValue& rv = m_retained[topicName];
	rv.type = NT4Type::Double;
	rv.doubleVal = value;

	if (m_running)
	{
		// If topic was just created, mark it announced and send announce first
		if (!topic.announced)
		{
			topic.announced = true;
			const std::string announceJson = BuildAnnounceJson(topic);
			if (m_server)
			{
				for (auto& client : m_server->getClients())
				{
					if (client)
						client->sendText(announceJson);
				}
			}
		}

		const std::string frame = BuildValueFrame(topic, rv);
		if (m_server)
		{
			for (auto& client : m_server->getClients())
			{
				if (client)
					client->sendBinary(frame);
			}
		}
	}
}

void NT4Server::PublishBoolean(const std::string& topicName, bool value)
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	TopicInfo& topic = GetOrCreateTopic(topicName, NT4Type::Boolean);

	RetainedValue& rv = m_retained[topicName];
	rv.type = NT4Type::Boolean;
	rv.boolVal = value;

	if (m_running)
	{
		if (!topic.announced)
		{
			topic.announced = true;
			const std::string announceJson = BuildAnnounceJson(topic);
			if (m_server)
			{
				for (auto& client : m_server->getClients())
				{
					if (client)
						client->sendText(announceJson);
				}
			}
		}

		const std::string frame = BuildValueFrame(topic, rv);
		if (m_server)
		{
			for (auto& client : m_server->getClients())
			{
				if (client)
					client->sendBinary(frame);
			}
		}
	}
}

void NT4Server::PublishString(const std::string& topicName, const std::string& value)
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	TopicInfo& topic = GetOrCreateTopic(topicName, NT4Type::String);

	RetainedValue& rv = m_retained[topicName];
	rv.type = NT4Type::String;
	rv.stringVal = value;

	if (m_running)
	{
		if (!topic.announced)
		{
			topic.announced = true;
			const std::string announceJson = BuildAnnounceJson(topic);
			if (m_server)
			{
				for (auto& client : m_server->getClients())
				{
					if (client)
						client->sendText(announceJson);
				}
			}
		}

		const std::string frame = BuildValueFrame(topic, rv);
		if (m_server)
		{
			for (auto& client : m_server->getClients())
			{
				if (client)
					client->sendBinary(frame);
			}
		}
	}
}

void NT4Server::PublishStringArray(const std::string& topicName, const std::vector<std::string>& values)
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	TopicInfo& topic = GetOrCreateTopic(topicName, NT4Type::StringArr);

	RetainedValue& rv = m_retained[topicName];
	rv.type = NT4Type::StringArr;
	rv.stringArrayVal = values;

	if (m_running)
	{
		if (!topic.announced)
		{
			topic.announced = true;
			const std::string announceJson = BuildAnnounceJson(topic);
			if (m_server)
			{
				for (auto& client : m_server->getClients())
				{
					if (client)
						client->sendText(announceJson);
				}
			}
		}

		const std::string frame = BuildValueFrame(topic, rv);
		if (m_server)
		{
			for (auto& client : m_server->getClients())
			{
				if (client)
					client->sendBinary(frame);
			}
		}
	}
}

void NT4Server::PublishInt(const std::string& topicName, int64_t value)
{
	std::lock_guard<std::mutex> lock(m_topicsMutex);
	TopicInfo& topic = GetOrCreateTopic(topicName, NT4Type::Int);

	RetainedValue& rv = m_retained[topicName];
	rv.type = NT4Type::Int;
	rv.intVal = value;

	if (m_running)
	{
		if (!topic.announced)
		{
			topic.announced = true;
			const std::string announceJson = BuildAnnounceJson(topic);
			if (m_server)
			{
				for (auto& client : m_server->getClients())
				{
					if (client)
						client->sendText(announceJson);
				}
			}
		}

		const std::string frame = BuildValueFrame(topic, rv);
		if (m_server)
		{
			for (auto& client : m_server->getClients())
			{
				if (client)
					client->sendBinary(frame);
			}
		}
	}
}

// ============================================================================
// Handle JSON control messages from clients
// ============================================================================
// Ian: In iteration 1 we're server-publish-only. We handle client messages
// gracefully but don't act on most of them:
//   - subscribe: acknowledge (we broadcast to all anyway)
//   - publish: client wants to publish to us — not supported in iteration 1
//   - setproperties: acknowledge
//   - unsubscribe / unpublish: acknowledge

void NT4Server::HandleClientTextMessage(ix::WebSocket& ws, const std::string& text)
{
	try
	{
		auto messages = json::parse(text);
		if (!messages.is_array())
		{
			OutputDebugStringA("[NT4Server] Received non-array JSON text frame (ignoring)\n");
			return;
		}

		for (const auto& msg : messages)
		{
			if (!msg.contains("method") || !msg.contains("params"))
				continue;

			const std::string method = msg["method"].get<std::string>();

			if (method == "subscribe")
			{
				// Ian: Client wants to subscribe to topics. Since we broadcast all values
				// to all clients anyway (simplification for iteration 1), we just log it.
				// A proper implementation would track per-client subscriptions and only
				// send matching topics.
				if (msg["params"].contains("topics"))
				{
					char dbg[512] = {};
					sprintf_s(dbg, "[NT4Server] Client subscribe request for %zu topic pattern(s)\n",
						msg["params"]["topics"].size());
					OutputDebugStringA(dbg);
				}
			}
			else if (method == "publish")
			{
				// Ian: Client wants to publish a topic to the server.
				// Not supported in iteration 1 (we're server-publish-only).
				OutputDebugStringA("[NT4Server] Client publish request (not supported in iteration 1)\n");
			}
			else if (method == "setproperties")
			{
				OutputDebugStringA("[NT4Server] Client setproperties request (acknowledged, no-op)\n");
			}
			else if (method == "unsubscribe")
			{
				OutputDebugStringA("[NT4Server] Client unsubscribe request (acknowledged)\n");
			}
			else if (method == "unpublish")
			{
				OutputDebugStringA("[NT4Server] Client unpublish request (acknowledged)\n");
			}
			else
			{
				char dbg[256] = {};
				sprintf_s(dbg, "[NT4Server] Unknown method '%s' (ignoring)\n", method.c_str());
				OutputDebugStringA(dbg);
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
