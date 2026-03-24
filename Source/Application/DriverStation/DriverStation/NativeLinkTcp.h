#pragma once

#include "NativeLink.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace NativeLink::detail
{
	struct TcpFrameHeader
	{
		std::uint32_t magic = 0x4E4C5443;
		std::uint16_t version = 1;
		std::uint16_t kind = 0;
		std::uint32_t payloadBytes = 0;
	};

	enum class TcpFrameKind : std::uint16_t
	{
		ClientHello = 1,
		ServerHello = 2,
		ServerMessage = 3,
		ClientPublish = 4,
		Heartbeat = 5
	};

	struct TcpHelloPayload
	{
		char channelId[64] = {};
		char clientId[64] = {};
	};

	struct TcpServerHelloPayload
	{
		std::uint64_t serverSessionId = 0;
	};

	struct TcpHeartbeatPayload
	{
		std::uint64_t serverSessionId = 0;
	};

	constexpr std::uint32_t kTcpFrameMagic = 0x4E4C5443;
	constexpr std::uint16_t kTcpFrameVersion = 1;

	class ITcpServerCarrier
	{
	public:
		virtual ~ITcpServerCarrier() = default;
		virtual bool Start() = 0;
		virtual void Stop() = 0;
		virtual void PublishSnapshotForClient(const std::string& clientId, const std::vector<SnapshotEvent>& snapshot) = 0;
		virtual void PublishEnvelopeToAllClients(const UpdateEnvelope& envelope) = 0;
		virtual void PublishEnvelopeToSingleClient(const std::string& clientId, const UpdateEnvelope& envelope) = 0;
		virtual void DrainClientWrites(std::vector<std::pair<std::string, std::pair<std::string, TopicValue>>>& writes) = 0;
	};

	class ITcpClientCarrier
	{
	public:
		virtual ~ITcpClientCarrier() = default;
		virtual bool Start() = 0;
		virtual void Stop() = 0;
		virtual std::vector<UpdateEnvelope> DrainMessages(std::uint32_t timeoutMs) = 0;
		virtual bool Publish(const std::string& keyName, const TopicValue& value) = 0;
	};

	std::unique_ptr<ITcpServerCarrier> CreateTcpServerCarrier(const ServerConfig& config, Core& core);
	std::unique_ptr<ITcpClientCarrier> CreateTcpClientCarrier(const TestClientConfig& config);
}
