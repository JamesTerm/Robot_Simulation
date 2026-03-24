#pragma once

#include "Transport.h"
#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

	namespace NativeLink
	{
		enum class CarrierKind
		{
			SharedMemory,
			Tcp
		};

		const char* ToString(CarrierKind kind);
		bool TryParseCarrierKind(const std::string& text, CarrierKind& outKind);

		enum class TopicKind
		{
		State,
		Command,
		Event
	};

	enum class ValueType
	{
		Bool,
		Double,
		String,
		StringArray
	};

	enum class RetentionMode
	{
		None,
		LatestValue
	};

	enum class WriterPolicy
	{
		ServerOnly,
		LeaseSingleWriter
	};

	enum class DeliveryKind
	{
		SnapshotState,
		LiveState,
		LiveCommand,
		LiveCommandAck,
		LiveCommandReject,
		LiveEvent
	};

	enum class FreshnessReason
	{
		Live,
		TtlExpired
	};

	enum class SnapshotEventKind
	{
		DescriptorSnapshotBegin,
		Descriptor,
		DescriptorSnapshotEnd,
		StateSnapshotBegin,
		Update,
		StateSnapshotEnd,
		LiveBegin
	};

	enum class WriteRejectReason
	{
		None,
		UnknownTopic,
		WrongType,
		ReadOnly,
		LeaseRequired,
		LeaseNotHolder,
		PolicyViolation
	};

	struct TopicValue
	{
		ValueType type = ValueType::Bool;
		bool boolValue = false;
		double doubleValue = 0.0;
		std::string stringValue;
		std::vector<std::string> stringArrayValue;

		static TopicValue Bool(bool value);
		static TopicValue Double(double value);
		static TopicValue String(const std::string& value);
		static TopicValue StringArray(const std::vector<std::string>& value);
	};

	struct TopicDescriptor
	{
		std::string topicPath;
		TopicKind topicKind = TopicKind::State;
		ValueType valueType = ValueType::Bool;
		std::string schemaName;
		int schemaVersion = 1;
		RetentionMode retentionMode = RetentionMode::None;
		bool replayOnSubscribe = false;
		int ttlMs = 0;
		WriterPolicy writerPolicy = WriterPolicy::ServerOnly;
		std::string description;
	};

	struct UpdateEnvelope
	{
		std::uint64_t serverSessionId = 0;
		std::uint64_t serverSequence = 0;
		std::uint64_t topicId = 0;
		std::string topicPath;
		std::string sourceClientId;
		DeliveryKind deliveryKind = DeliveryKind::LiveState;
		TopicValue value;
		int ttlMs = 0;
		int ageMsAtEmit = 0;
		bool isStale = false;
		FreshnessReason freshnessReason = FreshnessReason::Live;
		WriteRejectReason rejectionReason = WriteRejectReason::None;
	};

	struct SnapshotEvent
	{
		SnapshotEventKind kind = SnapshotEventKind::DescriptorSnapshotBegin;
		bool hasDescriptor = false;
		TopicDescriptor descriptor;
		bool hasUpdate = false;
		UpdateEnvelope update;
	};

	struct RegisterTopicResult
	{
		bool ok = false;
		std::string message;
		std::uint64_t topicId = 0;
	};

	struct WriteResult
	{
		bool accepted = false;
		WriteRejectReason rejectionReason = WriteRejectReason::None;
		std::uint64_t serverSequence = 0;
	};

	struct ClientSessionView
	{
		std::vector<SnapshotEvent> snapshotEvents;
	};

		struct TopicLeaseInfo
		{
			bool hasLeaseHolder = false;
			std::string leaseHolderClientId;
		};

		struct ServerConfig
		{
			CarrierKind carrierKind = CarrierKind::SharedMemory;
			std::string channelId = "native-link-default";
			std::string host = "127.0.0.1";
			std::uint16_t port = 5810;
		};

		ServerConfig LoadServerConfigFromEnvironment();

		struct TestClientConfig
		{
			CarrierKind carrierKind = CarrierKind::SharedMemory;
			std::string channelId;
			std::string clientId;
			std::string host = "127.0.0.1";
			std::uint16_t port = 5810;
		};

	class Core
	{
	public:
		typedef std::function<std::chrono::steady_clock::time_point(void)> Clock;

		explicit Core(Clock clock = Clock());
		~Core();

		RegisterTopicResult RegisterTopic(const TopicDescriptor& descriptor);
		bool AcquireLease(const std::string& topicPath, const std::string& clientId);
		bool ReleaseLease(const std::string& topicPath, const std::string& clientId);
		ClientSessionView ConnectClient(const std::string& clientId);
		bool DisconnectClient(const std::string& clientId);

		WriteResult Publish(const std::string& topicPath, const TopicValue& value, const std::string& sourceClientId);
		WriteResult PublishFromServer(const std::string& topicPath, const TopicValue& value);

		std::vector<SnapshotEvent> BuildSnapshotForClient(const std::string& clientId) const;
		std::vector<UpdateEnvelope> DrainClientEvents(const std::string& clientId);
		bool TryGetLatestValue(const std::string& topicPath, TopicValue& outValue) const;
		TopicLeaseInfo GetTopicLeaseInfo(const std::string& topicPath) const;
		bool IsTopicRegistered(const std::string& topicPath) const;
		void BeginNewSession();
		std::uint64_t GetServerSessionId() const;

	public:
		struct TopicRuntime
		{
			TopicDescriptor descriptor;
			std::uint64_t topicId = 0;
			bool hasLatestValue = false;
			TopicValue latestValue;
			std::string latestSourceClientId;
			std::chrono::steady_clock::time_point latestValueTime;
			std::string leaseHolderClientId;
		};

		struct ClientRuntime
		{
			std::string clientId;
			std::deque<UpdateEnvelope> pendingEvents;
		};

		const TopicRuntime* LookupTopic(const std::string& topicPath) const;
		DeliveryKind GetLiveDeliveryKind(TopicKind topicKind) const;

	private:

		Clock m_clock;
		std::uint64_t m_serverSessionId;
		std::uint64_t m_nextTopicId;
		std::uint64_t m_nextServerSequence;
		std::vector<TopicRuntime> m_topics;
		std::vector<ClientRuntime> m_clients;

		const TopicRuntime* FindTopic(const std::string& topicPath) const;
		TopicRuntime* FindTopic(const std::string& topicPath);
		const ClientRuntime* FindClient(const std::string& clientId) const;
		ClientRuntime* FindClient(const std::string& clientId);
		static RegisterTopicResult ValidateDescriptor(const TopicDescriptor& descriptor);
		WriteResult PublishInternal(const std::string& topicPath, const TopicValue& value, const std::string& sourceClientId, bool allowServerOnly);
		UpdateEnvelope BuildEnvelope(const TopicRuntime& topic, DeliveryKind deliveryKind, std::uint64_t serverSequence) const;
		DeliveryKind DetermineLiveDeliveryKind(TopicKind topicKind) const;
		void EnqueueLiveEvent(const UpdateEnvelope& envelope);
		void EnqueueEventForClient(const std::string& clientId, const UpdateEnvelope& envelope);
		std::chrono::steady_clock::time_point Now() const;
	};

		class Server
		{
		public:
			explicit Server(const ServerConfig& config = ServerConfig());
			explicit Server(const std::string& channelId);
			~Server();

		bool Start();
		void Stop();

		void PublishBoolean(const std::string& keyName, bool value);
		void PublishNumber(const std::string& keyName, double value);
		void PublishString(const std::string& keyName, const std::string& value);
		void PublishStringArray(const std::string& keyName, const std::vector<std::string>& values);

		bool TryGetBoolean(const std::string& keyName, bool& value) const;
		bool TryGetNumber(const std::string& keyName, double& value) const;
		bool TryGetString(const std::string& keyName, std::string& value) const;

		std::uint64_t GetServerSessionId() const;

	private:
		struct Impl;
		std::unique_ptr<Impl> m_impl;
	};

		class TestClient
		{
		public:
			TestClient(const TestClientConfig& config);
			TestClient(const std::string& channelId, const std::string& clientId);
			~TestClient();

		bool Start();
		void Stop();
		std::vector<SnapshotEvent> Connect(std::uint32_t timeoutMs = 2000);
		std::vector<UpdateEnvelope> DrainLiveEvents(std::uint32_t timeoutMs = 0);

		bool PublishBoolean(const std::string& keyName, bool value);
		bool PublishNumber(const std::string& keyName, double value);
		bool PublishString(const std::string& keyName, const std::string& value);

	private:
		struct Impl;
		std::unique_ptr<Impl> m_impl;
	};

	std::unique_ptr<IConnectionBackend> CreateNativeLinkBackend();
}
