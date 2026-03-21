#include "stdafx.h"
#include "NativeLink.h"
#include "NativeLinkAuthorityHelpers.h"
#include "NativeLinkSharedMemory.h"
#include "NativeLinkTcp.h"

#include <Windows.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace NativeLink
{
	namespace
	{
		using detail::AutoHandle;
		using detail::CopyUtf8;
		using detail::DeserializeValue;
		using detail::GetSteadyNowUs;
		using detail::kMaxClients;
		using detail::kMaxMessages;
		using detail::kSharedMagic;
		using detail::kSharedVersion;
		using detail::MakeKernelObjectName;
		using detail::ReadUtf8;
		using detail::SharedClientSlot;
		using detail::SharedMessage;
		using detail::SharedState;

		bool TopicValueMatches(ValueType type, const TopicValue& value)
		{
			switch (type)
			{
			case ValueType::Bool:
				return value.type == ValueType::Bool;
			case ValueType::Double:
				return value.type == ValueType::Double;
			case ValueType::String:
				return value.type == ValueType::String;
			case ValueType::StringArray:
				return value.type == ValueType::StringArray;
			}

			return false;
		}

		std::string ToLowerCopy(const std::string& text)
		{
			std::string normalized(text);
			for (std::size_t i = 0; i < normalized.size(); ++i)
			{
				normalized[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(normalized[i])));
			}
			return normalized;
		}

	}

	const char* ToString(CarrierKind kind)
	{
		switch (kind)
		{
		case CarrierKind::SharedMemory:
			return "shm";
		case CarrierKind::Tcp:
			return "tcp";
		}

		return "unknown";
	}

	bool TryParseCarrierKind(const std::string& text, CarrierKind& outKind)
	{
		const std::string normalized = ToLowerCopy(text);
		if (normalized == "shm" || normalized == "shared_memory" || normalized == "shared-memory")
		{
			outKind = CarrierKind::SharedMemory;
			return true;
		}
		if (normalized == "tcp")
		{
			outKind = CarrierKind::Tcp;
			return true;
		}

		return false;
	}

	ServerConfig LoadServerConfigFromEnvironment()
	{
		ServerConfig config;

		char buffer[256] = {};
		char* value = nullptr;
		std::size_t required = 0;
		if (_dupenv_s(&value, &required, "NATIVE_LINK_CARRIER") == 0 && value != nullptr)
		{
			CarrierKind parsed = config.carrierKind;
			if (TryParseCarrierKind(value, parsed))
				config.carrierKind = parsed;
			free(value);
		}

		DWORD len = GetEnvironmentVariableA("NATIVE_LINK_CHANNEL_ID", buffer, static_cast<DWORD>(sizeof(buffer)));
		if (len > 0 && len < sizeof(buffer))
			config.channelId = buffer;

		len = GetEnvironmentVariableA("NATIVE_LINK_HOST", buffer, static_cast<DWORD>(sizeof(buffer)));
		if (len > 0 && len < sizeof(buffer))
			config.host = buffer;

		len = GetEnvironmentVariableA("NATIVE_LINK_PORT", buffer, static_cast<DWORD>(sizeof(buffer)));
		if (len > 0 && len < sizeof(buffer))
		{
			const unsigned long parsed = std::strtoul(buffer, nullptr, 10);
			if (parsed > 0 && parsed <= 65535UL)
				config.port = static_cast<std::uint16_t>(parsed);
		}

		return config;
	}

	TopicValue TopicValue::Bool(bool value)
	{
		TopicValue result;
		result.type = ValueType::Bool;
		result.boolValue = value;
		return result;
	}

	TopicValue TopicValue::Double(double value)
	{
		TopicValue result;
		result.type = ValueType::Double;
		result.doubleValue = value;
		return result;
	}

	TopicValue TopicValue::String(const std::string& value)
	{
		TopicValue result;
		result.type = ValueType::String;
		result.stringValue = value;
		return result;
	}

	TopicValue TopicValue::StringArray(const std::vector<std::string>& value)
	{
		TopicValue result;
		result.type = ValueType::StringArray;
		result.stringArrayValue = value;
		return result;
	}

	Core::Core(Clock clock)
		: m_clock(clock)
		, m_serverSessionId(1)
		, m_nextTopicId(1)
		, m_nextServerSequence(1)
	{
	}

	Core::~Core() = default;

	RegisterTopicResult Core::RegisterTopic(const TopicDescriptor& descriptor)
	{
		RegisterTopicResult validation = ValidateDescriptor(descriptor);
		if (!validation.ok)
			return validation;

		if (FindTopic(descriptor.topicPath) != nullptr)
			return RegisterTopicResult{ false, "topic already registered", 0 };

		TopicRuntime runtime;
		runtime.descriptor = descriptor;
		runtime.topicId = m_nextTopicId++;
		runtime.latestValueTime = Now();
		m_topics.push_back(runtime);
		return RegisterTopicResult{ true, std::string(), runtime.topicId };
	}

	bool Core::AcquireLease(const std::string& topicPath, const std::string& clientId)
	{
		TopicRuntime* topic = FindTopic(topicPath);
		if (topic == nullptr)
			return false;
		if (topic->descriptor.writerPolicy != WriterPolicy::LeaseSingleWriter)
			return false;
		if (!topic->leaseHolderClientId.empty() && topic->leaseHolderClientId != clientId)
			return false;
		topic->leaseHolderClientId = clientId;
		return true;
	}

	bool Core::ReleaseLease(const std::string& topicPath, const std::string& clientId)
	{
		TopicRuntime* topic = FindTopic(topicPath);
		if (topic == nullptr)
			return false;
		if (topic->leaseHolderClientId != clientId)
			return false;
		topic->leaseHolderClientId.clear();
		return true;
	}

	ClientSessionView Core::ConnectClient(const std::string& clientId)
	{
		ClientRuntime* existing = FindClient(clientId);
		if (existing == nullptr)
		{
			ClientRuntime runtime;
			runtime.clientId = clientId;
			m_clients.push_back(runtime);
		}

		ClientSessionView view;
		view.snapshotEvents = BuildSnapshotForClient(clientId);
		return view;
	}

	bool Core::DisconnectClient(const std::string& clientId)
	{
		const std::vector<ClientRuntime>::iterator it = std::find_if(m_clients.begin(), m_clients.end(), [&clientId](const ClientRuntime& client)
		{
			return client.clientId == clientId;
		});
		if (it == m_clients.end())
			return false;
		m_clients.erase(it);
		return true;
	}

	WriteResult Core::Publish(const std::string& topicPath, const TopicValue& value, const std::string& sourceClientId)
	{
		return PublishInternal(topicPath, value, sourceClientId, false);
	}

	WriteResult Core::PublishFromServer(const std::string& topicPath, const TopicValue& value)
	{
		return PublishInternal(topicPath, value, detail::kServerClientId, true);
	}

	// Ian: PublishInternal is the single write-path gate.  It auto-registers
	// unknown topics for server-originated writes (allowServerOnly=true) so
	// robot code doesn't need to manually pre-declare every telemetry key.
	// Client-originated writes on unknown topics are still rejected, preserving
	// the server-authoritative ownership model: only the simulator may introduce
	// new topics; dashboard clients may only write to pre-declared writable topics.
	WriteResult Core::PublishInternal(const std::string& topicPath, const TopicValue& value, const std::string& sourceClientId, bool allowServerOnly)
	{
		TopicRuntime* topic = FindTopic(topicPath);
		if (topic == nullptr)
		{
			// Ian: Auto-register unknown topics on first server-originated write so
			// any SmartDashboard::PutNumber/PutString call from robot code flows
			// through to clients without requiring manual registration in
			// RegisterDefaultTopics. Client-originated writes on unknown topics
			// are still rejected -- they should only write to pre-declared topics.
			if (allowServerOnly)
			{
				TopicDescriptor autoDesc;
				autoDesc.topicPath = topicPath;
				autoDesc.topicKind = TopicKind::State;
				autoDesc.valueType = value.type;
				autoDesc.retentionMode = RetentionMode::LatestValue;
				autoDesc.replayOnSubscribe = true;
				autoDesc.writerPolicy = WriterPolicy::ServerOnly;
				RegisterTopic(autoDesc);
				topic = FindTopic(topicPath);
			}

			if (topic == nullptr)
			{
				UpdateEnvelope rejectEvent;
				rejectEvent.serverSessionId = m_serverSessionId;
				rejectEvent.topicPath = topicPath;
				rejectEvent.sourceClientId = sourceClientId;
				rejectEvent.deliveryKind = DeliveryKind::LiveCommandReject;
				rejectEvent.value = value;
				rejectEvent.rejectionReason = WriteRejectReason::UnknownTopic;
				EnqueueEventForClient(sourceClientId, rejectEvent);
				return WriteResult{ false, WriteRejectReason::UnknownTopic, 0 };
			}
		}

		if (!TopicValueMatches(topic->descriptor.valueType, value))
		{
			UpdateEnvelope rejectEvent = BuildEnvelope(*topic, DeliveryKind::LiveCommandReject, 0);
			rejectEvent.value = value;
			rejectEvent.rejectionReason = WriteRejectReason::WrongType;
			EnqueueEventForClient(sourceClientId, rejectEvent);
			return WriteResult{ false, WriteRejectReason::WrongType, 0 };
		}

		if (topic->descriptor.writerPolicy == WriterPolicy::ServerOnly && !allowServerOnly)
		{
			UpdateEnvelope rejectEvent = BuildEnvelope(*topic, DeliveryKind::LiveCommandReject, 0);
			rejectEvent.value = value;
			rejectEvent.rejectionReason = WriteRejectReason::ReadOnly;
			EnqueueEventForClient(sourceClientId, rejectEvent);
			return WriteResult{ false, WriteRejectReason::ReadOnly, 0 };
		}

		if (topic->descriptor.writerPolicy == WriterPolicy::LeaseSingleWriter && !allowServerOnly)
		{
			if (topic->leaseHolderClientId.empty())
			{
				UpdateEnvelope rejectEvent = BuildEnvelope(*topic, DeliveryKind::LiveCommandReject, 0);
				rejectEvent.value = value;
				rejectEvent.rejectionReason = WriteRejectReason::LeaseRequired;
				EnqueueEventForClient(sourceClientId, rejectEvent);
				return WriteResult{ false, WriteRejectReason::LeaseRequired, 0 };
			}
			if (topic->leaseHolderClientId != sourceClientId)
			{
				UpdateEnvelope rejectEvent = BuildEnvelope(*topic, DeliveryKind::LiveCommandReject, 0);
				rejectEvent.value = value;
				rejectEvent.rejectionReason = WriteRejectReason::LeaseNotHolder;
				EnqueueEventForClient(sourceClientId, rejectEvent);
				return WriteResult{ false, WriteRejectReason::LeaseNotHolder, 0 };
			}
		}

		topic->hasLatestValue = true;
		topic->latestValue = value;
		topic->latestSourceClientId = sourceClientId;
		topic->latestValueTime = Now();

		const std::uint64_t serverSequence = m_nextServerSequence++;
		const UpdateEnvelope liveEvent = BuildEnvelope(*topic, DetermineLiveDeliveryKind(topic->descriptor.topicKind), serverSequence);
		EnqueueLiveEvent(liveEvent);

		if (!allowServerOnly && topic->descriptor.topicKind == TopicKind::Command)
		{
			UpdateEnvelope ackEvent = liveEvent;
			ackEvent.deliveryKind = DeliveryKind::LiveCommandAck;
			EnqueueEventForClient(sourceClientId, ackEvent);
		}

		return WriteResult{ true, WriteRejectReason::None, serverSequence };
	}

	std::vector<SnapshotEvent> Core::BuildSnapshotForClient(const std::string& clientId) const
	{
		static_cast<void>(clientId);
		std::vector<SnapshotEvent> events;
		events.push_back(SnapshotEvent());

		for (std::size_t i = 0; i < m_topics.size(); ++i)
		{
			SnapshotEvent event;
			event.kind = SnapshotEventKind::Descriptor;
			event.hasDescriptor = true;
			event.descriptor = m_topics[i].descriptor;
			events.push_back(event);
		}

		SnapshotEvent descriptorEnd;
		descriptorEnd.kind = SnapshotEventKind::DescriptorSnapshotEnd;
		events.push_back(descriptorEnd);

		SnapshotEvent stateBegin;
		stateBegin.kind = SnapshotEventKind::StateSnapshotBegin;
		events.push_back(stateBegin);

		for (std::size_t i = 0; i < m_topics.size(); ++i)
		{
			const TopicRuntime& topic = m_topics[i];
			if (!topic.hasLatestValue)
				continue;
			if (topic.descriptor.topicKind != TopicKind::State)
				continue;
			if (topic.descriptor.retentionMode != RetentionMode::LatestValue)
				continue;
			if (!topic.descriptor.replayOnSubscribe)
				continue;

			SnapshotEvent updateEvent;
			updateEvent.kind = SnapshotEventKind::Update;
			updateEvent.hasUpdate = true;
			updateEvent.update = BuildEnvelope(topic, DeliveryKind::SnapshotState, 0);
			events.push_back(updateEvent);
		}

		SnapshotEvent stateEnd;
		stateEnd.kind = SnapshotEventKind::StateSnapshotEnd;
		events.push_back(stateEnd);

		SnapshotEvent liveBegin;
		liveBegin.kind = SnapshotEventKind::LiveBegin;
		events.push_back(liveBegin);
		return events;
	}

	std::vector<UpdateEnvelope> Core::DrainClientEvents(const std::string& clientId)
	{
		ClientRuntime* client = FindClient(clientId);
		if (client == nullptr)
			return std::vector<UpdateEnvelope>();

		std::vector<UpdateEnvelope> events;
		while (!client->pendingEvents.empty())
		{
			events.push_back(client->pendingEvents.front());
			client->pendingEvents.pop_front();
		}
		return events;
	}

	bool Core::TryGetLatestValue(const std::string& topicPath, TopicValue& outValue) const
	{
		const TopicRuntime* topic = FindTopic(topicPath);
		if (topic == nullptr || !topic->hasLatestValue)
			return false;
		outValue = topic->latestValue;
		return true;
	}

	TopicLeaseInfo Core::GetTopicLeaseInfo(const std::string& topicPath) const
	{
		const TopicRuntime* topic = FindTopic(topicPath);
		if (topic == nullptr || topic->leaseHolderClientId.empty())
			return TopicLeaseInfo();

		TopicLeaseInfo info;
		info.hasLeaseHolder = true;
		info.leaseHolderClientId = topic->leaseHolderClientId;
		return info;
	}

	bool Core::IsTopicRegistered(const std::string& topicPath) const
	{
		return FindTopic(topicPath) != nullptr;
	}

	const Core::TopicRuntime* Core::LookupTopic(const std::string& topicPath) const
	{
		return FindTopic(topicPath);
	}

	DeliveryKind Core::GetLiveDeliveryKind(TopicKind topicKind) const
	{
		return DetermineLiveDeliveryKind(topicKind);
	}

	void Core::BeginNewSession()
	{
		++m_serverSessionId;
		m_nextServerSequence = 1;

		for (std::size_t i = 0; i < m_topics.size(); ++i)
			m_topics[i].leaseHolderClientId.clear();

		for (std::size_t i = 0; i < m_clients.size(); ++i)
			m_clients[i].pendingEvents.clear();
	}

	std::uint64_t Core::GetServerSessionId() const
	{
		return m_serverSessionId;
	}

	const Core::TopicRuntime* Core::FindTopic(const std::string& topicPath) const
	{
		for (std::size_t i = 0; i < m_topics.size(); ++i)
		{
			if (m_topics[i].descriptor.topicPath == topicPath)
				return &m_topics[i];
		}
		return nullptr;
	}

	Core::TopicRuntime* Core::FindTopic(const std::string& topicPath)
	{
		for (std::size_t i = 0; i < m_topics.size(); ++i)
		{
			if (m_topics[i].descriptor.topicPath == topicPath)
				return &m_topics[i];
		}
		return nullptr;
	}

	const Core::ClientRuntime* Core::FindClient(const std::string& clientId) const
	{
		for (std::size_t i = 0; i < m_clients.size(); ++i)
		{
			if (m_clients[i].clientId == clientId)
				return &m_clients[i];
		}
		return nullptr;
	}

	Core::ClientRuntime* Core::FindClient(const std::string& clientId)
	{
		for (std::size_t i = 0; i < m_clients.size(); ++i)
		{
			if (m_clients[i].clientId == clientId)
				return &m_clients[i];
		}
		return nullptr;
	}

	RegisterTopicResult Core::ValidateDescriptor(const TopicDescriptor& descriptor)
	{
		if (descriptor.topicPath.empty())
			return RegisterTopicResult{ false, "topic path is required", 0 };
		if (descriptor.topicKind == TopicKind::Command && descriptor.replayOnSubscribe)
			return RegisterTopicResult{ false, "command topics must not replay on subscribe", 0 };
		if (descriptor.topicKind != TopicKind::State && descriptor.retentionMode != RetentionMode::None)
			return RegisterTopicResult{ false, "only state topics may be retained", 0 };
		if (descriptor.topicKind == TopicKind::Event && descriptor.writerPolicy != WriterPolicy::ServerOnly)
			return RegisterTopicResult{ false, "event topics must be server-only", 0 };
		return RegisterTopicResult{ true, std::string(), 0 };
	}

	UpdateEnvelope Core::BuildEnvelope(const TopicRuntime& topic, DeliveryKind deliveryKind, std::uint64_t serverSequence) const
	{
		UpdateEnvelope envelope;
		envelope.serverSessionId = m_serverSessionId;
		envelope.serverSequence = serverSequence;
		envelope.topicId = topic.topicId;
		envelope.topicPath = topic.descriptor.topicPath;
		envelope.sourceClientId = topic.latestSourceClientId;
		envelope.deliveryKind = deliveryKind;
		envelope.value = topic.latestValue;
		envelope.ttlMs = topic.descriptor.ttlMs;

		if (topic.descriptor.ttlMs > 0)
		{
			const long long ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(Now() - topic.latestValueTime).count();
			envelope.ageMsAtEmit = static_cast<int>(ageMs);
			if (ageMs > topic.descriptor.ttlMs)
			{
				envelope.isStale = true;
				envelope.freshnessReason = FreshnessReason::TtlExpired;
			}
		}

		return envelope;
	}

	DeliveryKind Core::DetermineLiveDeliveryKind(TopicKind topicKind) const
	{
		switch (topicKind)
		{
		case TopicKind::State:
			return DeliveryKind::LiveState;
		case TopicKind::Command:
			return DeliveryKind::LiveCommand;
		case TopicKind::Event:
			return DeliveryKind::LiveEvent;
		}
		return DeliveryKind::LiveState;
	}

	void Core::EnqueueLiveEvent(const UpdateEnvelope& envelope)
	{
		for (std::size_t i = 0; i < m_clients.size(); ++i)
			m_clients[i].pendingEvents.push_back(envelope);
	}

	void Core::EnqueueEventForClient(const std::string& clientId, const UpdateEnvelope& envelope)
	{
		ClientRuntime* client = FindClient(clientId);
		if (client)
			client->pendingEvents.push_back(envelope);
	}

	std::chrono::steady_clock::time_point Core::Now() const
	{
		if (m_clock)
			return m_clock();
		return std::chrono::steady_clock::now();
	}

	struct Server::Impl
	{
		ServerConfig config;
		std::string channelId;
		Core core;
		mutable std::mutex mutex;
		AutoHandle mapping;
		void* mappingView = nullptr;
		SharedState* shared = nullptr;
		AutoHandle clientDataEvent;
		AutoHandle serverHeartbeatEvent;
		std::unique_ptr<detail::ITcpServerCarrier> tcpCarrier;
		std::thread worker;
		std::atomic<bool> running;
		std::atomic<bool> stopRequested;

		Impl(const ServerConfig& inConfig)
			: config(inConfig)
			, channelId(inConfig.channelId)
			, running(false)
			, stopRequested(false)
		{
		}

		~Impl()
		{
			Stop();
		}

		void RegisterDefaultTopics()
		{
			detail::RegisterDefaultTopics(core);
		}

		bool Start()
		{
			if (running.load())
				return true;

			if (config.carrierKind == CarrierKind::Tcp)
			{
				RegisterDefaultTopics();
				core.BeginNewSession();
				tcpCarrier = detail::CreateTcpServerCarrier(config, core);
				if (!tcpCarrier || !tcpCarrier->Start())
				{
					tcpCarrier.reset();
					return false;
				}
				stopRequested.store(false);
				running.store(true);
				worker = std::thread(&Impl::RunLoop, this);
				return true;
			}

			if (config.carrierKind != CarrierKind::SharedMemory)
			{
				// Ian: This checkpoint freezes SHM as the reference carrier while
				// making carrier choice explicit in the simulator-owned authority.
				// Return a clean failure for TCP until the real socket carrier exists
				// so selection bugs do not silently fall back to SHM semantics.
				return false;
			}

			const std::wstring mappingName = MakeKernelObjectName(L"Local\\NativeLink.Shared.", channelId);
			const std::wstring clientEventName = MakeKernelObjectName(L"Local\\NativeLink.ClientData.", channelId);
			const std::wstring heartbeatEventName = MakeKernelObjectName(L"Local\\NativeLink.ServerHeartbeat.", channelId);

			mapping.Reset(CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, sizeof(SharedState), mappingName.c_str()));
			if (!mapping.handle)
				return false;

			mappingView = MapViewOfFile(mapping.handle, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(SharedState));
			if (!mappingView)
				return false;

			shared = static_cast<SharedState*>(mappingView);
			const bool needsInit = (shared->magic != kSharedMagic) || (shared->version != kSharedVersion);
			if (needsInit)
			{
				memset(shared, 0, sizeof(SharedState));
				shared->magic = kSharedMagic;
				shared->version = kSharedVersion;
				CopyUtf8(shared->channelId, sizeof(shared->channelId), channelId);
			}

			RegisterDefaultTopics();
			// Ian: A fresh simulator start must always advance the authority session
			// even if the shared-memory mapping name stays the same. Reusing the same
			// mapping across launches is convenient for local IPC, but dashboards need
			// a new server generation so they can discard stale retained assumptions
			// and rebuild from snapshot-first semantics.
			core.BeginNewSession();
			shared->serverSessionId.store(core.GetServerSessionId(), std::memory_order_release);
			shared->serverBootTimeUs.store(GetSteadyNowUs(), std::memory_order_release);
			shared->lastServerHeartbeatUs.store(GetSteadyNowUs(), std::memory_order_release);

			for (std::uint32_t i = 0; i < kMaxClients; ++i)
			{
				shared->clients[i].serverWriteIndex.store(0, std::memory_order_release);
				shared->clients[i].clientReadIndex.store(0, std::memory_order_release);
				shared->clients[i].clientWriteSequence.store(0, std::memory_order_release);
				shared->clients[i].snapshotCompleteSessionId.store(0, std::memory_order_release);
				shared->clients[i].lastAckedSequence.store(0, std::memory_order_release);
			}

			clientDataEvent.Reset(CreateEventW(nullptr, FALSE, FALSE, clientEventName.c_str()));
			serverHeartbeatEvent.Reset(CreateEventW(nullptr, FALSE, FALSE, heartbeatEventName.c_str()));
			if (!clientDataEvent.handle || !serverHeartbeatEvent.handle)
				return false;

			stopRequested.store(false);
			running.store(true);
			worker = std::thread(&Impl::RunLoop, this);
			return true;
		}

		void Stop()
		{
			if (!running.load())
				return;

			stopRequested.store(true);
			if (clientDataEvent.handle)
				SetEvent(clientDataEvent.handle);
			if (worker.joinable())
				worker.join();
			running.store(false);

			if (tcpCarrier)
			{
				tcpCarrier->Stop();
				tcpCarrier.reset();
			}

			if (mappingView)
			{
				UnmapViewOfFile(mappingView);
				mappingView = nullptr;
				shared = nullptr;
			}
		}

		void PublishEnvelopeToAllClients(const UpdateEnvelope& envelope)
		{
			if (tcpCarrier)
			{
				tcpCarrier->PublishEnvelopeToAllClients(envelope);
				return;
			}
			if (!shared)
				return;

			// Ian: This is explicit per-client fan-out, not one shared consumed cursor
			// like the old Direct ring used to have. We already learned the hard way
			// that shared-consumer semantics make extra observers/watchers perturb the
			// main dashboard path, so Native Link starts with one producer stream and
			// independent reader progress per client slot.
			const SharedMessage message = detail::BuildSharedMessage(envelope);

			for (std::uint32_t i = 0; i < kMaxClients; ++i)
			{
				SharedClientSlot& slot = shared->clients[i];
				const std::uint64_t tag = slot.clientTag.load(std::memory_order_acquire);
				if (tag == 0)
					continue;

				const std::uint32_t writeIndex = slot.serverWriteIndex.load(std::memory_order_acquire);
				slot.messages[writeIndex % kMaxMessages] = message;
				slot.serverWriteIndex.store(writeIndex + 1, std::memory_order_release);
			}
		}

		void PublishSnapshotForClient(const std::string& clientId)
		{
			const std::vector<SnapshotEvent> snapshot = core.ConnectClient(clientId).snapshotEvents;
			if (tcpCarrier)
			{
				tcpCarrier->PublishSnapshotForClient(clientId, snapshot);
				return;
			}
			for (std::size_t i = 0; i < snapshot.size(); ++i)
			{
				PublishEnvelopeToSingleClient(clientId, detail::SnapshotEventToEnvelope(snapshot[i], core.GetServerSessionId()));
			}
		}

		void PublishEnvelopeToSingleClient(const std::string& clientId, const UpdateEnvelope& envelope)
		{
			if (tcpCarrier)
			{
				tcpCarrier->PublishEnvelopeToSingleClient(clientId, envelope);
				return;
			}
			if (!shared)
				return;

			const SharedMessage message = detail::BuildSharedMessage(envelope);

			for (std::uint32_t i = 0; i < kMaxClients; ++i)
			{
				SharedClientSlot& slot = shared->clients[i];
				if (ReadUtf8(slot.clientId, sizeof(slot.clientId)) != clientId)
					continue;
				const std::uint32_t writeIndex = slot.serverWriteIndex.load(std::memory_order_acquire);
				slot.messages[writeIndex % kMaxMessages] = message;
				slot.serverWriteIndex.store(writeIndex + 1, std::memory_order_release);
				break;
			}
		}

		void HandleClientWrite(const std::string& clientId, const SharedMessage& message)
		{
			TopicValue value;
			const std::size_t payloadSize = static_cast<std::size_t>(message.flags);
			if (!DeserializeValue(static_cast<ValueType>(message.valueType), message.payload, payloadSize, value))
				return;

			if (core.GetTopicLeaseInfo(message.topicPath).hasLeaseHolder == false)
				core.AcquireLease(message.topicPath, clientId);

			// Ian: For the first simulator-authority slice we opportunistically claim
			// the lease on first writer contact for lease-controlled topics. That keeps
			// the 1:1 operator flow close to the proven Direct experience while still
			// exercising explicit ownership in the core instead of hiding writes behind
			// an implicit last-writer-wins policy.
			const WriteResult result = core.Publish(message.topicPath, value, clientId);
			std::vector<UpdateEnvelope> clientEvents = core.DrainClientEvents(clientId);
			for (std::size_t i = 0; i < clientEvents.size(); ++i)
				PublishEnvelopeToSingleClient(clientId, clientEvents[i]);

			if (result.accepted)
			{
				UpdateEnvelope live;
				if (detail::BuildLiveEnvelopeForTopic(core, message.topicPath, clientId, result.serverSequence, live))
				{
					// Ian: Snapshot replay and live delivery are separate on purpose. A
					// reconnecting dashboard should first rebuild retained state from the
					// snapshot path and only then treat later writes as live deltas.
					PublishEnvelopeToAllClients(live);
				}
			}
		}

		void RunLoop()
		{
			while (!stopRequested.load())
			{
				if (tcpCarrier)
				{
					std::vector<std::pair<std::string, std::pair<std::string, TopicValue>>> writes;
					tcpCarrier->DrainClientWrites(writes);
					for (std::size_t i = 0; i < writes.size(); ++i)
					{
						UpdateEnvelope live;
						if (!core.GetTopicLeaseInfo(writes[i].second.first).hasLeaseHolder)
							core.AcquireLease(writes[i].second.first, writes[i].first);
						const WriteResult result = core.Publish(writes[i].second.first, writes[i].second.second, writes[i].first);
						std::vector<UpdateEnvelope> clientEvents = core.DrainClientEvents(writes[i].first);
						for (std::size_t j = 0; j < clientEvents.size(); ++j)
							PublishEnvelopeToSingleClient(writes[i].first, clientEvents[j]);
						if (result.accepted && detail::BuildLiveEnvelopeForTopic(core, writes[i].second.first, writes[i].first, result.serverSequence, live))
							PublishEnvelopeToAllClients(live);
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
					continue;
				}

				shared->lastServerHeartbeatUs.store(GetSteadyNowUs(), std::memory_order_release);
				if (serverHeartbeatEvent.handle)
					SetEvent(serverHeartbeatEvent.handle);

				for (std::uint32_t i = 0; i < kMaxClients; ++i)
				{
					SharedClientSlot& slot = shared->clients[i];
					const std::uint64_t tag = slot.clientTag.load(std::memory_order_acquire);
					if (tag == 0)
						continue;

					const std::string clientId = ReadUtf8(slot.clientId, sizeof(slot.clientId));
					const std::uint64_t nowUs = GetSteadyNowUs();
					// Ian: The authority samples `nowUs` once, but a dashboard client can
					// refresh `lastHeartbeatUs` immediately afterward. Clamp instead of
					// letting unsigned underflow turn a healthy client into a bogus stale
					// disconnect and slot clear.
					const std::uint64_t heartbeatAgeUs = nowUs >= slot.lastHeartbeatUs.load(std::memory_order_acquire)
						? (nowUs - slot.lastHeartbeatUs.load(std::memory_order_acquire))
						: 0;
					if (heartbeatAgeUs > 5000000ULL)
					{
						core.DisconnectClient(clientId);
						memset(slot.clientId, 0, sizeof(slot.clientId));
						slot.lastHeartbeatUs.store(0, std::memory_order_release);
						slot.snapshotCompleteSessionId.store(0, std::memory_order_release);
						slot.lastAckedSequence.store(0, std::memory_order_release);
						slot.serverWriteIndex.store(0, std::memory_order_release);
						slot.clientReadIndex.store(0, std::memory_order_release);
						slot.clientWriteSequence.store(0, std::memory_order_release);
						slot.clientTag.store(0, std::memory_order_release);
						continue;
					}

					if (slot.snapshotCompleteSessionId.load(std::memory_order_acquire) != core.GetServerSessionId())
					{
						PublishSnapshotForClient(clientId);
						slot.snapshotCompleteSessionId.store(core.GetServerSessionId(), std::memory_order_release);
					}

					const std::uint64_t clientWriteSequence = slot.clientWriteSequence.load(std::memory_order_acquire);
					const std::uint64_t lastAckedSequence = slot.lastAckedSequence.load(std::memory_order_acquire);
					if (clientWriteSequence > lastAckedSequence)
					{
						// Ian: Client->server writes use a tiny "latest command slot" instead of
						// a replayable history. Commands should be observed and applied by the
						// server authority, but they should not be rediscovered as retained
						// state later the way setup/config topics are.
						const SharedMessage message = slot.clientWriteMessage;
						HandleClientWrite(clientId, message);
						slot.lastAckedSequence.store(clientWriteSequence, std::memory_order_release);
					}
				}

				WaitForSingleObject(clientDataEvent.handle, 25);
			}
		}
	};

	Server::Server(const ServerConfig& config)
		: m_impl(new Impl(config))
	{
	}

	Server::Server(const std::string& channelId)
		: Server(ServerConfig{ CarrierKind::SharedMemory, channelId })
	{
	}

	Server::~Server() = default;

	bool Server::Start()
	{
		return m_impl && m_impl->Start();
	}

	void Server::Stop()
	{
		if (m_impl)
			m_impl->Stop();
	}

	void Server::PublishBoolean(const std::string& keyName, bool value)
	{
		if (!m_impl)
			return;
		std::lock_guard<std::mutex> lock(m_impl->mutex);
		UpdateEnvelope event;
		if (detail::PublishServerValue(m_impl->core, keyName, TopicValue::Bool(value), event))
		{
			m_impl->PublishEnvelopeToAllClients(event);
		}
	}

	void Server::PublishNumber(const std::string& keyName, double value)
	{
		if (!m_impl)
			return;
		std::lock_guard<std::mutex> lock(m_impl->mutex);
		UpdateEnvelope event;
		if (detail::PublishServerValue(m_impl->core, keyName, TopicValue::Double(value), event))
		{
			m_impl->PublishEnvelopeToAllClients(event);
		}
	}

	void Server::PublishString(const std::string& keyName, const std::string& value)
	{
		if (!m_impl)
			return;
		std::lock_guard<std::mutex> lock(m_impl->mutex);
		UpdateEnvelope event;
		if (detail::PublishServerValue(m_impl->core, keyName, TopicValue::String(value), event))
		{
			m_impl->PublishEnvelopeToAllClients(event);
		}
	}

	void Server::PublishStringArray(const std::string& keyName, const std::vector<std::string>& values)
	{
		if (!m_impl)
			return;
		std::lock_guard<std::mutex> lock(m_impl->mutex);
		UpdateEnvelope event;
		if (detail::PublishServerValue(m_impl->core, keyName, TopicValue::StringArray(values), event))
		{
			m_impl->PublishEnvelopeToAllClients(event);
		}
	}

	bool Server::TryGetBoolean(const std::string& keyName, bool& value) const
	{
		if (!m_impl)
			return false;
		std::lock_guard<std::mutex> lock(m_impl->mutex);
		TopicValue topicValue;
		if (!m_impl->core.TryGetLatestValue(keyName, topicValue))
			return false;
		if (topicValue.type == ValueType::Bool)
		{
			value = topicValue.boolValue;
			return true;
		}
		return false;
	}

	bool Server::TryGetNumber(const std::string& keyName, double& value) const
	{
		if (!m_impl)
			return false;
		std::lock_guard<std::mutex> lock(m_impl->mutex);
		TopicValue topicValue;
		if (!m_impl->core.TryGetLatestValue(keyName, topicValue))
			return false;
		if (topicValue.type == ValueType::Double)
		{
			value = topicValue.doubleValue;
			return true;
		}
		return false;
	}

	bool Server::TryGetString(const std::string& keyName, std::string& value) const
	{
		if (!m_impl)
			return false;
		std::lock_guard<std::mutex> lock(m_impl->mutex);
		TopicValue topicValue;
		if (!m_impl->core.TryGetLatestValue(keyName, topicValue))
			return false;
		if (topicValue.type == ValueType::String)
		{
			value = topicValue.stringValue;
			return true;
		}
		return false;
	}

	std::uint64_t Server::GetServerSessionId() const
	{
		if (!m_impl)
			return 0;
		return m_impl->core.GetServerSessionId();
	}

	struct TestClient::Impl
	{
		TestClientConfig config;
		std::string channelId;
		std::string clientId;
		std::wstring mappingName;
		std::wstring clientEventName;
		std::wstring heartbeatEventName;
		AutoHandle mapping;
		void* mappingView = nullptr;
		SharedState* shared = nullptr;
		AutoHandle clientDataEvent;
		AutoHandle heartbeatEvent;
		std::unique_ptr<detail::ITcpClientCarrier> tcpCarrier;
		std::uint32_t slotIndex = kMaxClients;
		std::uint64_t clientTag = 0;

		Impl(const TestClientConfig& inConfig)
			: config(inConfig)
			, channelId(inConfig.channelId)
			, clientId(inConfig.clientId)
		{
			mappingName = MakeKernelObjectName(L"Local\\NativeLink.Shared.", channelId);
			clientEventName = MakeKernelObjectName(L"Local\\NativeLink.ClientData.", channelId);
			heartbeatEventName = MakeKernelObjectName(L"Local\\NativeLink.ServerHeartbeat.", channelId);
		}

		~Impl()
		{
			Stop();
		}

		bool Start()
		{
			if (config.carrierKind == CarrierKind::Tcp)
			{
				tcpCarrier = detail::CreateTcpClientCarrier(config);
				return tcpCarrier && tcpCarrier->Start();
			}

			if (config.carrierKind != CarrierKind::SharedMemory)
			{
				return false;
			}

			mapping.Reset(OpenFileMappingW(FILE_MAP_ALL_ACCESS, FALSE, mappingName.c_str()));
			if (!mapping.handle)
				return false;

			mappingView = MapViewOfFile(mapping.handle, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(SharedState));
			if (!mappingView)
				return false;

			shared = static_cast<SharedState*>(mappingView);
			if (shared->magic != kSharedMagic || shared->version != kSharedVersion)
				return false;

			clientDataEvent.Reset(OpenEventW(EVENT_MODIFY_STATE | SYNCHRONIZE, FALSE, clientEventName.c_str()));
			heartbeatEvent.Reset(OpenEventW(SYNCHRONIZE, FALSE, heartbeatEventName.c_str()));
			if (!clientDataEvent.handle || !heartbeatEvent.handle)
				return false;

			clientTag = GetSteadyNowUs() ^ reinterpret_cast<std::uintptr_t>(this);
			for (std::uint32_t i = 0; i < kMaxClients; ++i)
			{
				SharedClientSlot& slot = shared->clients[i];
				std::uint64_t expected = 0;
				if (slot.clientTag.compare_exchange_strong(expected, clientTag, std::memory_order_acq_rel))
				{
					slotIndex = i;
					memset(slot.clientId, 0, sizeof(slot.clientId));
					slot.lastHeartbeatUs.store(0, std::memory_order_release);
					CopyUtf8(slot.clientId, sizeof(slot.clientId), clientId);
					slot.lastHeartbeatUs.store(GetSteadyNowUs(), std::memory_order_release);
					slot.snapshotCompleteSessionId.store(0, std::memory_order_release);
					slot.lastAckedSequence.store(0, std::memory_order_release);
					slot.serverWriteIndex.store(0, std::memory_order_release);
					slot.clientReadIndex.store(0, std::memory_order_release);
					slot.clientWriteSequence.store(0, std::memory_order_release);
					shared->clientCount.fetch_add(1, std::memory_order_acq_rel);
					return true;
				}
			}

			return false;
		}

		void Stop()
		{
			if (tcpCarrier)
			{
				tcpCarrier->Stop();
				tcpCarrier.reset();
			}

			if (shared && slotIndex < kMaxClients)
			{
				shared->clients[slotIndex].clientTag.store(0, std::memory_order_release);
				shared->clientCount.fetch_sub(1, std::memory_order_acq_rel);
				slotIndex = kMaxClients;
			}
			if (mappingView)
			{
				UnmapViewOfFile(mappingView);
				mappingView = nullptr;
				shared = nullptr;
			}
		}

		std::vector<UpdateEnvelope> DrainMessages(std::uint32_t timeoutMs)
		{
			std::vector<UpdateEnvelope> result;
			if (tcpCarrier)
				return tcpCarrier->DrainMessages(timeoutMs);
			if (!shared || slotIndex >= kMaxClients)
				return result;

			if (timeoutMs > 0 && heartbeatEvent.handle)
				WaitForSingleObject(heartbeatEvent.handle, timeoutMs);

			SharedClientSlot& slot = shared->clients[slotIndex];
			slot.lastHeartbeatUs.store(GetSteadyNowUs(), std::memory_order_release);

			std::uint32_t readIndex = slot.clientReadIndex.load(std::memory_order_acquire);
			const std::uint32_t writeIndex = slot.serverWriteIndex.load(std::memory_order_acquire);
			while (readIndex < writeIndex)
			{
				result.push_back(detail::SharedMessageToUpdateEnvelope(slot.messages[readIndex % kMaxMessages]));
				++readIndex;
			}
			slot.clientReadIndex.store(readIndex, std::memory_order_release);
			return result;
		}

		bool Publish(const std::string& keyName, const TopicValue& value)
		{
			if (tcpCarrier)
				return tcpCarrier->Publish(keyName, value);
			if (!shared || slotIndex >= kMaxClients)
				return false;

			SharedClientSlot& slot = shared->clients[slotIndex];
			const std::uint64_t writeSequence = slot.clientWriteSequence.load(std::memory_order_acquire) + 1;
			slot.clientWriteMessage = detail::BuildClientWriteMessage(keyName, clientId, value);

			slot.clientWriteSequence.store(writeSequence, std::memory_order_release);
			slot.lastHeartbeatUs.store(GetSteadyNowUs(), std::memory_order_release);
			SetEvent(clientDataEvent.handle);
			return true;
		}
	};

	TestClient::TestClient(const TestClientConfig& config)
		: m_impl(new Impl(config))
	{
	}

	TestClient::TestClient(const std::string& channelId, const std::string& clientId)
		: TestClient(TestClientConfig{ CarrierKind::SharedMemory, channelId, clientId })
	{
	}

	TestClient::~TestClient() = default;

	bool TestClient::Start()
	{
		return m_impl && m_impl->Start();
	}

	void TestClient::Stop()
	{
		if (m_impl)
			m_impl->Stop();
	}

	std::vector<SnapshotEvent> TestClient::Connect(std::uint32_t timeoutMs)
	{
		std::vector<SnapshotEvent> snapshot;
		if (!m_impl)
			return snapshot;

		const std::vector<UpdateEnvelope> events = m_impl->DrainMessages(timeoutMs);
		for (std::size_t i = 0; i < events.size(); ++i)
		{
			const UpdateEnvelope& event = events[i];
			SnapshotEvent item;
			if (event.topicPath == "__snapshot_begin__")
				item.kind = SnapshotEventKind::DescriptorSnapshotBegin;
			else if (event.topicPath == "__descriptor_end__")
				item.kind = SnapshotEventKind::DescriptorSnapshotEnd;
			else if (event.topicPath == "__state_begin__")
				item.kind = SnapshotEventKind::StateSnapshotBegin;
			else if (event.topicPath == "__state_end__")
				item.kind = SnapshotEventKind::StateSnapshotEnd;
			else if (event.topicPath == "__live_begin__")
				item.kind = SnapshotEventKind::LiveBegin;
			else
			{
				item.kind = SnapshotEventKind::Update;
				item.hasUpdate = true;
				item.update = event;
			}
			snapshot.push_back(item);
		}
		return snapshot;
	}

	std::vector<UpdateEnvelope> TestClient::DrainLiveEvents(std::uint32_t timeoutMs)
	{
		if (!m_impl)
			return std::vector<UpdateEnvelope>();
		return m_impl->DrainMessages(timeoutMs);
	}

	bool TestClient::PublishBoolean(const std::string& keyName, bool value)
	{
		return m_impl && m_impl->Publish(keyName, TopicValue::Bool(value));
	}

	bool TestClient::PublishNumber(const std::string& keyName, double value)
	{
		return m_impl && m_impl->Publish(keyName, TopicValue::Double(value));
	}

	bool TestClient::PublishString(const std::string& keyName, const std::string& value)
	{
		return m_impl && m_impl->Publish(keyName, TopicValue::String(value));
	}

	class NativeLinkBackend final : public IConnectionBackend
		, public SmartDashboardDirectPublishSink
		, public SmartDashboardDirectQuerySource
	{
	public:
		NativeLinkBackend()
			: m_server(LoadServerConfigFromEnvironment())
		{
		}

		void Initialize() override
		{
			SmartDashboard::SetConnectionMode(SmartDashboardConnectionMode::eNativeLink);
			SmartDashboard::SetDirectPublishSink(this);
			SmartDashboard::SetDirectQuerySource(this);
			m_running = m_server.Start();
			OutputDebugStringW(m_running
				? L"[Transport] Native Link backend initialized\n"
				: L"[Transport] Native Link backend failed to start\n");
		}

		void Shutdown() override
		{
			SmartDashboard::ClearDirectPublishSink();
			SmartDashboard::ClearDirectQuerySource();
			m_server.Stop();
			m_running = false;
			OutputDebugStringW(L"[Transport] Native Link backend shutdown requested\n");
		}

		const wchar_t* GetBackendName() const override
		{
			return m_running ? L"Native Link" : L"Native Link (inactive)";
		}

		void PublishBoolean(const std::string& keyName, bool value) override
		{
			if (m_running)
				m_server.PublishBoolean(keyName, value);
		}

		void PublishNumber(const std::string& keyName, double value) override
		{
			if (m_running)
				m_server.PublishNumber(keyName, value);
		}

		void PublishString(const std::string& keyName, const std::string& value) override
		{
			if (m_running)
				m_server.PublishString(keyName, value);
		}

		void PublishStringArray(const std::string& keyName, const std::vector<std::string>& values) override
		{
			if (m_running)
				m_server.PublishStringArray(keyName, values);
		}

		bool TryGetBoolean(const std::string& keyName, bool& value) override
		{
			return m_running && m_server.TryGetBoolean(keyName, value);
		}

		bool TryGetNumber(const std::string& keyName, double& value) override
		{
			return m_running && m_server.TryGetNumber(keyName, value);
		}

		bool TryGetString(const std::string& keyName, std::string& value) override
		{
			return m_running && m_server.TryGetString(keyName, value);
		}

	private:
		Server m_server;
		bool m_running = false;
	};

	std::unique_ptr<IConnectionBackend> CreateNativeLinkBackend()
	{
		return std::unique_ptr<IConnectionBackend>(new NativeLinkBackend());
	}
}
