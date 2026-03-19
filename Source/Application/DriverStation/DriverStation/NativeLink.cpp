#include "stdafx.h"
#include "NativeLink.h"

#include <Windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace NativeLink
{
	namespace
	{
		const std::uint32_t kSharedMagic = 0x4E4C4E4B;
		const std::uint32_t kSharedVersion = 3;
		const std::uint32_t kMaxClients = 8;
		const std::uint32_t kMaxMessages = 1024;
		const std::uint32_t kMaxPayloadBytes = 1024;
		const std::uint32_t kSnapshotStartTopicId = 0xFFFFFFF0u;
		const std::uint32_t kSnapshotEndTopicId = 0xFFFFFFF1u;
		const std::uint32_t kLiveBeginTopicId = 0xFFFFFFF2u;

		const char* const kServerClientId = "server";

		struct SharedMessage
		{
			std::uint32_t size = 0;
			std::uint32_t topicId = 0;
			std::uint32_t deliveryKind = 0;
			std::uint32_t valueType = 0;
			std::uint64_t serverSessionId = 0;
			std::uint64_t serverSequence = 0;
			std::uint64_t flags = 0;
			char topicPath[192] = {};
			char sourceClientId[64] = {};
			unsigned char payload[kMaxPayloadBytes] = {};
		};

		struct SharedClientSlot
		{
			std::atomic<std::uint64_t> clientTag;
			std::atomic<std::uint64_t> lastHeartbeatUs;
			std::atomic<std::uint64_t> snapshotCompleteSessionId;
			std::atomic<std::uint64_t> lastAckedSequence;
			std::atomic<std::uint32_t> serverWriteIndex;
			std::atomic<std::uint32_t> clientReadIndex;
			std::atomic<std::uint64_t> clientWriteSequence;
			char clientId[64];
			SharedMessage clientWriteMessage;
			SharedMessage messages[kMaxMessages];
		};

		struct SharedState
		{
			std::uint32_t magic = 0;
			std::uint32_t version = 0;
			std::atomic<std::uint64_t> serverSessionId;
			std::atomic<std::uint64_t> serverBootTimeUs;
			std::atomic<std::uint64_t> lastServerHeartbeatUs;
			std::atomic<std::uint32_t> clientCount;
			// Ian: The mapped client slots contain 64-bit atomics. Keep the array
			// 8-byte aligned in the carrier layout or startup/ack behavior becomes
			// undefined memory access instead of a normal transport ordering bug.
			std::uint32_t reserved0 = 0;
			char channelId[64];
			SharedClientSlot clients[kMaxClients];
		};

		// Ian: The shared carrier already uses fixed-width fields. Keeping the
		// atomics naturally aligned is safer than forcing a packed layout around
		// cross-process atomic state.

		static_assert(offsetof(SharedClientSlot, clientTag) % alignof(std::atomic<std::uint64_t>) == 0, "clientTag alignment");
		static_assert(offsetof(SharedClientSlot, lastHeartbeatUs) % alignof(std::atomic<std::uint64_t>) == 0, "lastHeartbeatUs alignment");
		static_assert(offsetof(SharedClientSlot, snapshotCompleteSessionId) % alignof(std::atomic<std::uint64_t>) == 0, "snapshotCompleteSessionId alignment");
		static_assert(offsetof(SharedClientSlot, lastAckedSequence) % alignof(std::atomic<std::uint64_t>) == 0, "lastAckedSequence alignment");
		static_assert(offsetof(SharedClientSlot, clientWriteSequence) % alignof(std::atomic<std::uint64_t>) == 0, "clientWriteSequence alignment");
		static_assert(offsetof(SharedState, clients) % alignof(std::atomic<std::uint64_t>) == 0, "clients alignment");
		static_assert(sizeof(SharedClientSlot) % alignof(std::atomic<std::uint64_t>) == 0, "slot size alignment");

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

		std::uint64_t GetSteadyNowUs()
		{
			return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
				std::chrono::steady_clock::now().time_since_epoch()).count());
		}

		void CopyUtf8(char* dest, std::size_t destCount, const std::string& value)
		{
			if (!dest || destCount == 0)
				return;

			const std::size_t maxCopy = destCount > 0 ? destCount - 1 : 0;
			const std::size_t copyCount = (std::min)(maxCopy, value.size());
			memcpy(dest, value.data(), copyCount);
			dest[copyCount] = '\0';
		}

		std::string ReadUtf8(const char* src, std::size_t srcCount)
		{
			if (!src || srcCount == 0)
				return std::string();

			const char* end = static_cast<const char*>(memchr(src, '\0', srcCount));
			if (!end)
				return std::string(src, src + srcCount);

			return std::string(src, end);
		}

		std::wstring Utf8ToWide(const std::string& value)
		{
			if (value.empty())
				return std::wstring();

			const int required = MultiByteToWideChar(CP_UTF8, 0, value.c_str(), static_cast<int>(value.size()), nullptr, 0);
			if (required <= 0)
				return std::wstring(value.begin(), value.end());

			std::wstring wide(required, L'\0');
			MultiByteToWideChar(CP_UTF8, 0, value.c_str(), static_cast<int>(value.size()), &wide[0], required);
			return wide;
		}

		std::wstring MakeKernelObjectName(const wchar_t* prefix, const std::string& channelId)
		{
			std::wstring name = prefix;
			std::wstring wideChannel = Utf8ToWide(channelId);
			for (std::size_t i = 0; i < wideChannel.size(); ++i)
			{
				wchar_t ch = wideChannel[i];
				const bool safe =
					(ch >= L'0' && ch <= L'9') ||
					(ch >= L'a' && ch <= L'z') ||
					(ch >= L'A' && ch <= L'Z') ||
					ch == L'-' || ch == L'_';
				name += safe ? ch : L'_';
			}
			return name;
		}

		std::vector<unsigned char> SerializeValue(const TopicValue& value)
		{
			std::vector<unsigned char> bytes;
			switch (value.type)
			{
			case ValueType::Bool:
				bytes.resize(1);
				bytes[0] = value.boolValue ? 1 : 0;
				break;
			case ValueType::Double:
				bytes.resize(sizeof(double));
				memcpy(bytes.data(), &value.doubleValue, sizeof(double));
				break;
			case ValueType::String:
				bytes.assign(value.stringValue.begin(), value.stringValue.end());
				break;
			case ValueType::StringArray:
				for (std::size_t i = 0; i < value.stringArrayValue.size(); ++i)
				{
					const std::string& item = value.stringArrayValue[i];
					const std::uint32_t len = static_cast<std::uint32_t>(item.size());
					const unsigned char* rawLen = reinterpret_cast<const unsigned char*>(&len);
					bytes.insert(bytes.end(), rawLen, rawLen + sizeof(std::uint32_t));
					bytes.insert(bytes.end(), item.begin(), item.end());
				}
				break;
			}
			return bytes;
		}

		bool DeserializeValue(ValueType type, const unsigned char* bytes, std::size_t byteCount, TopicValue& outValue)
		{
			outValue = TopicValue();
			outValue.type = type;

			switch (type)
			{
			case ValueType::Bool:
				if (byteCount < 1)
					return false;
				outValue.boolValue = bytes[0] != 0;
				return true;
			case ValueType::Double:
				if (byteCount < sizeof(double))
					return false;
				memcpy(&outValue.doubleValue, bytes, sizeof(double));
				return true;
			case ValueType::String:
				outValue.stringValue.assign(reinterpret_cast<const char*>(bytes), reinterpret_cast<const char*>(bytes) + byteCount);
				return true;
			case ValueType::StringArray:
				{
					std::size_t offset = 0;
					while (offset + sizeof(std::uint32_t) <= byteCount)
					{
						std::uint32_t len = 0;
						memcpy(&len, bytes + offset, sizeof(std::uint32_t));
						offset += sizeof(std::uint32_t);
						if (offset + len > byteCount)
							return false;
						outValue.stringArrayValue.push_back(std::string(reinterpret_cast<const char*>(bytes + offset), reinterpret_cast<const char*>(bytes + offset + len)));
						offset += len;
					}
					return offset == byteCount;
				}
			}

			return false;
		}

		struct AutoHandle
		{
			HANDLE handle = nullptr;

			~AutoHandle()
			{
				Reset();
			}

			void Reset(HANDLE newHandle = nullptr)
			{
				if (handle)
					CloseHandle(handle);
				handle = newHandle;
			}
		};
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
		return PublishInternal(topicPath, value, kServerClientId, true);
	}

	WriteResult Core::PublishInternal(const std::string& topicPath, const TopicValue& value, const std::string& sourceClientId, bool allowServerOnly)
	{
		TopicRuntime* topic = FindTopic(topicPath);
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
		std::string channelId;
		Core core;
		mutable std::mutex mutex;
		AutoHandle mapping;
		void* mappingView = nullptr;
		SharedState* shared = nullptr;
		AutoHandle clientDataEvent;
		AutoHandle serverHeartbeatEvent;
		std::thread worker;
		std::atomic<bool> running;
		std::atomic<bool> stopRequested;

		Impl(const std::string& inChannelId)
			: channelId(inChannelId)
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
			if (core.IsTopicRegistered("Test/Auton_Selection/AutoChooser/selected"))
				return;

			// Ian: Start the first simulator-owned contract with the same keys that
			// already proved useful in the Direct harness. That gives us a stable
			// parity baseline for 1:1 validation, while Native Link changes the
			// transport semantics underneath (retained snapshot, leases, session
			// authority) instead of also moving the application-level goalposts.
			TopicDescriptor chooserOptions;
			chooserOptions.topicPath = "Test/Auton_Selection/AutoChooser/options";
			chooserOptions.topicKind = TopicKind::State;
			chooserOptions.valueType = ValueType::StringArray;
			chooserOptions.retentionMode = RetentionMode::LatestValue;
			chooserOptions.replayOnSubscribe = true;
			chooserOptions.writerPolicy = WriterPolicy::ServerOnly;
			core.RegisterTopic(chooserOptions);

			TopicDescriptor chooserDefault;
			chooserDefault.topicPath = "Test/Auton_Selection/AutoChooser/default";
			chooserDefault.topicKind = TopicKind::State;
			chooserDefault.valueType = ValueType::String;
			chooserDefault.retentionMode = RetentionMode::LatestValue;
			chooserDefault.replayOnSubscribe = true;
			chooserDefault.writerPolicy = WriterPolicy::ServerOnly;
			core.RegisterTopic(chooserDefault);

			TopicDescriptor chooserActive = chooserDefault;
			chooserActive.topicPath = "Test/Auton_Selection/AutoChooser/active";
			core.RegisterTopic(chooserActive);

			TopicDescriptor chooserSelected = chooserDefault;
			chooserSelected.topicPath = "Test/Auton_Selection/AutoChooser/selected";
			chooserSelected.writerPolicy = WriterPolicy::LeaseSingleWriter;
			core.RegisterTopic(chooserSelected);

			TopicDescriptor testMove;
			testMove.topicPath = "TestMove";
			testMove.topicKind = TopicKind::State;
			testMove.valueType = ValueType::Double;
			testMove.retentionMode = RetentionMode::LatestValue;
			testMove.replayOnSubscribe = true;
			testMove.writerPolicy = WriterPolicy::LeaseSingleWriter;
			core.RegisterTopic(testMove);

			TopicDescriptor timer;
			timer.topicPath = "Timer";
			timer.topicKind = TopicKind::State;
			timer.valueType = ValueType::Double;
			timer.retentionMode = RetentionMode::LatestValue;
			timer.replayOnSubscribe = true;
			timer.writerPolicy = WriterPolicy::ServerOnly;
			core.RegisterTopic(timer);

			TopicDescriptor yFeet = timer;
			yFeet.topicPath = "Y_ft";
			core.RegisterTopic(yFeet);

			core.PublishFromServer("Test/Auton_Selection/AutoChooser/options", TopicValue::StringArray(std::vector<std::string>{ "Do Nothing", "Just Move Forward", "Just Rotate", "Move Rotate Sequence", "Box Waypoints", "Smart Waypoints" }));
			core.PublishFromServer("Test/Auton_Selection/AutoChooser/default", TopicValue::String("Do Nothing"));
			core.PublishFromServer("Test/Auton_Selection/AutoChooser/active", TopicValue::String("Do Nothing"));
			core.PublishFromServer("Test/Auton_Selection/AutoChooser/selected", TopicValue::String("Do Nothing"));
			core.PublishFromServer("TestMove", TopicValue::Double(0.0));
			core.PublishFromServer("Timer", TopicValue::Double(0.0));
			core.PublishFromServer("Y_ft", TopicValue::Double(0.0));
		}

		bool Start()
		{
			if (running.load())
				return true;

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

			if (mappingView)
			{
				UnmapViewOfFile(mappingView);
				mappingView = nullptr;
				shared = nullptr;
			}
		}

		void PublishEnvelopeToAllClients(const UpdateEnvelope& envelope)
		{
			if (!shared)
				return;

			// Ian: This is explicit per-client fan-out, not one shared consumed cursor
			// like the old Direct ring used to have. We already learned the hard way
			// that shared-consumer semantics make extra observers/watchers perturb the
			// main dashboard path, so Native Link starts with one producer stream and
			// independent reader progress per client slot.
			SharedMessage message;
			message.size = sizeof(SharedMessage);
			message.topicId = static_cast<std::uint32_t>(envelope.topicId);
			message.deliveryKind = static_cast<std::uint32_t>(envelope.deliveryKind);
			message.valueType = static_cast<std::uint32_t>(envelope.value.type);
			message.serverSessionId = envelope.serverSessionId;
			message.serverSequence = envelope.serverSequence;
			message.flags = 0;
			CopyUtf8(message.topicPath, sizeof(message.topicPath), envelope.topicPath);
			CopyUtf8(message.sourceClientId, sizeof(message.sourceClientId), envelope.sourceClientId);

			const std::vector<unsigned char> payload = SerializeValue(envelope.value);
			const std::size_t payloadBytes = (std::min<std::size_t>)(payload.size(), sizeof(message.payload));
			if (payloadBytes > 0)
				memcpy(message.payload, payload.data(), payloadBytes);
			message.flags = static_cast<std::uint64_t>(payloadBytes);

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
			for (std::size_t i = 0; i < snapshot.size(); ++i)
			{
				const SnapshotEvent& event = snapshot[i];
				UpdateEnvelope envelope;
				if (event.kind == SnapshotEventKind::Update && event.hasUpdate)
				{
					envelope = event.update;
				}
				else
				{
					envelope.serverSessionId = core.GetServerSessionId();
					envelope.sourceClientId = kServerClientId;
					envelope.value = TopicValue::String(std::string());
					envelope.deliveryKind = DeliveryKind::LiveEvent;
					switch (event.kind)
					{
					case SnapshotEventKind::DescriptorSnapshotBegin:
						envelope.topicId = kSnapshotStartTopicId;
						envelope.topicPath = "__snapshot_begin__";
						break;
					case SnapshotEventKind::DescriptorSnapshotEnd:
						envelope.topicId = kSnapshotEndTopicId;
						envelope.topicPath = "__descriptor_end__";
						break;
					case SnapshotEventKind::StateSnapshotBegin:
						envelope.topicId = kSnapshotStartTopicId;
						envelope.topicPath = "__state_begin__";
						break;
					case SnapshotEventKind::StateSnapshotEnd:
						envelope.topicId = kSnapshotEndTopicId;
						envelope.topicPath = "__state_end__";
						break;
					case SnapshotEventKind::LiveBegin:
						envelope.topicId = kLiveBeginTopicId;
						envelope.topicPath = "__live_begin__";
						break;
					case SnapshotEventKind::Descriptor:
						envelope.topicId = static_cast<std::uint32_t>(event.descriptor.schemaVersion);
						envelope.topicPath = event.descriptor.topicPath;
						envelope.value = TopicValue::String(event.descriptor.topicPath);
						break;
					default:
						break;
					}
				}

				PublishEnvelopeToSingleClient(clientId, envelope);
			}
		}

		void PublishEnvelopeToSingleClient(const std::string& clientId, const UpdateEnvelope& envelope)
		{
			if (!shared)
				return;

			SharedMessage message;
			message.size = sizeof(SharedMessage);
			message.topicId = static_cast<std::uint32_t>(envelope.topicId);
			message.deliveryKind = static_cast<std::uint32_t>(envelope.deliveryKind);
			message.valueType = static_cast<std::uint32_t>(envelope.value.type);
			message.serverSessionId = envelope.serverSessionId;
			message.serverSequence = envelope.serverSequence;
			message.flags = 0;
			CopyUtf8(message.topicPath, sizeof(message.topicPath), envelope.topicPath);
			CopyUtf8(message.sourceClientId, sizeof(message.sourceClientId), envelope.sourceClientId);

			const std::vector<unsigned char> payload = SerializeValue(envelope.value);
			const std::size_t payloadBytes = (std::min<std::size_t>)(payload.size(), sizeof(message.payload));
			if (payloadBytes > 0)
				memcpy(message.payload, payload.data(), payloadBytes);
			message.flags = static_cast<std::uint64_t>(payloadBytes);

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
				TopicValue latest;
				if (core.TryGetLatestValue(message.topicPath, latest))
				{
					const Core::TopicRuntime* topic = core.LookupTopic(message.topicPath);
					if (!topic)
						return;
					UpdateEnvelope live;
					live.serverSessionId = core.GetServerSessionId();
					live.serverSequence = result.serverSequence;
					live.topicId = topic->topicId;
					live.topicPath = message.topicPath;
					live.sourceClientId = clientId;
					live.value = latest;
					// Ian: Snapshot replay and live delivery are separate on purpose. A
					// reconnecting dashboard should first rebuild retained state from the
					// snapshot path and only then treat later writes as live deltas.
					live.deliveryKind = core.GetLiveDeliveryKind(topic->descriptor.topicKind);
					PublishEnvelopeToAllClients(live);
				}
			}
		}

		void RunLoop()
		{
			while (!stopRequested.load())
			{
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

	Server::Server(const std::string& channelId)
		: m_impl(new Impl(channelId))
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
		const WriteResult result = m_impl->core.PublishFromServer(keyName, TopicValue::Bool(value));
		if (result.accepted)
		{
			TopicValue latest;
			if (m_impl->core.TryGetLatestValue(keyName, latest))
			{
				UpdateEnvelope event;
				event.serverSessionId = m_impl->core.GetServerSessionId();
				event.serverSequence = result.serverSequence;
				event.topicPath = keyName;
				event.sourceClientId = kServerClientId;
				event.value = latest;
				event.deliveryKind = DeliveryKind::LiveState;
				m_impl->PublishEnvelopeToAllClients(event);
			}
		}
	}

	void Server::PublishNumber(const std::string& keyName, double value)
	{
		if (!m_impl)
			return;
		std::lock_guard<std::mutex> lock(m_impl->mutex);
		const WriteResult result = m_impl->core.PublishFromServer(keyName, TopicValue::Double(value));
		if (result.accepted)
		{
			TopicValue latest;
			if (m_impl->core.TryGetLatestValue(keyName, latest))
			{
				UpdateEnvelope event;
				event.serverSessionId = m_impl->core.GetServerSessionId();
				event.serverSequence = result.serverSequence;
				event.topicPath = keyName;
				event.sourceClientId = kServerClientId;
				event.value = latest;
				event.deliveryKind = DeliveryKind::LiveState;
				m_impl->PublishEnvelopeToAllClients(event);
			}
		}
	}

	void Server::PublishString(const std::string& keyName, const std::string& value)
	{
		if (!m_impl)
			return;
		std::lock_guard<std::mutex> lock(m_impl->mutex);
		const WriteResult result = m_impl->core.PublishFromServer(keyName, TopicValue::String(value));
		if (result.accepted)
		{
			TopicValue latest;
			if (m_impl->core.TryGetLatestValue(keyName, latest))
			{
				UpdateEnvelope event;
				event.serverSessionId = m_impl->core.GetServerSessionId();
				event.serverSequence = result.serverSequence;
				event.topicPath = keyName;
				event.sourceClientId = kServerClientId;
				event.value = latest;
				event.deliveryKind = DeliveryKind::LiveState;
				m_impl->PublishEnvelopeToAllClients(event);
			}
		}
	}

	void Server::PublishStringArray(const std::string& keyName, const std::vector<std::string>& values)
	{
		if (!m_impl)
			return;
		std::lock_guard<std::mutex> lock(m_impl->mutex);
		const WriteResult result = m_impl->core.PublishFromServer(keyName, TopicValue::StringArray(values));
		if (result.accepted)
		{
			TopicValue latest;
			if (m_impl->core.TryGetLatestValue(keyName, latest))
			{
				UpdateEnvelope event;
				event.serverSessionId = m_impl->core.GetServerSessionId();
				event.serverSequence = result.serverSequence;
				event.topicPath = keyName;
				event.sourceClientId = kServerClientId;
				event.value = latest;
				event.deliveryKind = DeliveryKind::LiveState;
				m_impl->PublishEnvelopeToAllClients(event);
			}
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
		std::uint32_t slotIndex = kMaxClients;
		std::uint64_t clientTag = 0;

		Impl(const std::string& inChannelId, const std::string& inClientId)
			: channelId(inChannelId)
			, clientId(inClientId)
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
				const SharedMessage& message = slot.messages[readIndex % kMaxMessages];
				UpdateEnvelope event;
				event.serverSessionId = message.serverSessionId;
				event.serverSequence = message.serverSequence;
				event.topicId = message.topicId;
				event.topicPath = ReadUtf8(message.topicPath, sizeof(message.topicPath));
				event.sourceClientId = ReadUtf8(message.sourceClientId, sizeof(message.sourceClientId));
				event.deliveryKind = static_cast<DeliveryKind>(message.deliveryKind);
				event.rejectionReason = WriteRejectReason::None;
				DeserializeValue(static_cast<ValueType>(message.valueType), message.payload, static_cast<std::size_t>(message.flags), event.value);
				result.push_back(event);
				++readIndex;
			}
			slot.clientReadIndex.store(readIndex, std::memory_order_release);
			return result;
		}

		bool Publish(const std::string& keyName, const TopicValue& value)
		{
			if (!shared || slotIndex >= kMaxClients)
				return false;

			SharedClientSlot& slot = shared->clients[slotIndex];
			const std::uint64_t writeSequence = slot.clientWriteSequence.load(std::memory_order_acquire) + 1;
			SharedMessage& message = slot.clientWriteMessage;
			memset(&message, 0, sizeof(message));
			message.size = sizeof(message);
			message.valueType = static_cast<std::uint32_t>(value.type);
			CopyUtf8(message.topicPath, sizeof(message.topicPath), keyName);
			CopyUtf8(message.sourceClientId, sizeof(message.sourceClientId), clientId);

			const std::vector<unsigned char> payload = SerializeValue(value);
			const std::size_t payloadBytes = (std::min<std::size_t>)(payload.size(), sizeof(message.payload));
			if (payloadBytes > 0)
				memcpy(message.payload, payload.data(), payloadBytes);
			message.flags = static_cast<std::uint64_t>(payloadBytes);

			slot.clientWriteSequence.store(writeSequence, std::memory_order_release);
			slot.lastHeartbeatUs.store(GetSteadyNowUs(), std::memory_order_release);
			SetEvent(clientDataEvent.handle);
			return true;
		}
	};

	TestClient::TestClient(const std::string& channelId, const std::string& clientId)
		: m_impl(new Impl(channelId, clientId))
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
			: m_server("native-link-default")
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
