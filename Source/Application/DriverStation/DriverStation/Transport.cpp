#include "stdafx.h"
#include "Transport.h"
#include "NativeLink.h"
#include "NT4Server.h"
#include "MjpegServer.h"
#include "SimCameraSource.h"

#include "../../../Libraries/SmartDashboard/SmartDashboard_Import.h"

#include <Windows.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace
{
	void AppendTransportLogLine(const std::string& line)
	{
		static std::mutex logMutex;
		static std::ofstream log = []()
		{
			char tempDir[MAX_PATH] = {};
			GetTempPathA(static_cast<DWORD>(_countof(tempDir)), tempDir);
			std::string logPath = tempDir;
			if (!logPath.empty() && logPath.back() != '\\')
				logPath += '\\';
			logPath += "direct_transport_debug_log.txt";
			return std::ofstream(logPath, std::ios::out | std::ios::trunc);
		}();
		std::lock_guard<std::mutex> lock(logMutex);
		if (!log.is_open())
			return;
		log << line << '\n';
		log.flush();
	}

	void AppendTransportLogLine(const wchar_t* line)
	{
		if (!line)
			return;
		char buffer[1024] = {};
		size_t converted = 0;
		wcstombs_s(&converted, buffer, line, _TRUNCATE);
		AppendTransportLogLine(buffer);
	}

	class IDirectPublisher
	{
	public:
		virtual ~IDirectPublisher() = default;
		virtual bool Start() = 0;
		virtual void Stop() = 0;
		virtual void PublishBool(const std::string& key, bool value) = 0;
		virtual void PublishDouble(const std::string& key, double value) = 0;
		virtual void PublishString(const std::string& key, const std::string& value) = 0;
		virtual void PublishStringArray(const std::string& key, const std::vector<std::string>& values) = 0;
		virtual bool TryGetBoolean(const std::string& key, bool& value) const = 0;
		virtual bool TryGetNumber(const std::string& key, double& value) const = 0;
		virtual bool TryGetString(const std::string& key, std::string& value) const = 0;
		virtual bool FlushNow() = 0;
	};

	class DirectPublisherStub final : public IDirectPublisher
	{
	public:
		explicit DirectPublisherStub(
			const std::wstring& mappingName,
			const std::wstring& dataEventName,
			const std::wstring& heartbeatEventName)
			: m_mappingName(mappingName)
			, m_dataEventName(dataEventName)
			, m_heartbeatEventName(heartbeatEventName)
		{
		}

		~DirectPublisherStub() override
		{
			Stop();
		}

		bool Start() override
		{
			if (m_running.load())
				return true;

			const std::size_t mappingBytes = sizeof(RingHeader) + kDefaultCapacityBytes;
			m_mapping = CreateFileMappingW(
				INVALID_HANDLE_VALUE,
				nullptr,
				PAGE_READWRITE,
				static_cast<DWORD>((static_cast<unsigned long long>(mappingBytes) >> 32U) & 0xFFFFFFFFULL),
				static_cast<DWORD>(static_cast<unsigned long long>(mappingBytes) & 0xFFFFFFFFULL),
				m_mappingName.c_str());
			if (m_mapping == nullptr)
				return false;

			m_view = MapViewOfFile(m_mapping, FILE_MAP_ALL_ACCESS, 0, 0, mappingBytes);
			if (m_view == nullptr)
			{
				Close();
				return false;
			}

			m_dataEvent = CreateEventW(nullptr, FALSE, FALSE, m_dataEventName.c_str());
			if (m_dataEvent == nullptr)
			{
				Close();
				return false;
			}

			m_heartbeatEvent = CreateEventW(nullptr, FALSE, FALSE, m_heartbeatEventName.c_str());
			if (m_heartbeatEvent == nullptr)
			{
				Close();
				return false;
			}

			auto* header = static_cast<RingHeader*>(m_view);
			const std::uint32_t payloadCapacity = static_cast<std::uint32_t>(mappingBytes - sizeof(RingHeader));
			// Read magic/version/capacity from raw bytes before any atomic access.
			// These four fields are plain uint32_t (non-atomic) at the start of RingHeader,
			// so a memcpy read is safe even before placement-new.
			std::uint32_t rawMagic = 0, rawVersion = 0, rawCapacity = 0;
			std::memcpy(&rawMagic,    static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, magic),         sizeof(rawMagic));
			std::memcpy(&rawVersion,  static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, version),       sizeof(rawVersion));
			std::memcpy(&rawCapacity, static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, capacityBytes), sizeof(rawCapacity));
			const bool needsInit =
				(rawMagic    != kMagic) ||
				(rawVersion  != kVersion) ||
				(rawCapacity != payloadCapacity);
			if (needsInit)
			{
				// Zero the payload region only (leave header bytes — placement-new will overwrite them).
				std::memset(static_cast<std::uint8_t*>(m_view) + sizeof(RingHeader), 0, kDefaultCapacityBytes);
				// Placement-new constructs all std::atomic members in this process's address space.
				header = new (m_view) RingHeader{};
				header->magic         = kMagic;
				header->version       = kVersion;
				header->reserved0     = 0;
				header->capacityBytes = payloadCapacity;
				header->writeIndex.store(0, std::memory_order_release);
				header->readIndex.store(0, std::memory_order_release);
				header->publishedSeq.store(0, std::memory_order_release);
				header->droppedCount.store(0, std::memory_order_release);
				const std::uint64_t nowUs = GetSteadyNowUs();
				header->lastProducerHeartbeatUs.store(nowUs, std::memory_order_release);
				header->lastConsumerHeartbeatUs.store(nowUs, std::memory_order_release);
				header->consumerInstanceId.store(0, std::memory_order_release);
				header->consumerReadIndex.store(0, std::memory_order_release);
			}
			else
			{
				// SHM already initialized by a prior process. Placement-new to construct
				// the std::atomic members in *this* process — required in MSVC Debug.
				// Save the live scalar values first via memcpy (safe — no atomic access yet).
				std::uint32_t savedWriteIndex = 0, savedReadIndex = 0,
				              savedPublishedSeq = 0, savedDroppedCount = 0,
				              savedConsumerInstanceId = 0, savedConsumerReadIndex = 0;
				std::uint64_t savedLastProducerUs = 0, savedLastConsumerUs = 0;
				std::memcpy(&savedWriteIndex,          static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, writeIndex),           sizeof(savedWriteIndex));
				std::memcpy(&savedReadIndex,           static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, readIndex),            sizeof(savedReadIndex));
				std::memcpy(&savedPublishedSeq,        static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, publishedSeq),         sizeof(savedPublishedSeq));
				std::memcpy(&savedDroppedCount,        static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, droppedCount),         sizeof(savedDroppedCount));
				std::memcpy(&savedLastProducerUs,      static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, lastProducerHeartbeatUs), sizeof(savedLastProducerUs));
				std::memcpy(&savedLastConsumerUs,      static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, lastConsumerHeartbeatUs), sizeof(savedLastConsumerUs));
				std::memcpy(&savedConsumerInstanceId,  static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, consumerInstanceId),   sizeof(savedConsumerInstanceId));
				std::memcpy(&savedConsumerReadIndex,   static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, consumerReadIndex),    sizeof(savedConsumerReadIndex));
				// Placement-new constructs atomics, then restore saved values.
				header = new (m_view) RingHeader{};
				header->magic         = rawMagic;
				header->version       = rawVersion;
				header->capacityBytes = rawCapacity;
				header->writeIndex.store(savedWriteIndex,         std::memory_order_relaxed);
				header->readIndex.store(savedReadIndex,           std::memory_order_relaxed);
				header->publishedSeq.store(savedPublishedSeq,     std::memory_order_relaxed);
				header->droppedCount.store(savedDroppedCount,     std::memory_order_relaxed);
				header->lastProducerHeartbeatUs.store(savedLastProducerUs,     std::memory_order_relaxed);
				header->lastConsumerHeartbeatUs.store(savedLastConsumerUs,     std::memory_order_relaxed);
				header->consumerInstanceId.store(savedConsumerInstanceId,     std::memory_order_relaxed);
				header->consumerReadIndex.store(savedConsumerReadIndex,       std::memory_order_relaxed);
			}

			m_header = header;
			m_payload = reinterpret_cast<std::uint8_t*>(header + 1);
			m_capacity = header->capacityBytes;
			m_sequence = header->publishedSeq.load(std::memory_order_acquire);

			m_running.store(true);
			m_worker = std::thread(&DirectPublisherStub::RunLoop, this);
			return true;
		}

		void Stop() override
		{
			if (!m_running.load())
				return;

			FlushNow();
			m_running.store(false);
			if (m_worker.joinable())
				m_worker.join();

			m_consumerWasActive = false;

			Close();
		}

		void PublishBool(const std::string& key, bool value) override
		{
			PendingValue pending;
			pending.type = ValueType::Bool;
			pending.boolValue = value;
			StorePending(key, pending);
		}

		void PublishDouble(const std::string& key, double value) override
		{
			PendingValue pending;
			pending.type = ValueType::Double;
			pending.doubleValue = value;
			StorePending(key, pending);
		}

		void PublishString(const std::string& key, const std::string& value) override
		{
			PendingValue pending;
			pending.type = ValueType::String;
			pending.stringValue = value;
			StorePending(key, pending);
		}

		void PublishStringArray(const std::string& key, const std::vector<std::string>& values) override
		{
			PendingValue pending;
			pending.type = ValueType::StringArray;
			pending.stringArrayValue = values;
			StorePending(key, pending);
		}

		bool TryGetBoolean(const std::string& key, bool& value) const override
		{
			std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(m_retainedMutex));
			auto it = m_retained.find(key);
			if (it == m_retained.end() || it->second.type != ValueType::Bool)
				return false;
			value = it->second.boolValue;
			return true;
		}

		bool TryGetNumber(const std::string& key, double& value) const override
		{
			std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(m_retainedMutex));
			auto it = m_retained.find(key);
			if (it == m_retained.end())
			{
				if (key == "TestMove" || key == "Test/TestMove")
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[DirectPublisherStub] TryGetNumber miss key='%s'", key.c_str());
					AppendTransportLogLine(dbg);
				}
				return false;
			}
			if (it->second.type == ValueType::Double)
			{
				value = it->second.doubleValue;
				if (key == "TestMove" || key == "Test/TestMove")
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[DirectPublisherStub] TryGetNumber key='%s' type=double value=%g", key.c_str(), value);
					AppendTransportLogLine(dbg);
				}
				return true;
			}
			if (it->second.type == ValueType::String)
			{
				value = atof(it->second.stringValue.c_str());
				if (key == "TestMove" || key == "Test/TestMove")
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[DirectPublisherStub] TryGetNumber key='%s' type=string raw='%s' parsed=%g", key.c_str(), it->second.stringValue.c_str(), value);
					AppendTransportLogLine(dbg);
				}
				return true;
			}
			if (it->second.type == ValueType::Bool)
			{
				value = it->second.boolValue ? 1.0 : 0.0;
				if (key == "TestMove" || key == "Test/TestMove")
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[DirectPublisherStub] TryGetNumber key='%s' type=bool value=%g", key.c_str(), value);
					AppendTransportLogLine(dbg);
				}
				return true;
			}
			if (key == "TestMove" || key == "Test/TestMove")
			{
				char dbg[256] = {};
				sprintf_s(dbg, "[DirectPublisherStub] TryGetNumber key='%s' unsupported_type=%u", key.c_str(), static_cast<unsigned>(it->second.type));
				AppendTransportLogLine(dbg);
			}
			return false;
		}

		bool TryGetString(const std::string& key, std::string& value) const override
		{
			std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(m_retainedMutex));
			auto it = m_retained.find(key);
			if (it == m_retained.end())
				return false;
			if (it->second.type == ValueType::String)
			{
				value = it->second.stringValue;
				return true;
			}
			if (it->second.type == ValueType::StringArray)
			{
				value.clear();
				for (size_t i = 0; i < it->second.stringArrayValue.size(); ++i)
				{
					if (i > 0)
						value += ",";
					value += it->second.stringArrayValue[i];
				}
				return true;
			}
			return false;
		}

		bool FlushNow() override
		{
			if (!m_running.load() || m_header == nullptr)
				return false;

			std::unordered_map<std::string, PendingValue> pending;
			{
				std::lock_guard<std::mutex> lock(m_pendingMutex);
				pending.swap(m_pending);
			}

			bool published = false;
			for (const auto& entry : pending)
			{
				published = WritePendingValue(entry.first, entry.second) || published;
			}

			m_header->lastProducerHeartbeatUs.store(GetSteadyNowUs(), std::memory_order_release);
			if (published && m_dataEvent != nullptr)
				SetEvent(m_dataEvent);
			if (m_heartbeatEvent != nullptr)
				SetEvent(m_heartbeatEvent);
			return true;
		}

	private:
		enum class ValueType : std::uint8_t
		{
			Bool = 1,
			Double = 2,
			String = 3,
			StringArray = 4
		};

		struct PendingValue
		{
			ValueType type = ValueType::String;
			bool boolValue = false;
			double doubleValue = 0.0;
			std::string stringValue;
			std::vector<std::string> stringArrayValue;
		};

		struct alignas(8) RingHeader
		{
			std::uint32_t magic;
			std::uint16_t version;
			std::uint16_t reserved0;
			std::uint32_t capacityBytes;
			std::atomic<std::uint32_t> writeIndex;
			std::atomic<std::uint32_t> readIndex;
			std::atomic<std::uint64_t> publishedSeq;
			std::atomic<std::uint64_t> droppedCount;
			std::atomic<std::uint64_t> lastProducerHeartbeatUs;
			std::atomic<std::uint64_t> lastConsumerHeartbeatUs;
			std::atomic<std::uint64_t> consumerInstanceId;
			std::atomic<std::uint32_t> consumerReadIndex;
		};

		struct alignas(8) MessageHeader
		{
			std::uint16_t messageBytes;
			std::uint8_t messageType;
			std::uint8_t valueType;
			std::uint64_t seq;
			std::uint64_t sourceTimestampUs;
			std::uint16_t keyLen;
			std::uint16_t valueLen;
		};

		static constexpr std::uint32_t kMagic = 0x53444442;
		static constexpr std::uint16_t kVersion = 1;
		static constexpr std::uint32_t kDefaultCapacityBytes = 1U << 20;
		static constexpr std::size_t kKeyMax = 128;
		static constexpr std::size_t kStringMax = 256;
		static constexpr std::size_t kStringArrayMaxCount = 32;
		static constexpr std::uint8_t kMsgTypeUpsert = 1;

		static std::uint64_t GetSteadyNowUs()
		{
			return static_cast<std::uint64_t>(
				std::chrono::duration_cast<std::chrono::microseconds>(
					std::chrono::steady_clock::now().time_since_epoch()).count());
		}

		void RunLoop()
		{
			std::uint64_t previousLoopUs = 0;
			while (m_running.load())
			{
				try
				{
					const std::uint64_t nowUs = GetSteadyNowUs();
					if (previousLoopUs != 0)
					{
						const std::uint64_t loopDeltaUs = nowUs - previousLoopUs;
						if (loopDeltaUs > 250000ULL)
						{
							char dbgGap[256] = {};
							sprintf_s(
								dbgGap,
								"[Transport] Telemetry run loop gap_us=%llu",
								static_cast<unsigned long long>(loopDeltaUs));
							AppendTransportLogLine(dbgGap);
						}
					}
					previousLoopUs = nowUs;

					MaybeReplayRetainedSnapshot();
					FlushNow();
					if (m_header != nullptr)
					{
						const std::uint64_t lastConsumerHeartbeatUs = m_header->lastConsumerHeartbeatUs.load(std::memory_order_acquire);
						const bool consumerActive =
							(lastConsumerHeartbeatUs != 0) &&
							(nowUs >= lastConsumerHeartbeatUs) &&
							((nowUs - lastConsumerHeartbeatUs) <= 500000ULL);
						if (consumerActive != m_loggedConsumerActive)
						{
							char dbgA[256] = {};
							sprintf_s(
								dbgA,
								"[Transport] Telemetry consumer active=%d heartbeat_delta_us=%llu",
								consumerActive ? 1 : 0,
								static_cast<unsigned long long>(consumerActive ? (nowUs - lastConsumerHeartbeatUs) : 0ULL));
							OutputDebugStringA(dbgA);
							OutputDebugStringA("\n");
							AppendTransportLogLine(dbgA);
							m_loggedConsumerActive = consumerActive;
						}
					}
				}
				catch (const std::exception& ex)
				{
					char dbg[512] = {};
					sprintf_s(dbg, "[Publisher] RunLoop caught std::exception: %s\n", ex.what());
					OutputDebugStringA(dbg);
					AppendTransportLogLine(std::string("[Publisher] RunLoop caught std::exception: ") + ex.what());
				}
				catch (...)
				{
					OutputDebugStringA("[Publisher] RunLoop caught unknown exception\n");
					AppendTransportLogLine("[Publisher] RunLoop caught unknown exception");
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(16));
			}
		}

		void StorePending(const std::string& key, const PendingValue& pending)
		{
			{
				std::lock_guard<std::mutex> retainedLock(m_retainedMutex);
				m_retained[key] = pending;
			}
			std::lock_guard<std::mutex> lock(m_pendingMutex);
			m_pending[key] = pending;
		}

		void MaybeReplayRetainedSnapshot()
		{
			if (m_header == nullptr)
				return;

			const std::uint64_t lastConsumerHeartbeatUs = m_header->lastConsumerHeartbeatUs.load(std::memory_order_acquire);
			const std::uint64_t nowUs = GetSteadyNowUs();
			const bool consumerActive =
				(lastConsumerHeartbeatUs != 0) &&
				(nowUs >= lastConsumerHeartbeatUs) &&
				((nowUs - lastConsumerHeartbeatUs) <= 500000ULL);

			if (!consumerActive)
			{
				m_consumerWasActive = false;
				return;
			}

			if (m_consumerWasActive)
				return;

			m_consumerWasActive = true;

			std::unordered_map<std::string, PendingValue> retainedCopy;
			{
				std::lock_guard<std::mutex> retainedLock(m_retainedMutex);
				retainedCopy = m_retained;
			}

			if (retainedCopy.empty())
				return;

			{
				std::lock_guard<std::mutex> pendingLock(m_pendingMutex);
				for (const auto& entry : retainedCopy)
					m_pending[entry.first] = entry.second;
			}

			OutputDebugStringW(L"[Transport] Direct publisher replayed retained snapshot for dashboard reconnect\n");
			AppendTransportLogLine("[Transport] Direct publisher replayed retained snapshot for dashboard reconnect");
		}

		static std::uint32_t RingUsed(const RingHeader* header, std::uint32_t capacity)
		{
			const std::uint32_t writeIndex = header->writeIndex.load(std::memory_order_acquire);
			const std::uint32_t readIndex = header->consumerReadIndex.load(std::memory_order_acquire);
			if (writeIndex >= readIndex)
				return writeIndex - readIndex;
			return capacity - (readIndex - writeIndex);
		}

		std::uint32_t RingFree() const
		{
			return m_capacity - RingUsed(m_header, m_capacity) - 1U;
		}

		void CopyToRing(std::uint32_t index, const std::uint8_t* bytes, std::uint32_t count)
		{
			if (count == 0)
				return;
			const std::uint32_t firstPart = (std::min)(count, m_capacity - index);
			std::memcpy(m_payload + index, bytes, firstPart);
			if (count > firstPart)
				std::memcpy(m_payload, bytes + firstPart, count - firstPart);
		}

		bool WritePendingValue(const std::string& key, const PendingValue& pending)
		{
			const std::uint16_t keyLen = static_cast<std::uint16_t>((std::min<std::size_t>)(key.size(), kKeyMax));
			std::uint16_t valueLen = 0;
			switch (pending.type)
			{
			case ValueType::Bool: valueLen = 1; break;
			case ValueType::Double: valueLen = 8; break;
			case ValueType::String: valueLen = static_cast<std::uint16_t>((std::min<std::size_t>)(pending.stringValue.size(), kStringMax)); break;
			case ValueType::StringArray:
			{
				std::size_t total = 1;
				const std::size_t count = (std::min<std::size_t>)(pending.stringArrayValue.size(), kStringArrayMaxCount);
				for (std::size_t i = 0; i < count; ++i)
				{
					total += 2;
					total += (std::min<std::size_t>)(pending.stringArrayValue[i].size(), kStringMax);
				}
				valueLen = static_cast<std::uint16_t>(total);
				break;
			}
			}

			const std::uint32_t totalBytes = static_cast<std::uint32_t>(sizeof(MessageHeader) + keyLen + valueLen);
			if (totalBytes >= m_capacity || RingFree() < totalBytes)
			{
				m_header->droppedCount.fetch_add(1, std::memory_order_acq_rel);
				return false;
			}

			const std::uint64_t seq = ++m_sequence;
			MessageHeader msg {};
			msg.messageBytes = static_cast<std::uint16_t>(totalBytes);
			msg.messageType = kMsgTypeUpsert;
			msg.valueType = static_cast<std::uint8_t>(pending.type);
			msg.seq = seq;
			msg.sourceTimestampUs = GetSteadyNowUs();
			msg.keyLen = keyLen;
			msg.valueLen = valueLen;

			std::uint32_t cursor = m_header->writeIndex.load(std::memory_order_acquire);
			CopyToRing(cursor, reinterpret_cast<const std::uint8_t*>(&msg), static_cast<std::uint32_t>(sizeof(msg)));
			cursor = (cursor + static_cast<std::uint32_t>(sizeof(msg))) % m_capacity;

			if (keyLen > 0)
			{
				CopyToRing(cursor, reinterpret_cast<const std::uint8_t*>(key.data()), keyLen);
				cursor = (cursor + keyLen) % m_capacity;
			}

			switch (pending.type)
			{
			case ValueType::Bool:
			{
				const std::uint8_t b = pending.boolValue ? 1U : 0U;
				CopyToRing(cursor, &b, 1U);
				cursor = (cursor + 1U) % m_capacity;
				break;
			}
			case ValueType::Double:
			{
				std::uint8_t dBytes[8] = {};
				std::memcpy(dBytes, &pending.doubleValue, 8);
				CopyToRing(cursor, dBytes, 8U);
				cursor = (cursor + 8U) % m_capacity;
				break;
			}
			case ValueType::String:
			{
				if (valueLen > 0)
				{
					CopyToRing(cursor, reinterpret_cast<const std::uint8_t*>(pending.stringValue.data()), valueLen);
					cursor = (cursor + valueLen) % m_capacity;
				}
				break;
			}
			case ValueType::StringArray:
			{
				const std::uint8_t count = static_cast<std::uint8_t>((std::min<std::size_t>)(pending.stringArrayValue.size(), kStringArrayMaxCount));
				CopyToRing(cursor, &count, 1U);
				cursor = (cursor + 1U) % m_capacity;
				for (std::size_t i = 0; i < count; ++i)
				{
					const std::uint16_t itemLen = static_cast<std::uint16_t>((std::min<std::size_t>)(pending.stringArrayValue[i].size(), kStringMax));
					CopyToRing(cursor, reinterpret_cast<const std::uint8_t*>(&itemLen), sizeof(itemLen));
					cursor = (cursor + static_cast<std::uint32_t>(sizeof(itemLen))) % m_capacity;
					if (itemLen > 0)
					{
						CopyToRing(cursor, reinterpret_cast<const std::uint8_t*>(pending.stringArrayValue[i].data()), itemLen);
						cursor = (cursor + itemLen) % m_capacity;
					}
				}
				break;
			}
			}

			m_header->writeIndex.store(cursor, std::memory_order_release);
			m_header->publishedSeq.store(seq, std::memory_order_release);
			return true;
		}

		void Close()
		{
			if (m_heartbeatEvent != nullptr)
			{
				CloseHandle(m_heartbeatEvent);
				m_heartbeatEvent = nullptr;
			}
			if (m_dataEvent != nullptr)
			{
				CloseHandle(m_dataEvent);
				m_dataEvent = nullptr;
			}
			if (m_view != nullptr)
			{
				UnmapViewOfFile(m_view);
				m_view = nullptr;
			}
			if (m_mapping != nullptr)
			{
				CloseHandle(m_mapping);
				m_mapping = nullptr;
			}
			m_header = nullptr;
			m_payload = nullptr;
			m_capacity = 0;
			m_sequence = 0;
			m_consumerWasActive = false;
		}

		std::wstring m_mappingName;
		std::wstring m_dataEventName;
		std::wstring m_heartbeatEventName;
		std::atomic<bool> m_running {false};
		HANDLE m_mapping = nullptr;
		void* m_view = nullptr;
		HANDLE m_dataEvent = nullptr;
		HANDLE m_heartbeatEvent = nullptr;
		RingHeader* m_header = nullptr;
		std::uint8_t* m_payload = nullptr;
		std::uint32_t m_capacity = 0;
		std::uint64_t m_sequence = 0;
		std::thread m_worker;
		std::mutex m_pendingMutex;
		std::unordered_map<std::string, PendingValue> m_pending;
		std::mutex m_retainedMutex;
		std::unordered_map<std::string, PendingValue> m_retained;
		bool m_consumerWasActive = false;
		bool m_loggedConsumerActive = false;
	};

	class DirectCommandSubscriber
	{
	public:
		explicit DirectCommandSubscriber(
			const std::wstring& mappingName,
			const std::wstring& dataEventName,
			const std::wstring& heartbeatEventName)
			: m_mappingName(mappingName)
			, m_dataEventName(dataEventName)
			, m_heartbeatEventName(heartbeatEventName)
		{
		}

		~DirectCommandSubscriber()
		{
			Stop();
		}

		bool Start()
		{
			if (m_running.load())
				return true;

			const std::size_t mappingBytes = sizeof(RingHeader) + kDefaultCapacityBytes;
			m_mapping = CreateFileMappingW(
				INVALID_HANDLE_VALUE,
				nullptr,
				PAGE_READWRITE,
				static_cast<DWORD>((static_cast<unsigned long long>(mappingBytes) >> 32U) & 0xFFFFFFFFULL),
				static_cast<DWORD>(static_cast<unsigned long long>(mappingBytes) & 0xFFFFFFFFULL),
				m_mappingName.c_str());
			if (m_mapping == nullptr)
				return false;

			m_view = MapViewOfFile(m_mapping, FILE_MAP_ALL_ACCESS, 0, 0, mappingBytes);
			if (m_view == nullptr)
			{
				Close();
				return false;
			}

			m_dataEvent = CreateEventW(nullptr, FALSE, FALSE, m_dataEventName.c_str());
			if (m_dataEvent == nullptr)
			{
				Close();
				return false;
			}

			m_heartbeatEvent = CreateEventW(nullptr, FALSE, FALSE, m_heartbeatEventName.c_str());
			if (m_heartbeatEvent == nullptr)
			{
				Close();
				return false;
			}

			auto* header = static_cast<RingHeader*>(m_view);
			const std::uint32_t payloadCapacity = static_cast<std::uint32_t>(mappingBytes - sizeof(RingHeader));
			// Read magic/version/capacity from raw bytes before any atomic access.
			std::uint32_t rawMagic = 0, rawVersion = 0, rawCapacity = 0;
			std::memcpy(&rawMagic,    static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, magic),         sizeof(rawMagic));
			std::memcpy(&rawVersion,  static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, version),       sizeof(rawVersion));
			std::memcpy(&rawCapacity, static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, capacityBytes), sizeof(rawCapacity));
			const bool needsInit =
				(rawMagic    != kMagic) ||
				(rawVersion  != kVersion) ||
				(rawCapacity != payloadCapacity);
			if (needsInit)
			{
				// Zero the payload region only, then placement-new the header.
				std::memset(static_cast<std::uint8_t*>(m_view) + sizeof(RingHeader), 0, kDefaultCapacityBytes);
				header = new (m_view) RingHeader{};
				header->magic         = kMagic;
				header->version       = kVersion;
				header->reserved0     = 0;
				header->capacityBytes = payloadCapacity;
				header->writeIndex.store(0, std::memory_order_release);
				header->readIndex.store(0, std::memory_order_release);
				header->publishedSeq.store(0, std::memory_order_release);
				header->droppedCount.store(0, std::memory_order_release);
				const std::uint64_t nowUs = GetSteadyNowUs();
				header->lastProducerHeartbeatUs.store(nowUs, std::memory_order_release);
				header->lastConsumerHeartbeatUs.store(nowUs, std::memory_order_release);
				header->consumerInstanceId.store(0, std::memory_order_release);
				header->consumerReadIndex.store(0, std::memory_order_release);
			}
			else
			{
				// SHM already initialized. Placement-new to construct atomics in this process.
				std::uint32_t savedWriteIndex = 0, savedReadIndex = 0,
				              savedPublishedSeq = 0, savedDroppedCount = 0,
				              savedConsumerInstanceId = 0, savedConsumerReadIndex = 0;
				std::uint64_t savedLastProducerUs = 0, savedLastConsumerUs = 0;
				std::memcpy(&savedWriteIndex,         static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, writeIndex),              sizeof(savedWriteIndex));
				std::memcpy(&savedReadIndex,          static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, readIndex),               sizeof(savedReadIndex));
				std::memcpy(&savedPublishedSeq,       static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, publishedSeq),            sizeof(savedPublishedSeq));
				std::memcpy(&savedDroppedCount,       static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, droppedCount),            sizeof(savedDroppedCount));
				std::memcpy(&savedLastProducerUs,     static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, lastProducerHeartbeatUs), sizeof(savedLastProducerUs));
				std::memcpy(&savedLastConsumerUs,     static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, lastConsumerHeartbeatUs), sizeof(savedLastConsumerUs));
				std::memcpy(&savedConsumerInstanceId, static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, consumerInstanceId),      sizeof(savedConsumerInstanceId));
				std::memcpy(&savedConsumerReadIndex,  static_cast<const std::uint8_t*>(m_view) + offsetof(RingHeader, consumerReadIndex),       sizeof(savedConsumerReadIndex));
				header = new (m_view) RingHeader{};
				header->magic         = rawMagic;
				header->version       = rawVersion;
				header->capacityBytes = rawCapacity;
				header->writeIndex.store(savedWriteIndex,         std::memory_order_relaxed);
				header->readIndex.store(savedReadIndex,           std::memory_order_relaxed);
				header->publishedSeq.store(savedPublishedSeq,     std::memory_order_relaxed);
				header->droppedCount.store(savedDroppedCount,     std::memory_order_relaxed);
				header->lastProducerHeartbeatUs.store(savedLastProducerUs,     std::memory_order_relaxed);
				header->lastConsumerHeartbeatUs.store(savedLastConsumerUs,     std::memory_order_relaxed);
				header->consumerInstanceId.store(savedConsumerInstanceId,     std::memory_order_relaxed);
				header->consumerReadIndex.store(savedConsumerReadIndex,       std::memory_order_relaxed);
			}

			m_header = header;
			m_payload = reinterpret_cast<std::uint8_t*>(header + 1);
			m_capacity = header->capacityBytes;
			m_instanceId = NextInstanceId();
			m_readCursor = header->writeIndex.load(std::memory_order_acquire);

			// Prime retained command values before returning so startup consumers
			// can observe operator-owned settings immediately.
			DrainPendingValues();

			m_running.store(true);
			m_worker = std::thread(&DirectCommandSubscriber::RunLoop, this);
			return true;
		}

		void Stop()
		{
			if (!m_running.load())
				return;

			m_running.store(false);
			if (m_dataEvent != nullptr)
				SetEvent(m_dataEvent);

			if (m_worker.joinable())
				m_worker.join();

			Close();
		}

		bool TryGetBoolean(const std::string& keyName, bool& value)
		{
			std::lock_guard<std::mutex> lock(m_valuesMutex);
			auto it = m_values.find(keyName);
			if (it == m_values.end() && keyName.compare(0, 15, "SmartDashboard/") == 0)
			{
				const std::string normalized = keyName.substr(15);
				it = m_values.find(normalized);
			}
			if (it == m_values.end() || it->second.type != ValueType::Bool)
				return false;
			value = it->second.boolValue;
			return true;
		}

		bool TryGetNumber(const std::string& keyName, double& value)
		{
			std::lock_guard<std::mutex> lock(m_valuesMutex);
			auto it = m_values.find(keyName);
			if (it == m_values.end() && keyName.compare(0, 15, "SmartDashboard/") == 0)
			{
				const std::string normalized = keyName.substr(15);
				it = m_values.find(normalized);
			}
			if (it == m_values.end() && keyName == "AutonTest")
			{
				it = m_values.find("Test/AutonTest");
				if (it != m_values.end())
					OutputDebugStringA("[DirectCommandSubscriber] AutonTest alias hit via Test/AutonTest\n");
			}
			if (it == m_values.end() && keyName == "TestMove")
			{
				it = m_values.find("Test/TestMove");
				if (it != m_values.end())
					OutputDebugStringA("[DirectCommandSubscriber] TestMove alias hit via Test/TestMove\n");
			}
			if (it == m_values.end())
			{
				if (keyName == "AutonTest")
					OutputDebugStringA("[DirectCommandSubscriber] AutonTest miss\n");
				if (keyName == "TestMove")
				{
					OutputDebugStringA("[DirectCommandSubscriber] TestMove miss\n");
					AppendTransportLogLine("[DirectCommandSubscriber] TestMove miss");
				}
				return false;
			}
			if (it->second.type == ValueType::Double)
			{
				value = it->second.doubleValue;
				if (keyName == "AutonTest")
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[DirectCommandSubscriber] AutonTest type=double value=%g\n", value);
					OutputDebugStringA(dbg);
				}
				if (keyName == "TestMove")
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[DirectCommandSubscriber] TestMove type=double value=%g\n", value);
					OutputDebugStringA(dbg);
					char dbgFile[256] = {};
					sprintf_s(dbgFile, "[DirectCommandSubscriber] TestMove type=double value=%g", value);
					AppendTransportLogLine(dbgFile);
				}
				return true;
			}
			if (it->second.type == ValueType::String)
			{
				value = atof(it->second.stringValue.c_str());
				if (keyName == "AutonTest")
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[DirectCommandSubscriber] AutonTest type=string raw='%s' parsed=%g\n", it->second.stringValue.c_str(), value);
					OutputDebugStringA(dbg);
				}
				if (keyName == "TestMove")
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[DirectCommandSubscriber] TestMove type=string raw='%s' parsed=%g\n", it->second.stringValue.c_str(), value);
					OutputDebugStringA(dbg);
					char dbgFile[256] = {};
					sprintf_s(dbgFile, "[DirectCommandSubscriber] TestMove type=string raw='%s' parsed=%g", it->second.stringValue.c_str(), value);
					AppendTransportLogLine(dbgFile);
				}
				return true;
			}
			if (it->second.type == ValueType::Bool)
			{
				value = it->second.boolValue ? 1.0 : 0.0;
				if (keyName == "AutonTest")
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[DirectCommandSubscriber] AutonTest type=bool value=%g\n", value);
					OutputDebugStringA(dbg);
				}
				if (keyName == "TestMove")
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[DirectCommandSubscriber] TestMove type=bool value=%g\n", value);
					OutputDebugStringA(dbg);
					char dbgFile[256] = {};
					sprintf_s(dbgFile, "[DirectCommandSubscriber] TestMove type=bool value=%g", value);
					AppendTransportLogLine(dbgFile);
				}
				return true;
			}
			return false;
		}

		bool TryGetString(const std::string& keyName, std::string& value)
		{
			std::lock_guard<std::mutex> lock(m_valuesMutex);
			auto it = m_values.find(keyName);
			if (it == m_values.end())
			{
				if (keyName == "AutonTest")
					it = m_values.find("Test/AutonTest");
				else if (keyName.compare(0, 15, "SmartDashboard/") == 0)
				{
					const std::string normalized = keyName.substr(15);
					it = m_values.find(normalized);
				}
			}
			if (it == m_values.end())
				return false;
			if (it->second.type == ValueType::String)
			{
				value = it->second.stringValue;
				return true;
			}
			if (it->second.type == ValueType::StringArray)
			{
				value.clear();
				for (size_t i = 0; i < it->second.stringArrayValue.size(); ++i)
				{
					if (i > 0)
						value += ",";
					value += it->second.stringArrayValue[i];
				}
				return true;
			}
			return false;
		}

	private:
		enum class ValueType : std::uint8_t
		{
			Bool = 1,
			Double = 2,
			String = 3,
			StringArray = 4
		};

		struct StoredValue
		{
			ValueType type = ValueType::String;
			bool boolValue = false;
			double doubleValue = 0.0;
			std::string stringValue;
			std::vector<std::string> stringArrayValue;
		};

		struct alignas(8) RingHeader
		{
			std::uint32_t magic;
			std::uint16_t version;
			std::uint16_t reserved0;
			std::uint32_t capacityBytes;
			std::atomic<std::uint32_t> writeIndex;
			std::atomic<std::uint32_t> readIndex;
			std::atomic<std::uint64_t> publishedSeq;
			std::atomic<std::uint64_t> droppedCount;
			std::atomic<std::uint64_t> lastProducerHeartbeatUs;
			std::atomic<std::uint64_t> lastConsumerHeartbeatUs;
			std::atomic<std::uint64_t> consumerInstanceId;
			std::atomic<std::uint32_t> consumerReadIndex;
		};

		struct alignas(8) MessageHeader
		{
			std::uint16_t messageBytes;
			std::uint8_t messageType;
			std::uint8_t valueType;
			std::uint64_t seq;
			std::uint64_t sourceTimestampUs;
			std::uint16_t keyLen;
			std::uint16_t valueLen;
		};

		static constexpr std::uint32_t kMagic = 0x53444442;
		static constexpr std::uint16_t kVersion = 1;
		static constexpr std::uint32_t kDefaultCapacityBytes = 1U << 20;
		// Must mirror the writer-side caps (DirectPublisherStub::kKeyMax / kStringMax).
		// These are the maximum byte lengths the producer is allowed to write;
		// anything larger from the ring is a protocol mismatch or corruption.
		static constexpr std::uint16_t kKeyMax = 128;
		static constexpr std::uint16_t kValueMax = 65535; // uint16_t ceiling; individual types capped further by protocol

		static std::uint64_t GetSteadyNowUs()
		{
			return static_cast<std::uint64_t>(
				std::chrono::duration_cast<std::chrono::microseconds>(
					std::chrono::steady_clock::now().time_since_epoch()).count());
		}

		static std::uint64_t NextInstanceId()
		{
			static std::atomic<std::uint64_t> nextId {1};
			const std::uint64_t localId = nextId.fetch_add(1, std::memory_order_relaxed);
			const std::uint64_t nowUs = GetSteadyNowUs();
			return (static_cast<std::uint64_t>(::GetCurrentProcessId()) << 32) ^ nowUs ^ localId;
		}

		void RunLoop()
		{
			while (m_running.load())
			{
				const DWORD waitResult = WaitForSingleObject(m_dataEvent, 50);
				if (waitResult == WAIT_OBJECT_0 || waitResult == WAIT_TIMEOUT)
				{
					try
					{
						DrainPendingValues();
					}
					catch (const std::exception& ex)
					{
						char dbg[512] = {};
						sprintf_s(dbg, "[DirectCommandSubscriber] DrainPendingValues threw: %s -- skipping drain\n", ex.what());
						OutputDebugStringA(dbg);
						AppendTransportLogLine(std::string("[DirectCommandSubscriber] DrainPendingValues threw: ") + ex.what() + " -- skipping drain");
					}
					catch (...)
					{
						OutputDebugStringA("[DirectCommandSubscriber] DrainPendingValues threw unknown exception -- skipping drain\n");
						AppendTransportLogLine("[DirectCommandSubscriber] DrainPendingValues threw unknown exception -- skipping drain");
					}
				}
				try
				{
					if (m_header != nullptr)
					{
						const std::uint64_t nowUs = GetSteadyNowUs();
						m_header->consumerInstanceId.store(m_instanceId, std::memory_order_release);
						m_header->lastConsumerHeartbeatUs.store(nowUs, std::memory_order_release);
						const std::uint64_t producerHeartbeatUs = m_header->lastProducerHeartbeatUs.load(std::memory_order_acquire);
						const bool producerActive =
							(producerHeartbeatUs != 0) &&
							(nowUs >= producerHeartbeatUs) &&
							((nowUs - producerHeartbeatUs) <= 500000ULL);
						if (producerActive != m_loggedProducerActive)
						{
							char dbg[256] = {};
							sprintf_s(
								dbg,
								"[DirectCommandSubscriber] producer active=%d heartbeat_delta_us=%llu\n",
								producerActive ? 1 : 0,
								static_cast<unsigned long long>(producerActive ? (nowUs - producerHeartbeatUs) : 0ULL));
							OutputDebugStringA(dbg);
							char dbgFile[256] = {};
							sprintf_s(
								dbgFile,
								"[DirectCommandSubscriber] producer active=%d heartbeat_delta_us=%llu",
								producerActive ? 1 : 0,
								static_cast<unsigned long long>(producerActive ? (nowUs - producerHeartbeatUs) : 0ULL));
							AppendTransportLogLine(dbgFile);
							m_loggedProducerActive = producerActive;
						}
					}
				}
				catch (const std::exception& ex)
				{
					char dbg[512] = {};
					sprintf_s(dbg, "[Subscriber] heartbeat block threw: %s\n", ex.what());
					OutputDebugStringA(dbg);
					AppendTransportLogLine(std::string("[Subscriber] heartbeat block threw: ") + ex.what());
				}
				catch (...)
				{
					OutputDebugStringA("[Subscriber] heartbeat block threw unknown exception\n");
					AppendTransportLogLine("[Subscriber] heartbeat block threw unknown exception");
				}
			}
		}

		void DrainPendingValues()
		{
			StoredValue value;
			std::string key;
			while (ReadNextValue(key, value))
			{
				if (key == "AutonTest" || key == "Test/AutonTest" || key == "TestMove" || key == "Test/TestMove")
				{
					char dbg[320] = {};
					switch (value.type)
					{
					case ValueType::Double:
						sprintf_s(dbg, "[DirectCommandSubscriber] rx key='%s' type=double value=%g\n", key.c_str(), value.doubleValue);
						break;
					case ValueType::String:
						sprintf_s(dbg, "[DirectCommandSubscriber] rx key='%s' type=string value='%s'\n", key.c_str(), value.stringValue.c_str());
						break;
					case ValueType::StringArray:
						sprintf_s(dbg, "[DirectCommandSubscriber] rx key='%s' type=string_array count=%zu\n", key.c_str(), value.stringArrayValue.size());
						break;
					case ValueType::Bool:
						sprintf_s(dbg, "[DirectCommandSubscriber] rx key='%s' type=bool value=%d\n", key.c_str(), value.boolValue ? 1 : 0);
						break;
					}
					OutputDebugStringA(dbg);
				}
				std::lock_guard<std::mutex> lock(m_valuesMutex);
				m_values[key] = value;
			}
		}

		void CopyFromRing(std::uint32_t index, std::uint8_t* outBytes, std::uint32_t count)
		{
			if (count == 0)
				return;
			const std::uint32_t firstPart = (std::min)(count, m_capacity - index);
			std::memcpy(outBytes, m_payload + index, firstPart);
			if (count > firstPart)
				std::memcpy(outBytes + firstPart, m_payload, count - firstPart);
		}

		bool ReadNextValue(std::string& outKey, StoredValue& outValue)
		{
			if (m_header == nullptr || m_payload == nullptr || m_capacity == 0)
				return false;

			const std::uint32_t readIndex = m_readCursor;
			const std::uint32_t writeIndex = m_header->writeIndex.load(std::memory_order_acquire);
			if (readIndex == writeIndex)
				return false;

			MessageHeader msg {};
			CopyFromRing(readIndex, reinterpret_cast<std::uint8_t*>(&msg), static_cast<std::uint32_t>(sizeof(msg)));
			if (msg.messageBytes < sizeof(MessageHeader))
			{
				char dbg[256] = {};
				sprintf_s(dbg, "[DirectCommandSubscriber] corrupt ring: messageBytes=%u < header size=%u -- resetting cursor\n",
					static_cast<unsigned>(msg.messageBytes), static_cast<unsigned>(sizeof(MessageHeader)));
				OutputDebugStringA(dbg);
				AppendTransportLogLine(std::string(dbg));
				m_readCursor = writeIndex;
				m_header->consumerReadIndex.store(writeIndex, std::memory_order_release);
				return false;
			}

			// Validate message size and field lengths before any allocation.
			// All values come from an untrusted remote process via shared memory.
			// A protocol mismatch or stale/garbage ring data can produce arbitrary
			// keyLen / valueLen values that would cause std::bad_alloc (or worse)
			// and kill the process via std::terminate on this background thread.
			const std::uint32_t expectedBytes =
				static_cast<std::uint32_t>(sizeof(MessageHeader)) +
				static_cast<std::uint32_t>(msg.keyLen) +
				static_cast<std::uint32_t>(msg.valueLen);
			if (msg.messageBytes > m_capacity ||
				msg.keyLen > kKeyMax ||
				static_cast<std::uint32_t>(msg.messageBytes) != expectedBytes)
			{
				char dbg[512] = {};
				sprintf_s(dbg,
					"[DirectCommandSubscriber] corrupt ring: messageBytes=%u expectedBytes=%u keyLen=%u valueLen=%u capacity=%u -- skipping message\n",
					static_cast<unsigned>(msg.messageBytes),
					static_cast<unsigned>(expectedBytes),
					static_cast<unsigned>(msg.keyLen),
					static_cast<unsigned>(msg.valueLen),
					static_cast<unsigned>(m_capacity));
				OutputDebugStringA(dbg);
				AppendTransportLogLine(std::string(dbg));
				// Advance past this malformed message using messageBytes (best effort);
				// if messageBytes itself is suspect fall back to resetting to writeIndex.
				if (msg.messageBytes > 0 && msg.messageBytes <= m_capacity)
				{
					const std::uint32_t next = (readIndex + msg.messageBytes) % m_capacity;
					m_readCursor = next;
					m_header->consumerReadIndex.store(next, std::memory_order_release);
				}
				else
				{
					m_readCursor = writeIndex;
					m_header->consumerReadIndex.store(writeIndex, std::memory_order_release);
				}
				return false;
			}

			std::uint32_t cursor = (readIndex + static_cast<std::uint32_t>(sizeof(MessageHeader))) % m_capacity;
			outKey.clear();
			if (msg.keyLen > 0)
			{
				outKey.resize(msg.keyLen);
				CopyFromRing(cursor, reinterpret_cast<std::uint8_t*>(&outKey[0]), msg.keyLen);
				cursor = (cursor + msg.keyLen) % m_capacity;
			}

			outValue = StoredValue{};
			outValue.type = static_cast<ValueType>(msg.valueType);
			switch (outValue.type)
			{
			case ValueType::Bool:
			{
				std::uint8_t b = 0;
				if (msg.valueLen >= 1)
					CopyFromRing(cursor, &b, 1);
				outValue.boolValue = (b != 0);
				break;
			}
			case ValueType::Double:
			{
				std::uint8_t db[8] = {};
				if (msg.valueLen >= 8)
				{
					CopyFromRing(cursor, db, 8);
					std::memcpy(&outValue.doubleValue, db, 8);
				}
				break;
			}
			case ValueType::String:
			{
				if (msg.valueLen > 0)
				{
					outValue.stringValue.resize(msg.valueLen);
					CopyFromRing(cursor, reinterpret_cast<std::uint8_t*>(&outValue.stringValue[0]), msg.valueLen);
				}
				break;
			}
			case ValueType::StringArray:
			{
				if (msg.valueLen > 0)
				{
					std::vector<std::uint8_t> bytes(msg.valueLen);
					CopyFromRing(cursor, bytes.data(), msg.valueLen);
					std::size_t offset = 0;
					const std::uint8_t count = bytes[offset++];
					outValue.stringArrayValue.clear();
					outValue.stringArrayValue.reserve(count);
					for (std::uint8_t i = 0; i < count && offset + 2 <= bytes.size(); ++i)
					{
						std::uint16_t itemLen = 0;
						std::memcpy(&itemLen, bytes.data() + offset, sizeof(itemLen));
						offset += 2;
						const std::size_t boundedLen = (std::min<std::size_t>)(itemLen, bytes.size() - offset);
						outValue.stringArrayValue.push_back(std::string(reinterpret_cast<const char*>(bytes.data() + offset), boundedLen));
						offset += boundedLen;
					}
				}
				break;
			}
			}

			cursor = (readIndex + msg.messageBytes) % m_capacity;
			m_readCursor = cursor;
			m_header->consumerReadIndex.store(cursor, std::memory_order_release);
			return true;
		}

		void Close()
		{
			if (m_heartbeatEvent != nullptr)
			{
				CloseHandle(m_heartbeatEvent);
				m_heartbeatEvent = nullptr;
			}
			if (m_dataEvent != nullptr)
			{
				CloseHandle(m_dataEvent);
				m_dataEvent = nullptr;
			}
			if (m_view != nullptr)
			{
				UnmapViewOfFile(m_view);
				m_view = nullptr;
			}
			if (m_mapping != nullptr)
			{
				CloseHandle(m_mapping);
				m_mapping = nullptr;
			}
			m_header = nullptr;
			m_payload = nullptr;
			m_capacity = 0;
			m_readCursor = 0;
			m_instanceId = 0;
			std::lock_guard<std::mutex> lock(m_valuesMutex);
			m_values.clear();
		}

		std::wstring m_mappingName;
		std::wstring m_dataEventName;
		std::wstring m_heartbeatEventName;
		std::atomic<bool> m_running {false};
		HANDLE m_mapping = nullptr;
		void* m_view = nullptr;
		HANDLE m_dataEvent = nullptr;
		HANDLE m_heartbeatEvent = nullptr;
		RingHeader* m_header = nullptr;
		std::uint8_t* m_payload = nullptr;
		std::uint32_t m_capacity = 0;
		std::thread m_worker;
		std::mutex m_valuesMutex;
		std::unordered_map<std::string, StoredValue> m_values;
		std::uint32_t m_readCursor = 0;
		std::uint64_t m_instanceId = 0;
		bool m_loggedProducerActive = false;
	};

	class LegacySmartDashboardBackend final : public IConnectionBackend
	{
public:
		void Initialize() override
		{
			SmartDashboard::SetConnectionMode(SmartDashboardConnectionMode::eLegacySmartDashboard);
			if (!SmartDashboard::is_initialized())
			{
				try
				{
					SmartDashboard::init();
				}
				catch (const std::exception& e)
				{
					char buf[256];
					sprintf_s(buf, "[Transport] Legacy SmartDashboard init failed (NT socket error): %s\n", e.what());
					OutputDebugStringA(buf);
					// Backend failed to bind/listen; app continues without NT transport.
					return;
				}
			}
			OutputDebugStringW(L"[Transport] Legacy SmartDashboard backend initialized\n");
		}
		void Shutdown() override
		{
			// Historically shutdown was unstable in this app flow; preserve existing behavior.
			OutputDebugStringW(L"[Transport] Legacy SmartDashboard backend shutdown requested\n");
		}
		const wchar_t* GetBackendName() const override
		{
			return L"Legacy SmartDashboard";
		}
	};

	class DirectConnectBackend final : public IConnectionBackend
		, public SmartDashboardDirectPublishSink
		, public SmartDashboardDirectQuerySource
	{
	public:
		DirectConnectBackend()
			: m_publisher(std::make_unique<DirectPublisherStub>(
				L"Local\\SmartDashboard.Direct.Buffer",
				L"Local\\SmartDashboard.Direct.DataAvailable",
				L"Local\\SmartDashboard.Direct.Heartbeat"))
			, m_commandSubscriber(std::make_unique<DirectCommandSubscriber>(
				L"Local\\SmartDashboard.Direct.Command.Buffer",
				L"Local\\SmartDashboard.Direct.Command.DataAvailable",
				L"Local\\SmartDashboard.Direct.Command.Heartbeat"))
		{
		}

		void Initialize() override
		{
			SmartDashboard::SetConnectionMode(SmartDashboardConnectionMode::eDirectConnect);
			SmartDashboard::SetDirectPublishSink(this);
			SmartDashboard::SetDirectQuerySource(this);
			m_running = (m_publisher && m_publisher->Start());
			const bool commandRunning = (m_commandSubscriber && m_commandSubscriber->Start());
			if (m_running)
			{
				if (commandRunning)
					OutputDebugStringW(L"[Transport] Direct Connect backend initialized (telemetry + command channels active)\n");
				else
					OutputDebugStringW(L"[Transport] Direct Connect backend initialized (telemetry active, command channel inactive)\n");
			}
			else
				OutputDebugStringW(L"[Transport] Direct Connect backend failed to start; no direct stream available\n");
		}
		void Shutdown() override
		{
			SmartDashboard::ClearDirectPublishSink();
			SmartDashboard::ClearDirectQuerySource();
			SmartDashboard::SetConnectionMode(SmartDashboardConnectionMode::eDirectConnect);
			if (m_commandSubscriber)
				m_commandSubscriber->Stop();
			if (m_publisher)
				m_publisher->Stop();
			m_running = false;
			OutputDebugStringW(L"[Transport] Direct Connect backend shutdown requested\n");
		}
		const wchar_t* GetBackendName() const override
		{
			return m_running ? L"Direct Connect" : L"Direct Connect (inactive)";
		}

		void PublishBoolean(const std::string& keyName, bool value) override
		{
			if (m_publisher && m_running)
				m_publisher->PublishBool(keyName, value);
		}
		void PublishNumber(const std::string& keyName, double value) override
		{
			if (m_publisher && m_running)
				m_publisher->PublishDouble(keyName, value);
		}
		void PublishString(const std::string& keyName, const std::string& value) override
		{
			if (m_publisher && m_running)
				m_publisher->PublishString(keyName, value);
		}

		void PublishStringArray(const std::string& keyName, const std::vector<std::string>& values) override
		{
			if (m_publisher && m_running)
				m_publisher->PublishStringArray(keyName, values);
		}

		bool TryGetBoolean(const std::string& keyName, bool& value) override
		{
			if (m_commandSubscriber && m_commandSubscriber->TryGetBoolean(keyName, value))
				return true;
			if (!m_publisher)
				return false;
			return m_publisher->TryGetBoolean(keyName, value);
		}

		bool TryGetNumber(const std::string& keyName, double& value) override
		{
			if (m_commandSubscriber && m_commandSubscriber->TryGetNumber(keyName, value))
			{
				if (keyName == "TestMove" || keyName == "Test/TestMove")
				{
					char dbg[256] = {};
					sprintf_s(dbg, "[DirectConnectBackend] TryGetNumber source=command key='%s' value=%g", keyName.c_str(), value);
					AppendTransportLogLine(dbg);
				}
				return true;
			}
			if (!m_publisher)
				return false;
			const bool ok = m_publisher->TryGetNumber(keyName, value);
			if (keyName == "TestMove" || keyName == "Test/TestMove")
			{
				char dbg[256] = {};
				sprintf_s(dbg, "[DirectConnectBackend] TryGetNumber source=publisher key='%s' ok=%d value=%g", keyName.c_str(), ok ? 1 : 0, ok ? value : 0.0);
				AppendTransportLogLine(dbg);
			}
			return ok;
		}

		bool TryGetString(const std::string& keyName, std::string& value) override
		{
			if (m_commandSubscriber && m_commandSubscriber->TryGetString(keyName, value))
				return true;
			if (!m_publisher)
				return false;
			return m_publisher->TryGetString(keyName, value);
		}

	private:

		std::unique_ptr<IDirectPublisher> m_publisher;
		std::unique_ptr<DirectCommandSubscriber> m_commandSubscriber;
		bool m_running = false;
	};

	// Ian: NT4Backend is the NT4 WebSocket server backend used by Shuffleboard, Glass,
	// and any other NT4-speaking dashboard.
	// It implements SmartDashboardDirectPublishSink so that all SmartDashboard::Put*()
	// calls route through the NT4 server instead of through the legacy NT2 path.
	// It also implements SmartDashboardDirectQuerySource so that the simulator can
	// read back values written by dashboard clients (chooser selections, TestMove, etc.).
	// The query path delegates to NT4Server::TryGet*, which reads from the retained
	// value cache — same cache that HandleClientValueUpdate populates when a dashboard
	// client writes a value back.
	class NT4Backend final : public IConnectionBackend
		, public SmartDashboardDirectPublishSink
		, public SmartDashboardDirectQuerySource
	{
	public:
		void Initialize() override
		{
			SmartDashboard::SetConnectionMode(SmartDashboardConnectionMode::eNetworkTablesV4);
			SmartDashboard::SetDirectPublishSink(this);
			SmartDashboard::SetDirectQuerySource(this);
			m_running = m_nt4Server.Start(5810);
			if (m_running)
			{
				OutputDebugStringW(L"[Transport] NT4 backend initialized on port 5810\n");

				// Ian: Start the MJPEG camera streaming server on port 1181.
				// This runs alongside the NT4 server so dashboards can discover
				// the camera stream via CameraPublisher NT4 keys and connect to
				// the MJPEG stream over plain HTTP.
				//
				// LESSON LEARNED: The MjpegServer must be started BEFORE publishing
				// the CameraPublisher keys, because a fast dashboard might try to
				// connect to the stream URL immediately upon seeing the NT4 key.
				StartCameraStream();
			}
			else
			{
				OutputDebugStringW(L"[Transport] NT4 backend FAILED to start on port 5810\n");
			}
		}
		void Shutdown() override
		{
			StopCameraStream();
			SmartDashboard::ClearDirectPublishSink();
			SmartDashboard::ClearDirectQuerySource();
			m_nt4Server.Stop();
			m_running = false;
			OutputDebugStringW(L"[Transport] NT4 backend shutdown\n");
		}
		const wchar_t* GetBackendName() const override
		{
			return m_running ? L"NetworkTables V4" : L"NetworkTables V4 (inactive)";
		}

		// --- SmartDashboardDirectPublishSink ---
		// Ian: These convert flat SmartDashboard keys to NT4 topic paths.
		// NT4 dashboards expect topics under /SmartDashboard/<key>.
		void PublishBoolean(const std::string& keyName, bool value) override
		{
			if (m_running)
				m_nt4Server.PublishBoolean(ToNT4Path(keyName), value);
		}
		void PublishNumber(const std::string& keyName, double value) override
		{
			if (m_running)
				m_nt4Server.PublishDouble(ToNT4Path(keyName), value);
		}
		void PublishString(const std::string& keyName, const std::string& value) override
		{
			if (m_running)
				m_nt4Server.PublishString(ToNT4Path(keyName), value);
		}
		void PublishStringArray(const std::string& keyName, const std::vector<std::string>& values) override
		{
			if (m_running)
				m_nt4Server.PublishStringArray(ToNT4Path(keyName), values);
		}

		// --- SmartDashboardDirectQuerySource ---
		// Ian: The simulator reads values with flat keys like "TestMove" or
		// "Test/Auton_Selection/AutoChooser/selected". The NT4 retained cache
		// stores them under "/SmartDashboard/TestMove" etc. We try the key as-is
		// first (in case something stored it without the prefix), then with the
		// NT4 prefix. This matches the same key normalization the publish path does.
		bool TryGetBoolean(const std::string& keyName, bool& value) override
		{
			if (!m_running) return false;
			if (m_nt4Server.TryGetBoolean(keyName, value)) return true;
			return m_nt4Server.TryGetBoolean(ToNT4Path(keyName), value);
		}
		bool TryGetNumber(const std::string& keyName, double& value) override
		{
			if (!m_running) return false;
			if (m_nt4Server.TryGetNumber(keyName, value)) return true;
			return m_nt4Server.TryGetNumber(ToNT4Path(keyName), value);
		}
		bool TryGetString(const std::string& keyName, std::string& value) override
		{
			if (!m_running) return false;
			if (m_nt4Server.TryGetString(keyName, value)) return true;
			return m_nt4Server.TryGetString(ToNT4Path(keyName), value);
		}

	private:
		NT4Server m_nt4Server;
		bool m_running = false;

		// Ian: Camera streaming infrastructure.
		// MjpegServer owns the HTTP server on port 1181.
		// SimCameraSource generates frames on a worker thread and pushes them to MjpegServer.
		// Both are created/destroyed alongside the NT4 server.
		//
		// Ian: LESSON LEARNED — these must be unique_ptrs, not inline members, because
		// MjpegServer inherits from ix::SocketServer which is not movable/copyable.
		// We create them on demand in StartCameraStream() and destroy in StopCameraStream().
		std::unique_ptr<MjpegServer> m_mjpegServer;
		std::unique_ptr<SimCameraSource> m_cameraSource;

		void StartCameraStream()
		{
			// Ian: Create and start the MJPEG server on port 1181.
			m_mjpegServer = std::make_unique<MjpegServer>(1181, "0.0.0.0");
			auto [listenOk, listenErr] = m_mjpegServer->listen();
			if (!listenOk)
			{
				OutputDebugStringA(("[Transport] MJPEG server failed to listen: " + listenErr + "\n").c_str());
				m_mjpegServer.reset();
				return;
			}
			m_mjpegServer->start();
			OutputDebugStringW(L"[Transport] MJPEG server started on port 1181\n");

			// Ian: Create and start the camera frame source.
			// 320x240 @ 15fps is the standard FRC camera resolution/framerate.
			m_cameraSource = std::make_unique<SimCameraSource>();
			m_cameraSource->Start(m_mjpegServer.get(), 320, 240, 15);

			// Ian: Publish CameraPublisher discovery keys via NT4.
			// SmartDashboard's CameraPublisherDiscovery watches for:
			//   /CameraPublisher/{name}/streams  (string array with "mjpg:" prefix)
			//   /CameraPublisher/{name}/source   (string, description)
			//   /CameraPublisher/{name}/connected (boolean)
			//
			// The "streams" key is the critical one — it contains the URL(s) that
			// the MJPEG client connects to.  The "mjpg:" prefix tells the dashboard
			// this is an MJPEG stream (vs. other formats like H.264).
			//
			// Ian: We publish 127.0.0.1 because the simulator and dashboard run on
			// the same machine.  If we later need LAN access, we'd publish the
			// machine's actual IP or 0.0.0.0 and let the dashboard resolve it.
			m_nt4Server.PublishStringArray(
				"/CameraPublisher/SimCamera/streams",
				{"mjpg:http://127.0.0.1:1181/?action=stream"});
			m_nt4Server.PublishString(
				"/CameraPublisher/SimCamera/source",
				"Simulator Synthetic Camera");
			m_nt4Server.PublishBoolean(
				"/CameraPublisher/SimCamera/connected",
				true);

			OutputDebugStringW(L"[Transport] SimCamera published on NT4 for auto-discovery\n");
		}

		void StopCameraStream()
		{
			// Ian: Graceful camera shutdown sequence:
			//   1. Signal the MJPEG server to stop — this wakes all client handler
			//      threads immediately via m_stopping + notify_all on the condition
			//      variable, so they begin unwinding instead of blocking for up to
			//      1 second on the next wait_for timeout.
			//   2. Stop the camera source — joins the frame-producer worker thread
			//      so no more PushFrame() calls arrive.
			//   3. Stop the MJPEG server — joins client handler threads (which are
			//      already unwinding from step 1) and closes the listen socket.
			//
			// Without step 1 the server's stop() would block while client threads
			// sit in wait_for(), causing up to 1 second of shutdown delay per
			// connected client — and leaving ghost sockets if the process exits
			// before the threads finish.
			if (m_mjpegServer)
			{
				m_mjpegServer->SignalStop();
			}
			if (m_cameraSource)
			{
				m_cameraSource->Stop();
				m_cameraSource.reset();
			}
			if (m_mjpegServer)
			{
				m_mjpegServer->stop();
				m_mjpegServer.reset();
			}

			// Ian: Publish disconnected state so dashboards know the camera is gone.
			// (The NT4 server is still running at this point.)
			if (m_running)
			{
				m_nt4Server.PublishBoolean(
					"/CameraPublisher/SimCamera/connected",
					false);
			}

			OutputDebugStringW(L"[Transport] Camera stream stopped\n");
		}

		// Ian: Convert a flat SmartDashboard key to an NT4 topic path.
		// If the key already starts with '/', use it as-is.
		// Otherwise prefix with "/SmartDashboard/".
		static std::string ToNT4Path(const std::string& key)
		{
			if (!key.empty() && key[0] == '/')
				return key;
			return "/SmartDashboard/" + key;
		}
	};
}

const wchar_t* GetConnectionModeName(ConnectionMode mode)
{
	switch (mode)
	{
	case ConnectionMode::eLegacySmartDashboard:
		return L"Legacy SmartDashboard";
	case ConnectionMode::eDirectConnect:
		return L"Direct Connect";
	case ConnectionMode::eNetworkTablesV4:
		return L"NetworkTables V4";
	case ConnectionMode::eNativeLink:
		return L"Native Link";
	default:
		return L"Unknown";
	}
}

DashboardTransportRouter::DashboardTransportRouter() = default;

void DashboardTransportRouter::Initialize(ConnectionMode initial_mode)
{
	if (m_is_initialized)
	{
		SetMode(initial_mode);
		return;
	}

	m_mode = initial_mode;
	EnsureBackend();
	if (m_backend)
	{
		m_backend->Initialize();
	}
	else
	{
		OutputDebugStringW(L"[Transport] WARNING: m_backend is null after EnsureBackend\n");
	}
	m_is_initialized = true;
}

void DashboardTransportRouter::SetMode(ConnectionMode mode)
{
	if (!m_is_initialized)
	{
		Initialize(mode);
		return;
	}

	if (m_mode == mode && m_backend)
	{
		return;
	}

	if (UsesLegacyTransportPath(m_mode) && UsesLegacyTransportPath(mode))
	{
		// Ian: Native Link must *not* ride this legacy-backend reuse shortcut.
		// The transport smoke initializes directly into Native Link and works, but
		// the real DriverStation app often switches modes at runtime. If Native Link
		// is treated as "legacy enough" here, the UI can say "Native Link" while the
		// old backend keeps running underneath, which looks exactly like a transport
		// that never connects.
		m_mode = mode;
		std::wstring message = L"[Transport] Mode switch retained existing legacy backend to avoid NT reinit race: ";
		message += GetConnectionModeName(mode);
		message += L"\n";
		OutputDebugStringW(message.c_str());
		return;
	}

	if (m_backend)
		m_backend->Shutdown();

	m_mode = mode;
	EnsureBackend();
	if (m_backend)
		m_backend->Initialize();
}

bool DashboardTransportRouter::UsesLegacyTransportPath(ConnectionMode mode)
{
	// Ian: NT4 is not a legacy transport — it has its own WebSocket server backend.
	// Only Legacy SmartDashboard and Direct Connect share the legacy NetworkTables
	// path and can reuse each other's backend without teardown.
	return (mode == ConnectionMode::eLegacySmartDashboard) ||
		(mode == ConnectionMode::eDirectConnect);
}

ConnectionMode DashboardTransportRouter::GetMode() const
{
	return m_mode;
}

const wchar_t* DashboardTransportRouter::GetActiveBackendName() const
{
	if (m_backend)
		return m_backend->GetBackendName();
	return L"None";
}

void DashboardTransportRouter::Shutdown()
{
	if (m_backend)
		m_backend->Shutdown();
}

void DashboardTransportRouter::EnsureBackend()
{
	switch (m_mode)
	{
	case ConnectionMode::eLegacySmartDashboard:
		m_backend = std::make_unique<LegacySmartDashboardBackend>();
		break;
	case ConnectionMode::eDirectConnect:
		m_backend = std::make_unique<DirectConnectBackend>();
		break;
	case ConnectionMode::eNetworkTablesV4:
		m_backend = std::make_unique<NT4Backend>();
		break;
	case ConnectionMode::eNativeLink:
		// Ian: Keep Native Link as a separate backend beside Direct instead of
		// folding it into the existing path. Direct is our known-good 1:1
		// behavioral reference, and this separation lets us compare parity first
		// before layering on the semantics Direct never had (session reset,
		// retained snapshot, multi-client fan-out, lease ownership).
		m_backend = NativeLink::CreateNativeLinkBackend();
		break;
	default:
		m_backend = std::make_unique<LegacySmartDashboardBackend>();
		break;
	}
}
