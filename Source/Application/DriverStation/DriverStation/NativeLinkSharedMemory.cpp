#include "stdafx.h"
#include "NativeLinkSharedMemory.h"

#include <Windows.h>

#include <algorithm>
#include <chrono>
#include <cstring>

namespace NativeLink::detail
{
	namespace
	{
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
	}

	AutoHandle::~AutoHandle()
	{
		Reset();
	}

	void AutoHandle::Reset(HANDLE newHandle)
	{
		if (handle)
			CloseHandle(handle);
		handle = newHandle;
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

	SharedMessage BuildSharedMessage(const UpdateEnvelope& envelope)
	{
		// Ian: Keep the SHM wire-shape conversion in one helper so later TCP work
		// can reuse the same authority-side envelope rules without copying ad hoc
		// message packing logic back into the app/runtime classes.
		SharedMessage message {};
		message.size = sizeof(SharedMessage);
		message.topicId = static_cast<std::uint32_t>(envelope.topicId);
		message.deliveryKind = static_cast<std::uint32_t>(envelope.deliveryKind);
		message.valueType = static_cast<std::uint32_t>(envelope.value.type);
		message.serverSessionId = envelope.serverSessionId;
		message.serverSequence = envelope.serverSequence;
		CopyUtf8(message.topicPath, sizeof(message.topicPath), envelope.topicPath);
		CopyUtf8(message.sourceClientId, sizeof(message.sourceClientId), envelope.sourceClientId);

		const std::vector<unsigned char> payload = SerializeValue(envelope.value);
		const std::size_t payloadBytes = (std::min<std::size_t>)(payload.size(), sizeof(message.payload));
		if (payloadBytes > 0)
			memcpy(message.payload, payload.data(), payloadBytes);
		message.flags = static_cast<std::uint64_t>(payloadBytes);
		return message;
	}

	SharedMessage BuildClientWriteMessage(const std::string& topicPath, const std::string& sourceClientId, const TopicValue& value)
	{
		UpdateEnvelope envelope;
		envelope.topicPath = topicPath;
		envelope.sourceClientId = sourceClientId;
		envelope.value = value;
		return BuildSharedMessage(envelope);
	}

	UpdateEnvelope SharedMessageToUpdateEnvelope(const SharedMessage& message)
	{
		// Ian: This reverse conversion is the matching semantic boundary for the
		// test client path. If value/provenance mapping changes later, update it in
		// one place so SHM diagnostics and future carriers stay comparable.
		UpdateEnvelope event;
		event.serverSessionId = message.serverSessionId;
		event.serverSequence = message.serverSequence;
		event.topicId = message.topicId;
		event.topicPath = ReadUtf8(message.topicPath, sizeof(message.topicPath));
		event.sourceClientId = ReadUtf8(message.sourceClientId, sizeof(message.sourceClientId));
		event.deliveryKind = static_cast<DeliveryKind>(message.deliveryKind);
		event.rejectionReason = WriteRejectReason::None;
		DeserializeValue(static_cast<ValueType>(message.valueType), message.payload, static_cast<std::size_t>(message.flags), event.value);
		return event;
	}
}
