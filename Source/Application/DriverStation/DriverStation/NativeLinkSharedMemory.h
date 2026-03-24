#pragma once

#include "NativeLink.h"

#include <Windows.h>

#include "../../../../plugins/NativeLinkTransport/include/native_link_ipc_protocol.h"

#include <cstdint>
#include <string>
#include <vector>

namespace NativeLink::detail
{
	using SharedMessage = sd::nativelink::SharedMessage;
	using SharedClientSlot = sd::nativelink::SharedClientSlot;
	using SharedState = sd::nativelink::SharedState;

	constexpr std::uint32_t kSharedMagic = sd::nativelink::kSharedMagic;
	constexpr std::uint32_t kSharedVersion = sd::nativelink::kSharedVersion;
	constexpr std::uint32_t kMaxClients = sd::nativelink::kMaxClients;
	constexpr std::uint32_t kMaxMessages = sd::nativelink::kMaxMessages;
	constexpr std::uint32_t kMaxPayloadBytes = sd::nativelink::kMaxPayloadBytes;
	constexpr std::uint32_t kSnapshotStartTopicId = sd::nativelink::kSnapshotStartTopicId;
	constexpr std::uint32_t kSnapshotEndTopicId = sd::nativelink::kSnapshotEndTopicId;
	constexpr std::uint32_t kLiveBeginTopicId = sd::nativelink::kLiveBeginTopicId;

	struct AutoHandle
	{
		HANDLE handle = nullptr;

		~AutoHandle();
		void Reset(HANDLE newHandle = nullptr);
	};

	std::uint64_t GetSteadyNowUs();
	void CopyUtf8(char* dest, std::size_t destCount, const std::string& value);
	std::string ReadUtf8(const char* src, std::size_t srcCount);
	std::wstring MakeKernelObjectName(const wchar_t* prefix, const std::string& channelId);

	std::vector<unsigned char> SerializeValue(const TopicValue& value);
	bool DeserializeValue(ValueType type, const unsigned char* bytes, std::size_t byteCount, TopicValue& outValue);

	SharedMessage BuildSharedMessage(const UpdateEnvelope& envelope);
	SharedMessage BuildClientWriteMessage(const std::string& topicPath, const std::string& sourceClientId, const TopicValue& value);
	UpdateEnvelope SharedMessageToUpdateEnvelope(const SharedMessage& message);
}
