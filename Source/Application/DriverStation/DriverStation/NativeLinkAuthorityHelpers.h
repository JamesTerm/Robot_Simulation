#pragma once

#include "NativeLink.h"
#include "NativeLinkSharedMemory.h"

#include <cstdint>
#include <string>

namespace NativeLink::detail
{
	extern const char* const kServerClientId;

	void RegisterDefaultTopics(Core& core);
	UpdateEnvelope SnapshotEventToEnvelope(const SnapshotEvent& event, std::uint64_t serverSessionId);
	bool BuildLiveEnvelopeForTopic(
		const Core& core,
		const std::string& topicPath,
		const std::string& sourceClientId,
		std::uint64_t serverSequence,
		UpdateEnvelope& outEnvelope);
	bool PublishServerValue(Core& core, const std::string& topicPath, const TopicValue& value, UpdateEnvelope& outEnvelope);
}
