#include "stdafx.h"
#include "NativeLinkAuthorityHelpers.h"

namespace NativeLink::detail
{
	const char* const kServerClientId = "server";

	namespace
	{
		TopicDescriptor MakeStateTopic(
			const std::string& topicPath,
			ValueType valueType,
			WriterPolicy writerPolicy)
		{
			TopicDescriptor topic;
			topic.topicPath = topicPath;
			topic.topicKind = TopicKind::State;
			topic.valueType = valueType;
			topic.retentionMode = RetentionMode::LatestValue;
			topic.replayOnSubscribe = true;
			topic.writerPolicy = writerPolicy;
			return topic;
		}
	}

	void RegisterDefaultTopics(Core& core)
	{
		if (core.IsTopicRegistered("Test/Auton_Selection/AutoChooser/selected"))
			return;

		// Ian: This list is intentionally minimal.  Only the few topics that need
		// special writer policies (LeaseSingleWriter for operator-owned controls,
		// StringArray for the chooser options blob) must be pre-declared here.
		// All other robot-code keys (Velocity, Heading, wheel velocities, etc.) are
		// auto-registered on first server-originated write in Core::PublishInternal
		// when allowServerOnly=true.  Do NOT add every TeleAutonV2 key here — the
		// auto-register path is the correct place for those, and duplicating them
		// here would hide the auto-register logic and make both paths drift.
		//
		// Ian: Keep the first reference authority seeded with the same visible keys
		// we already validated in Direct so carrier work changes transport behavior,
		// not the application-level expectations students are trying to compare.
		core.RegisterTopic(MakeStateTopic("Test/Auton_Selection/AutoChooser/options", ValueType::StringArray, WriterPolicy::ServerOnly));
		core.RegisterTopic(MakeStateTopic("Test/Auton_Selection/AutoChooser/default", ValueType::String, WriterPolicy::ServerOnly));
		core.RegisterTopic(MakeStateTopic("Test/Auton_Selection/AutoChooser/active", ValueType::String, WriterPolicy::ServerOnly));
		core.RegisterTopic(MakeStateTopic("Test/Auton_Selection/AutoChooser/selected", ValueType::String, WriterPolicy::LeaseSingleWriter));
		core.RegisterTopic(MakeStateTopic("TestMove", ValueType::Double, WriterPolicy::LeaseSingleWriter));
		core.RegisterTopic(MakeStateTopic("Timer", ValueType::Double, WriterPolicy::ServerOnly));
		core.RegisterTopic(MakeStateTopic("Y_ft", ValueType::Double, WriterPolicy::ServerOnly));

		core.PublishFromServer("Test/Auton_Selection/AutoChooser/options", TopicValue::StringArray(std::vector<std::string>{ "Do Nothing", "Just Move Forward", "Just Rotate", "Move Rotate Sequence", "Box Waypoints", "Smart Waypoints" }));
		core.PublishFromServer("Test/Auton_Selection/AutoChooser/default", TopicValue::String("Do Nothing"));
		core.PublishFromServer("Test/Auton_Selection/AutoChooser/active", TopicValue::String("Do Nothing"));
		core.PublishFromServer("Test/Auton_Selection/AutoChooser/selected", TopicValue::String("Do Nothing"));
		core.PublishFromServer("TestMove", TopicValue::Double(0.0));
		core.PublishFromServer("Timer", TopicValue::Double(0.0));
		core.PublishFromServer("Y_ft", TopicValue::Double(0.0));
	}

	UpdateEnvelope SnapshotEventToEnvelope(const SnapshotEvent& event, std::uint64_t serverSessionId)
	{
		if (event.kind == SnapshotEventKind::Update && event.hasUpdate)
			return event.update;

		// Ian: Snapshot sentinels are converted here so the authority keeps one
		// consistent semantic story regardless of carrier. SHM, TCP, and future
		// adapters should all inherit the same descriptor/state/live boundaries.
		UpdateEnvelope envelope;
		envelope.serverSessionId = serverSessionId;
		envelope.sourceClientId = kServerClientId;
		envelope.value = TopicValue::String(std::string());
		envelope.deliveryKind = DeliveryKind::LiveEvent;

		switch (event.kind)
		{
		case SnapshotEventKind::DescriptorSnapshotBegin:
			envelope.topicId = detail::kSnapshotStartTopicId;
			envelope.topicPath = "__snapshot_begin__";
			break;
		case SnapshotEventKind::DescriptorSnapshotEnd:
			envelope.topicId = detail::kSnapshotEndTopicId;
			envelope.topicPath = "__descriptor_end__";
			break;
		case SnapshotEventKind::StateSnapshotBegin:
			envelope.topicId = detail::kSnapshotStartTopicId;
			envelope.topicPath = "__state_begin__";
			break;
		case SnapshotEventKind::StateSnapshotEnd:
			envelope.topicId = detail::kSnapshotEndTopicId;
			envelope.topicPath = "__state_end__";
			break;
		case SnapshotEventKind::LiveBegin:
			envelope.topicId = detail::kLiveBeginTopicId;
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

		return envelope;
	}

	bool BuildLiveEnvelopeForTopic(
		const Core& core,
		const std::string& topicPath,
		const std::string& sourceClientId,
		std::uint64_t serverSequence,
		UpdateEnvelope& outEnvelope)
	{
		const Core::TopicRuntime* topic = core.LookupTopic(topicPath);
		TopicValue latest;
		if (!topic || !core.TryGetLatestValue(topicPath, latest))
			return false;

		outEnvelope.serverSessionId = core.GetServerSessionId();
		outEnvelope.serverSequence = serverSequence;
		outEnvelope.topicId = topic->topicId;
		outEnvelope.topicPath = topicPath;
		outEnvelope.sourceClientId = sourceClientId;
		outEnvelope.value = latest;
		outEnvelope.deliveryKind = core.GetLiveDeliveryKind(topic->descriptor.topicKind);
		return true;
	}

	bool PublishServerValue(Core& core, const std::string& topicPath, const TopicValue& value, UpdateEnvelope& outEnvelope)
	{
		const WriteResult result = core.PublishFromServer(topicPath, value);
		if (!result.accepted)
			return false;
		return BuildLiveEnvelopeForTopic(core, topicPath, kServerClientId, result.serverSequence, outEnvelope);
	}
}
