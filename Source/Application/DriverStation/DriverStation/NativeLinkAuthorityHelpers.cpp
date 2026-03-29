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
		if (core.IsTopicRegistered("Autonomous/TestMove"))
			return;

		// Ian: OWNERSHIP RULE — whoever comes in last checks if the topic already
		// exists.  Pre-registration is ONLY for topics that need a WriterPolicy the
		// auto-register path cannot provide (auto-register always creates ServerOnly).
		// Do NOT publish initial values for topics the robot code will write — doing
		// so seeds the retained snapshot with stale defaults that (a) create orphan
		// tiles on SmartDashboard before the robot loop starts, and (b) would
		// overwrite a dashboard-selected autonomous choice on robot reboot.
		//
		// AutoChooser topics (options, default, active) are ServerOnly and auto-
		// register on first write from robot code via PublishAutonChooser().
		// AutoChooser/selected needs LeaseSingleWriter so the dashboard can write
		// the user's choice back.  We register the descriptor here but do NOT
		// publish an initial value — the robot's PublishAutonChooser() populates it.

		// --- AutoChooser/selected: LeaseSingleWriter descriptor only, NO initial value.
		// The robot code (AI_Input_Example::Activate -> PublishAutonChooser) publishes
		// the real selected value.  The dashboard writes the user's choice via lease.
		core.RegisterTopic(MakeStateTopic("Autonomous/Auton_Selection/AutoChooser/selected", ValueType::String, WriterPolicy::LeaseSingleWriter));

		// --- Autonomous/TestMove: LeaseSingleWriter descriptor only, NO initial value.
		// Ian: The dashboard layout (Swervelayout.json) uses "Autonomous/TestMove" as
		// the variableKey.  On NativeLink, client writes are rejected as UnknownTopic
		// unless the exact key is pre-registered.  The robot code reads via
		// Auton_Smart_GetSingleValue("TestMove") which tries "Autonomous/TestMove"
		// first, so the read path is covered.  The robot never PutNumber's TestMove —
		// only the dashboard slider writes it.
		core.RegisterTopic(MakeStateTopic("Autonomous/TestMove", ValueType::Double, WriterPolicy::LeaseSingleWriter));

		// Ian: All other topics (Timer, Y_ft, chooser options/default/active, etc.)
		// are ServerOnly and auto-register on first write from robot code via
		// Core::PublishInternal (allowServerOnly=true).  Do NOT add them here.
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
