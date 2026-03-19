#include <gtest/gtest.h>

#include "Application/DriverStation/DriverStation/NativeLink.h"

#include <thread>

using NativeLink::Core;
using NativeLink::DeliveryKind;
using NativeLink::RetentionMode;
using NativeLink::SnapshotEvent;
using NativeLink::SnapshotEventKind;
using NativeLink::TestClient;
using NativeLink::TopicDescriptor;
using NativeLink::TopicKind;
using NativeLink::TopicValue;
using NativeLink::ValueType;
using NativeLink::WriterPolicy;

namespace
{
	TopicDescriptor MakeStateDoubleTopic(const std::string& path, WriterPolicy policy = WriterPolicy::LeaseSingleWriter)
	{
		TopicDescriptor descriptor;
		descriptor.topicPath = path;
		descriptor.topicKind = TopicKind::State;
		descriptor.valueType = ValueType::Double;
		descriptor.retentionMode = RetentionMode::LatestValue;
		descriptor.replayOnSubscribe = true;
		descriptor.writerPolicy = policy;
		return descriptor;
	}

	const NativeLink::UpdateEnvelope* FindEventByTopic(const std::vector<NativeLink::UpdateEnvelope>& events, const std::string& topic)
	{
		for (std::size_t i = 0; i < events.size(); ++i)
		{
			if (events[i].topicPath == topic)
				return &events[i];
		}
		return nullptr;
	}

	std::vector<NativeLink::UpdateEnvelope> DrainUntilTopic(TestClient& client, const std::string& topic)
	{
		std::vector<NativeLink::UpdateEnvelope> aggregate;
		for (int attempt = 0; attempt < 10; ++attempt)
		{
			std::vector<NativeLink::UpdateEnvelope> batch = client.DrainLiveEvents(100);
			aggregate.insert(aggregate.end(), batch.begin(), batch.end());
			if (FindEventByTopic(aggregate, topic) != nullptr)
				break;
		}
		return aggregate;
	}
}

TEST(NativeLinkCoreTests, SnapshotOrdersDescriptorsBeforeReplayableState)
{
	Core core;
	ASSERT_TRUE(core.RegisterTopic(MakeStateDoubleTopic("TestMove")).ok);
	ASSERT_TRUE(core.AcquireLease("TestMove", "dashboard-a"));
	ASSERT_TRUE(core.Publish("TestMove", TopicValue::Double(3.5), "dashboard-a").accepted);

	const std::vector<SnapshotEvent> snapshot = core.BuildSnapshotForClient("dashboard-b");
	ASSERT_GE(snapshot.size(), 5u);
	EXPECT_EQ(snapshot[0].kind, SnapshotEventKind::DescriptorSnapshotBegin);
	EXPECT_EQ(snapshot[1].kind, SnapshotEventKind::Descriptor);
	EXPECT_EQ(snapshot[2].kind, SnapshotEventKind::DescriptorSnapshotEnd);
	EXPECT_EQ(snapshot[3].kind, SnapshotEventKind::StateSnapshotBegin);
	EXPECT_EQ(snapshot[4].kind, SnapshotEventKind::Update);
	EXPECT_EQ(snapshot[4].update.deliveryKind, DeliveryKind::SnapshotState);
	EXPECT_DOUBLE_EQ(snapshot[4].update.value.doubleValue, 3.5);
}

TEST(NativeLinkCoreTests, LeaseWriterPolicyRejectsNonHolder)
{
	Core core;
	ASSERT_TRUE(core.RegisterTopic(MakeStateDoubleTopic("TestMove")).ok);
	ASSERT_TRUE(core.AcquireLease("TestMove", "dashboard-a"));

	const NativeLink::WriteResult result = core.Publish("TestMove", TopicValue::Double(1.0), "dashboard-b");
	EXPECT_FALSE(result.accepted);
	EXPECT_EQ(result.rejectionReason, NativeLink::WriteRejectReason::LeaseNotHolder);
}

TEST(NativeLinkCoreTests, InProcessServerFansOutTelemetryToTwoClients)
{
	NativeLink::Server server("native-link-test-fanout");
	ASSERT_TRUE(server.Start());

	TestClient dashboardA("native-link-test-fanout", "dashboard-a");
	TestClient dashboardB("native-link-test-fanout", "dashboard-b");
	ASSERT_TRUE(dashboardA.Start());
	ASSERT_TRUE(dashboardB.Start());

	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	dashboardA.Connect(500);
	dashboardB.Connect(500);

	server.PublishNumber("Timer", 12.5);

	const std::vector<NativeLink::UpdateEnvelope> eventsA = DrainUntilTopic(dashboardA, "Timer");
	const std::vector<NativeLink::UpdateEnvelope> eventsB = DrainUntilTopic(dashboardB, "Timer");
	const NativeLink::UpdateEnvelope* timerA = FindEventByTopic(eventsA, "Timer");
	const NativeLink::UpdateEnvelope* timerB = FindEventByTopic(eventsB, "Timer");
	ASSERT_NE(timerA, nullptr);
	ASSERT_NE(timerB, nullptr);
	EXPECT_EQ(timerA->deliveryKind, DeliveryKind::LiveState);
	EXPECT_EQ(timerB->deliveryKind, DeliveryKind::LiveState);
	EXPECT_DOUBLE_EQ(timerA->value.doubleValue, 12.5);
	EXPECT_DOUBLE_EQ(timerB->value.doubleValue, 12.5);

	dashboardA.Stop();
	dashboardB.Stop();
	server.Stop();
}
