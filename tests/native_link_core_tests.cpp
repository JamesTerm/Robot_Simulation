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

TEST(NativeLinkCoreTests, CarrierParserRecognizesSharedMemoryAndTcpNames)
{
	NativeLink::CarrierKind kind = NativeLink::CarrierKind::Tcp;
	EXPECT_TRUE(NativeLink::TryParseCarrierKind("shm", kind));
	EXPECT_EQ(kind, NativeLink::CarrierKind::SharedMemory);
	EXPECT_TRUE(NativeLink::TryParseCarrierKind("shared-memory", kind));
	EXPECT_EQ(kind, NativeLink::CarrierKind::SharedMemory);
	EXPECT_TRUE(NativeLink::TryParseCarrierKind("tcp", kind));
	EXPECT_EQ(kind, NativeLink::CarrierKind::Tcp);
	EXPECT_FALSE(NativeLink::TryParseCarrierKind("udp", kind));
}

TEST(NativeLinkCoreTests, ExplicitTcpCarrierSelectionFailsUntilTcpBackendExists)
{
	NativeLink::ServerConfig serverConfig;
	serverConfig.carrierKind = NativeLink::CarrierKind::Tcp;
	serverConfig.channelId = "native-link-tcp-placeholder";
	serverConfig.port = 5820;
	NativeLink::Server server(serverConfig);
	EXPECT_TRUE(server.Start());

	NativeLink::TestClientConfig clientConfig;
	clientConfig.carrierKind = NativeLink::CarrierKind::Tcp;
	clientConfig.channelId = "native-link-tcp-placeholder";
	clientConfig.clientId = "dashboard-a";
	clientConfig.port = 5820;
	NativeLink::TestClient client(clientConfig);
	ASSERT_TRUE(client.Start());

	const std::vector<NativeLink::SnapshotEvent> snapshot = client.Connect(1000);
	ASSERT_FALSE(snapshot.empty());
	server.PublishNumber("Timer", 6.25);
	const std::vector<NativeLink::UpdateEnvelope> live = DrainUntilTopic(client, "Timer");
	const NativeLink::UpdateEnvelope* timer = FindEventByTopic(live, "Timer");
	ASSERT_NE(timer, nullptr);
	EXPECT_DOUBLE_EQ(timer->value.doubleValue, 6.25);

	ASSERT_TRUE(client.PublishNumber("TestMove", 8.5));
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	double latestMove = 0.0;
	EXPECT_TRUE(server.TryGetNumber("TestMove", latestMove));
	EXPECT_DOUBLE_EQ(latestMove, 8.5);

	client.Stop();
	server.Stop();
}

TEST(NativeLinkCoreTests, ServerConfigLoadsTcpSettingsFromEnvironment)
{
	char* oldCarrier = nullptr;
	char* oldChannel = nullptr;
	char* oldHost = nullptr;
	char* oldPort = nullptr;
	std::size_t ignored = 0;
	_dupenv_s(&oldCarrier, &ignored, "NATIVE_LINK_CARRIER");
	_dupenv_s(&oldChannel, &ignored, "NATIVE_LINK_CHANNEL_ID");
	_dupenv_s(&oldHost, &ignored, "NATIVE_LINK_HOST");
	_dupenv_s(&oldPort, &ignored, "NATIVE_LINK_PORT");

	_putenv("NATIVE_LINK_CARRIER=tcp");
	_putenv("NATIVE_LINK_CHANNEL_ID=native-link-env-test");
	_putenv("NATIVE_LINK_HOST=127.0.0.1");
	_putenv("NATIVE_LINK_PORT=5821");

	const NativeLink::ServerConfig config = NativeLink::LoadServerConfigFromEnvironment();
	EXPECT_EQ(config.carrierKind, NativeLink::CarrierKind::Tcp);
	EXPECT_EQ(config.channelId, "native-link-env-test");
	EXPECT_EQ(config.host, "127.0.0.1");
	EXPECT_EQ(config.port, 5821);

	if (oldCarrier != nullptr)
	{
		std::string restore = std::string("NATIVE_LINK_CARRIER=") + oldCarrier;
		_putenv(restore.c_str());
		free(oldCarrier);
	}
	else
	{
		_putenv("NATIVE_LINK_CARRIER=");
	}

	if (oldChannel != nullptr)
	{
		std::string restore = std::string("NATIVE_LINK_CHANNEL_ID=") + oldChannel;
		_putenv(restore.c_str());
		free(oldChannel);
	}
	else
	{
		_putenv("NATIVE_LINK_CHANNEL_ID=");
	}

	if (oldHost != nullptr)
	{
		std::string restore = std::string("NATIVE_LINK_HOST=") + oldHost;
		_putenv(restore.c_str());
		free(oldHost);
	}
	else
	{
		_putenv("NATIVE_LINK_HOST=");
	}

	if (oldPort != nullptr)
	{
		std::string restore = std::string("NATIVE_LINK_PORT=") + oldPort;
		_putenv(restore.c_str());
		free(oldPort);
	}
	else
	{
		_putenv("NATIVE_LINK_PORT=");
	}
}
