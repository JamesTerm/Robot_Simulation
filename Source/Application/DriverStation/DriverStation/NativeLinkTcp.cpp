#include "stdafx.h"
#include "NativeLinkTcp.h"

#include "NativeLinkAuthorityHelpers.h"
#include "NativeLinkSharedMemory.h"

#include <WinSock2.h>
#include <WS2tcpip.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <utility>

namespace NativeLink::detail
{
	namespace
	{
		enum class ReceiveStatus
		{
			Success,
			Timeout,
			Closed,
			Error
		};

		class WinsockRuntime
		{
		public:
			WinsockRuntime()
			{
				WSADATA data {};
				m_started = WSAStartup(MAKEWORD(2, 2), &data) == 0;
				if (!m_started)
					OutputDebugStringA("[NativeLinkTcp] WSAStartup failed\n");
			}

			~WinsockRuntime()
			{
				if (m_started)
					WSACleanup();
			}

			bool IsStarted() const
			{
				return m_started;
			}

		private:
			bool m_started = false;
		};

		WinsockRuntime& GetWinsockRuntime()
		{
			static WinsockRuntime runtime;
			return runtime;
		}

		bool SendAll(SOCKET socketHandle, const char* bytes, int totalBytes)
		{
			int sent = 0;
			while (sent < totalBytes)
			{
				const int chunk = send(socketHandle, bytes + sent, totalBytes - sent, 0);
				if (chunk == SOCKET_ERROR || chunk == 0)
					return false;
				sent += chunk;
			}
			return true;
		}

		ReceiveStatus RecvAllWithTimeout(SOCKET socketHandle, char* bytes, int totalBytes, std::uint32_t timeoutMs)
		{
			int received = 0;
			while (received < totalBytes)
			{
				fd_set readSet;
				FD_ZERO(&readSet);
				FD_SET(socketHandle, &readSet);

				timeval timeout {};
				timeout.tv_sec = static_cast<long>(timeoutMs / 1000);
				timeout.tv_usec = static_cast<long>((timeoutMs % 1000) * 1000);

				const int ready = select(0, &readSet, nullptr, nullptr, &timeout);
				if (ready == 0)
					return ReceiveStatus::Timeout;
				if (ready == SOCKET_ERROR)
					return ReceiveStatus::Error;

				const int chunk = recv(socketHandle, bytes + received, totalBytes - received, 0);
				if (chunk == 0)
					return ReceiveStatus::Closed;
				if (chunk == SOCKET_ERROR)
					return ReceiveStatus::Error;
				received += chunk;
			}
			return ReceiveStatus::Success;
		}

		bool SendFrame(SOCKET socketHandle, TcpFrameKind kind, const void* payload, std::uint32_t payloadBytes)
		{
			TcpFrameHeader header {};
			header.magic = kTcpFrameMagic;
			header.version = kTcpFrameVersion;
			header.kind = static_cast<std::uint16_t>(kind);
			header.payloadBytes = payloadBytes;
			if (!SendAll(socketHandle, reinterpret_cast<const char*>(&header), static_cast<int>(sizeof(header))))
				return false;
			if (payloadBytes > 0 && payload != nullptr)
			{
				if (!SendAll(socketHandle, reinterpret_cast<const char*>(payload), static_cast<int>(payloadBytes)))
					return false;
			}
			return true;
		}

		ReceiveStatus ReceiveFrame(SOCKET socketHandle, std::uint32_t timeoutMs, TcpFrameKind& outKind, std::vector<unsigned char>& outPayload)
		{
			TcpFrameHeader header {};
			const ReceiveStatus headerStatus = RecvAllWithTimeout(socketHandle, reinterpret_cast<char*>(&header), static_cast<int>(sizeof(header)), timeoutMs);
			if (headerStatus != ReceiveStatus::Success)
				return headerStatus;
			if (header.magic != kTcpFrameMagic || header.version != kTcpFrameVersion || header.payloadBytes > sizeof(SharedMessage))
				return ReceiveStatus::Error;

			outKind = static_cast<TcpFrameKind>(header.kind);
			outPayload.resize(header.payloadBytes);
			if (header.payloadBytes > 0)
			{
				const ReceiveStatus payloadStatus = RecvAllWithTimeout(socketHandle, reinterpret_cast<char*>(outPayload.data()), static_cast<int>(outPayload.size()), timeoutMs);
				if (payloadStatus != ReceiveStatus::Success)
					return payloadStatus;
			}
			return ReceiveStatus::Success;
		}
	}

	class TcpServerCarrier final : public ITcpServerCarrier
	{
	public:
		TcpServerCarrier(const ServerConfig& config, Core& core)
			: m_config(config)
			, m_core(core)
			, m_running(false)
			, m_stopRequested(false)
		{
		}

		~TcpServerCarrier() override
		{
			Stop();
		}

		bool Start() override
		{
			if (m_running.load())
				return true;
			if (!GetWinsockRuntime().IsStarted())
				return false;

			m_listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
			if (m_listenSocket == INVALID_SOCKET)
				return false;

			const BOOL reuse = 1;
			setsockopt(m_listenSocket, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse));

			sockaddr_in address {};
			address.sin_family = AF_INET;
			address.sin_port = htons(m_config.port);
			if (InetPtonA(AF_INET, m_config.host.c_str(), &address.sin_addr) != 1)
				return false;
			if (bind(m_listenSocket, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == SOCKET_ERROR)
				return false;
			if (listen(m_listenSocket, SOMAXCONN) == SOCKET_ERROR)
				return false;

			m_stopRequested.store(false);
			m_running.store(true);
			m_acceptThread = std::thread(&TcpServerCarrier::AcceptLoop, this);
			m_heartbeatThread = std::thread(&TcpServerCarrier::HeartbeatLoop, this);
			return true;
		}

		void Stop() override
		{
			m_stopRequested.store(true);
			m_running.store(false);

			if (m_listenSocket != INVALID_SOCKET)
			{
				shutdown(m_listenSocket, SD_BOTH);
				closesocket(m_listenSocket);
				m_listenSocket = INVALID_SOCKET;
			}

			if (m_acceptThread.joinable())
				m_acceptThread.join();
			if (m_heartbeatThread.joinable())
				m_heartbeatThread.join();

			std::lock_guard<std::mutex> lock(m_mutex);
			for (std::size_t i = 0; i < m_clients.size(); ++i)
			{
				if (m_clients[i]->socketHandle != INVALID_SOCKET)
				{
					shutdown(m_clients[i]->socketHandle, SD_BOTH);
					closesocket(m_clients[i]->socketHandle);
					m_clients[i]->socketHandle = INVALID_SOCKET;
				}
				if (m_clients[i]->thread.joinable())
					m_clients[i]->thread.join();
			}
			m_clients.clear();
		}

		void PublishSnapshotForClient(const std::string& clientId, const std::vector<SnapshotEvent>& snapshot) override
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			ClientSlot* client = FindClient(clientId);
			if (client == nullptr)
				return;
			for (std::size_t i = 0; i < snapshot.size(); ++i)
				SendEnvelopeLocked(*client, SnapshotEventToEnvelope(snapshot[i], m_core.GetServerSessionId()));
		}

		void PublishEnvelopeToAllClients(const UpdateEnvelope& envelope) override
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			for (std::size_t i = 0; i < m_clients.size(); ++i)
			{
				if (m_clients[i]->alive.load())
					SendEnvelopeLocked(*m_clients[i], envelope);
			}
		}

		void PublishEnvelopeToSingleClient(const std::string& clientId, const UpdateEnvelope& envelope) override
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			ClientSlot* client = FindClient(clientId);
			if (client != nullptr)
				SendEnvelopeLocked(*client, envelope);
		}

		void DrainClientWrites(std::vector<std::pair<std::string, std::pair<std::string, TopicValue>>>& writes) override
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			writes.swap(m_pendingWrites);
		}

	private:
		struct ClientSlot
		{
			SOCKET socketHandle = INVALID_SOCKET;
			std::string clientId;
			std::atomic<bool> alive;
			std::atomic<std::uint64_t> lastHeartbeatUs;
			std::thread thread;

			ClientSlot()
				: alive(false)
				, lastHeartbeatUs(0)
			{
			}
		};

		ServerConfig m_config;
		Core& m_core;
		SOCKET m_listenSocket = INVALID_SOCKET;
		std::thread m_acceptThread;
		std::thread m_heartbeatThread;
		std::mutex m_mutex;
		std::vector<std::unique_ptr<ClientSlot>> m_clients;
		std::vector<std::pair<std::string, std::pair<std::string, TopicValue>>> m_pendingWrites;
		std::atomic<bool> m_running;
		std::atomic<bool> m_stopRequested;

		TcpServerCarrier(const TcpServerCarrier&) = delete;
		TcpServerCarrier& operator=(const TcpServerCarrier&) = delete;

		ClientSlot* FindClient(const std::string& clientId)
		{
			for (std::size_t i = 0; i < m_clients.size(); ++i)
			{
				if (m_clients[i]->clientId == clientId)
					return m_clients[i].get();
			}
			return nullptr;
		}

		void AcceptLoop()
		{
			while (!m_stopRequested.load())
			{
				SOCKET clientSocket = accept(m_listenSocket, nullptr, nullptr);
				if (clientSocket == INVALID_SOCKET)
					break;
				auto client = std::make_unique<ClientSlot>();
				client->socketHandle = clientSocket;
				client->alive.store(true);
				client->lastHeartbeatUs.store(GetSteadyNowUs());
				client->thread = std::thread(&TcpServerCarrier::ClientLoop, this, client.get());
				std::lock_guard<std::mutex> lock(m_mutex);
				m_clients.push_back(std::move(client));
			}
		}

		void HeartbeatLoop()
		{
			while (!m_stopRequested.load())
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				std::lock_guard<std::mutex> lock(m_mutex);
				TcpHeartbeatPayload heartbeat {};
				heartbeat.serverSessionId = m_core.GetServerSessionId();
				for (std::size_t i = 0; i < m_clients.size(); ++i)
				{
					if (m_clients[i]->alive.load() && m_clients[i]->socketHandle != INVALID_SOCKET)
						SendFrame(m_clients[i]->socketHandle, TcpFrameKind::Heartbeat, &heartbeat, sizeof(heartbeat));
				}
			}
		}

		void ClientLoop(ClientSlot* client)
		{
			TcpFrameKind kind = TcpFrameKind::Heartbeat;
			std::vector<unsigned char> payload;
			if (ReceiveFrame(client->socketHandle, 2000, kind, payload) != ReceiveStatus::Success
				|| kind != TcpFrameKind::ClientHello
				|| payload.size() != sizeof(TcpHelloPayload))
			{
				client->alive.store(false);
				return;
			}

			TcpHelloPayload hello {};
			memcpy(&hello, payload.data(), sizeof(hello));
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				client->clientId = ReadUtf8(hello.clientId, sizeof(hello.clientId));
				TcpServerHelloPayload serverHello {};
				serverHello.serverSessionId = m_core.GetServerSessionId();
				SendFrame(client->socketHandle, TcpFrameKind::ServerHello, &serverHello, sizeof(serverHello));

				// Ian: The reference TCP authority sends the same snapshot-first
				// bootstrap story as SHM immediately after hello. Socket connect alone
				// is not semantic readiness; the client still has to rebuild from the
				// authority-owned descriptor/state snapshot before live deltas matter.
				const std::vector<SnapshotEvent> snapshot = m_core.ConnectClient(client->clientId).snapshotEvents;
				for (std::size_t i = 0; i < snapshot.size(); ++i)
					SendEnvelopeLocked(*client, SnapshotEventToEnvelope(snapshot[i], m_core.GetServerSessionId()));
			}

			while (!m_stopRequested.load() && client->alive.load())
			{
				const ReceiveStatus status = ReceiveFrame(client->socketHandle, 100, kind, payload);
				if (status == ReceiveStatus::Timeout)
					continue;
				if (status != ReceiveStatus::Success)
					break;
				client->lastHeartbeatUs.store(GetSteadyNowUs());
				if (kind == TcpFrameKind::Heartbeat)
					continue;
				if (kind == TcpFrameKind::ClientPublish && payload.size() == sizeof(SharedMessage))
				{
					SharedMessage message {};
					memcpy(&message, payload.data(), sizeof(message));
					TopicValue value;
					if (DeserializeValue(static_cast<ValueType>(message.valueType), message.payload, static_cast<std::size_t>(message.flags), value))
					{
						std::lock_guard<std::mutex> lock(m_mutex);
						m_pendingWrites.push_back(std::make_pair(client->clientId, std::make_pair(ReadUtf8(message.topicPath, sizeof(message.topicPath)), value)));
					}
					continue;
				}
				break;
			}

			client->alive.store(false);
			if (client->socketHandle != INVALID_SOCKET)
			{
				shutdown(client->socketHandle, SD_BOTH);
				closesocket(client->socketHandle);
				client->socketHandle = INVALID_SOCKET;
			}
		}

		void SendEnvelopeLocked(ClientSlot& client, const UpdateEnvelope& envelope)
		{
			const SharedMessage message = BuildSharedMessage(envelope);
			SendFrame(client.socketHandle, TcpFrameKind::ServerMessage, &message, sizeof(message));
		}

	public:
		TcpServerCarrier(TcpServerCarrier&&) = delete;
		TcpServerCarrier& operator=(TcpServerCarrier&&) = delete;
	};

	class TcpClientCarrier final : public ITcpClientCarrier
	{
	public:
		explicit TcpClientCarrier(const TestClientConfig& config)
			: m_config(config)
		{
		}

		~TcpClientCarrier() override
		{
			Stop();
		}

		bool Start() override
		{
			if (!GetWinsockRuntime().IsStarted())
				return false;
			m_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
			if (m_socket == INVALID_SOCKET)
				return false;

			sockaddr_in address {};
			address.sin_family = AF_INET;
			address.sin_port = htons(m_config.port);
			if (InetPtonA(AF_INET, m_config.host.c_str(), &address.sin_addr) != 1)
				return false;
			if (connect(m_socket, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == SOCKET_ERROR)
				return false;

			TcpHelloPayload hello {};
			CopyUtf8(hello.channelId, sizeof(hello.channelId), m_config.channelId);
			CopyUtf8(hello.clientId, sizeof(hello.clientId), m_config.clientId);
			if (!SendFrame(m_socket, TcpFrameKind::ClientHello, &hello, sizeof(hello)))
				return false;

			TcpFrameKind kind = TcpFrameKind::Heartbeat;
			std::vector<unsigned char> payload;
			if (ReceiveFrame(m_socket, 2000, kind, payload) != ReceiveStatus::Success
				|| kind != TcpFrameKind::ServerHello
				|| payload.size() != sizeof(TcpServerHelloPayload))
				return false;
			return true;
		}

		void Stop() override
		{
			if (m_socket != INVALID_SOCKET)
			{
				shutdown(m_socket, SD_BOTH);
				closesocket(m_socket);
				m_socket = INVALID_SOCKET;
			}
		}

		std::vector<UpdateEnvelope> DrainMessages(std::uint32_t timeoutMs) override
		{
			std::vector<UpdateEnvelope> result;
			if (m_socket == INVALID_SOCKET)
				return result;

			TcpFrameKind kind = TcpFrameKind::Heartbeat;
			std::vector<unsigned char> payload;
			const std::uint32_t firstTimeoutMs = timeoutMs > 0 ? timeoutMs : 1;
			std::uint32_t currentTimeoutMs = firstTimeoutMs;
			while (true)
			{
				const ReceiveStatus status = ReceiveFrame(m_socket, currentTimeoutMs, kind, payload);
				if (status == ReceiveStatus::Timeout)
					break;
				if (status != ReceiveStatus::Success)
					break;
				if (kind == TcpFrameKind::ServerMessage && payload.size() == sizeof(SharedMessage))
				{
					SharedMessage message {};
					memcpy(&message, payload.data(), sizeof(message));
					result.push_back(SharedMessageToUpdateEnvelope(message));
					currentTimeoutMs = 1;
					continue;
				}
				if (kind == TcpFrameKind::Heartbeat)
				{
					currentTimeoutMs = 1;
					continue;
				}
				break;
			}
			return result;
		}

		bool Publish(const std::string& keyName, const TopicValue& value) override
		{
			if (m_socket == INVALID_SOCKET)
				return false;
			const SharedMessage message = BuildClientWriteMessage(keyName, m_config.clientId, value);
			return SendFrame(m_socket, TcpFrameKind::ClientPublish, &message, sizeof(message));
		}

	private:
		TestClientConfig m_config;
		SOCKET m_socket = INVALID_SOCKET;
	};

	std::unique_ptr<ITcpServerCarrier> CreateTcpServerCarrier(const ServerConfig& config, Core& core)
	{
		return std::make_unique<TcpServerCarrier>(config, core);
	}

	std::unique_ptr<ITcpClientCarrier> CreateTcpClientCarrier(const TestClientConfig& config)
	{
		return std::make_unique<TcpClientCarrier>(config);
	}
}
