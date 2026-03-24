#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace sd::nativelink
{
    constexpr std::uint32_t kSharedMagic = 0x4E4C4E4B;
    constexpr std::uint32_t kSharedVersion = 3;
    constexpr std::uint32_t kMaxClients = 8;
    constexpr std::uint32_t kMaxMessages = 1024;
    constexpr std::uint32_t kMaxPayloadBytes = 1024;

    constexpr std::uint32_t kSnapshotStartTopicId = 0xFFFFFFF0u;
    constexpr std::uint32_t kSnapshotEndTopicId = 0xFFFFFFF1u;
    constexpr std::uint32_t kLiveBeginTopicId = 0xFFFFFFF2u;

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
        // Ian: The mapped clients array carries 64-bit atomics, so preserve
        // 8-byte alignment in the on-disk/shared-memory layout instead of
        // relying on packed-struct luck.
        std::uint32_t reserved0 = 0;
        char channelId[64];
        SharedClientSlot clients[kMaxClients];
    };

    // Ian: Fixed-width fields already make this IPC layout deterministic.
    // Avoid packed structs here so the shared atomics keep their natural
    // alignment instead of depending on compiler-specific packed behavior.

    static_assert(offsetof(SharedClientSlot, clientTag) % alignof(std::atomic<std::uint64_t>) == 0);
    static_assert(offsetof(SharedClientSlot, lastHeartbeatUs) % alignof(std::atomic<std::uint64_t>) == 0);
    static_assert(offsetof(SharedClientSlot, snapshotCompleteSessionId) % alignof(std::atomic<std::uint64_t>) == 0);
    static_assert(offsetof(SharedClientSlot, lastAckedSequence) % alignof(std::atomic<std::uint64_t>) == 0);
    static_assert(offsetof(SharedClientSlot, clientWriteSequence) % alignof(std::atomic<std::uint64_t>) == 0);
    static_assert(offsetof(SharedState, clients) % alignof(std::atomic<std::uint64_t>) == 0);
    static_assert(sizeof(SharedClientSlot) % alignof(std::atomic<std::uint64_t>) == 0);
}
