#pragma once

#include <cstddef>
#include <cstdint>

#include "lwip/sockets.h"

// Non-blocking UDP socket on all interfaces. Replies go to the committed peer.
class UdpLink {
public:
    bool open(uint16_t port);
    // Returns the datagram size, or 0 if nothing is pending. Remembers its sender.
    size_t poll(uint8_t* buf, size_t cap);
    // Makes the sender of the last polled datagram the telemetry destination.
    void commitPeer();
    // Sends to the committed peer; dropped if there is none.
    void send(const uint8_t* data, size_t len);
    bool hasPeer() const { return has_peer_; }

private:
    int sock_ = -1;
    sockaddr_in last_from_{};
    sockaddr_in peer_{};
    bool has_peer_ = false;
};
