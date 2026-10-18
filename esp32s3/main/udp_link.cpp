#include "udp_link.h"

#include <fcntl.h>
#include <unistd.h>

#include "esp_log.h"

static const char* TAG = "udp";

bool UdpLink::open(uint16_t port)
{
    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock_ < 0) {
        ESP_LOGE(TAG, "socket() failed");
        return false;
    }
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() to port %u failed", port);
        close(sock_);
        sock_ = -1;
        return false;
    }
    fcntl(sock_, F_SETFL, O_NONBLOCK);
    ESP_LOGI(TAG, "Listening on UDP port %u", port);
    return true;
}

size_t UdpLink::poll(uint8_t* buf, size_t cap)
{
    if (sock_ < 0) return 0;
    socklen_t from_len = sizeof(last_from_);
    int n = recvfrom(sock_, buf, cap, 0, (sockaddr*)&last_from_, &from_len);
    return n > 0 ? (size_t)n : 0;
}

void UdpLink::commitPeer()
{
    bool changed = !has_peer_ || peer_.sin_addr.s_addr != last_from_.sin_addr.s_addr ||
                   peer_.sin_port != last_from_.sin_port;
    peer_ = last_from_;
    has_peer_ = true;
    if (changed) {
        char ip[16];
        inet_ntoa_r(peer_.sin_addr, ip, sizeof(ip));
        ESP_LOGI(TAG, "Telemetry peer %s:%u", ip, ntohs(peer_.sin_port));
    }
}

void UdpLink::send(const uint8_t* data, size_t len)
{
    if (sock_ < 0 || !has_peer_) return;
    sendto(sock_, data, len, 0, (sockaddr*)&peer_, sizeof(peer_));  // Drops silently when buffers are full
}
