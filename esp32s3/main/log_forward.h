#pragma once

#include <cstddef>

constexpr size_t kLogLineMax = 128;

// Hooks ESP logging so each line can also be sent as a LOG datagram.
void log_forward_install();
// Copies one pending log line into buf. Returns its length, 0 if none. Non-blocking.
size_t log_forward_read(char* buf, size_t cap);
