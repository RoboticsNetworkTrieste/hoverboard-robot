#include "log_forward.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace {

struct LogLine {
    size_t len;
    char text[kLogLineMax];
};

QueueHandle_t s_queue = nullptr;
vprintf_like_t s_previous = nullptr;

int forwardVprintf(const char* fmt, va_list args)
{
    if (s_queue != nullptr) {
        LogLine line;
        va_list copy;
        va_copy(copy, args);
        int n = vsnprintf(line.text, sizeof(line.text), fmt, copy);
        va_end(copy);
        if (n > 0) {
            line.len = (size_t)n < sizeof(line.text) ? (size_t)n : sizeof(line.text) - 1;
            if (xQueueSend(s_queue, &line, 0) != pdTRUE) {
                LogLine oldest;  // Queue full: drop the oldest line so the newest survive
                xQueueReceive(s_queue, &oldest, 0);
                xQueueSend(s_queue, &line, 0);
            }
        }
    }
    return s_previous != nullptr ? s_previous(fmt, args) : 0;
}

}  // namespace

void log_forward_install()
{
    s_queue = xQueueCreate(16, sizeof(LogLine));
    s_previous = esp_log_set_vprintf(forwardVprintf);
}

size_t log_forward_read(char* buf, size_t cap)
{
    LogLine line;
    if (s_queue == nullptr || xQueueReceive(s_queue, &line, 0) != pdTRUE) return 0;
    size_t n = line.len < cap ? line.len : cap;
    std::memcpy(buf, line.text, n);
    return n;
}
