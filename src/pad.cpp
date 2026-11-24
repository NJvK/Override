// =====================================================================
//   CONTROLLER SCREEN (include/pad.hpp)
// =====================================================================
#include "pad.hpp"
#include "robot.hpp"   // controller_1

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace pad {
namespace {
constexpr int SEND_GAP_MS = 55;        // the controller takes one print / rumble every 50 ms
constexpr int REFRESH_MS = 2000;       // all three lines again this often (a controller that reconnected is blank)

// No mutex on purpose: an auton task can be deleted by field control at any moment, and a mutex it held would
// lock the screen for the rest of the match. A line read while it's being written is at worst garbled for 55 ms.
char wanted[3][WIDTH + 1];
char shown[3][WIDTH + 1];
std::atomic<const char*> rumbleWanted{nullptr};
bool started = false;

void setRow(int row, const char* text) {
    if (row < 0 || row > 2) return;
    char padded[WIDTH + 1];
    std::snprintf(padded, sizeof(padded), "%-*.*s", WIDTH, WIDTH, text);
    std::memcpy(wanted[row], padded, sizeof(padded));
}

void writer() {
    int next = 0;
    uint32_t lastRefresh = pros::millis();
    while (true) {
        if (pros::millis() - lastRefresh >= static_cast<uint32_t>(REFRESH_MS)) {
            lastRefresh = pros::millis();
            for (auto& row : shown) row[0] = '\x01';   // never matches: all three get sent again
        }
        const char* pattern = rumbleWanted.exchange(nullptr);
        if (pattern != nullptr) {
            controller_1.rumble(pattern);
        } else {
            for (int k = 0; k < 3; k++) {
                const int row = (next + k) % 3;
                char copy[WIDTH + 1];
                std::memcpy(copy, wanted[row], sizeof(copy));
                copy[WIDTH] = '\0';
                if (std::strcmp(copy, shown[row]) == 0) continue;
                controller_1.print(row, 0, "%s", copy);
                std::memcpy(shown[row], copy, sizeof(copy));
                next = row + 1;
                break;
            }
        }
        pros::delay(SEND_GAP_MS);
    }
}
}  // namespace

void start() {
    if (started) return;
    started = true;
    for (int row = 0; row < 3; row++) {
        setRow(row, "");
        shown[row][0] = '\x01';
    }
    static pros::Task task(writer);
}

void line(int row, const char* format, ...) {
    char text[64];
    va_list args;
    va_start(args, format);
    std::vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    setRow(row, text);
}

void lines(const char* row0, const char* row1, const char* row2) {
    setRow(0, row0);
    setRow(1, row1);
    setRow(2, row2);
}

void rumble(const char* pattern) { rumbleWanted.store(pattern); }
}  // namespace pad
