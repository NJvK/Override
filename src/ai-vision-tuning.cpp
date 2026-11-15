// =====================================================================
//   "AI VISION TUNING" auton (aiVisionTuningAuton)
// =====================================================================
// Sets the AI Vision sensor (AI_VISION_PORT, include/robot.hpp) up for the light it's in, then shows live what it
// detects on the CONTROLLER screen and buzzes the controller for what it sees. Nothing moves. Never ends.
// (This sensor only sees pins / cups with its AI model, not AprilTags.)
//
// What it can tune (PROS has no brightness / contrast / exposure setting for this sensor, so it can't change those):
//   1. White balance: runs the sensor's auto white balance in the current light (the closest thing to adjusting for
//      a dark room). Re-run it any time with A (if the controller's buttons work in auton: under competition control
//      / Timed Run they're usually off, so it also runs by itself at the start).
//   2. Confidence: shows the scores the model gives what it sees, so you can set AI_VISION_MIN_SCORE a bit under
//      what the real pins / cups get (cuts out false detections from glare or dark corners).
//
// Rumble, each time what's in front of it changes to:
//   a pin                 one short buzz
//   a cup                 two short buzzes
//   a stack (pin + cup)   one long buzz
// "What's in front" = the biggest (closest) thing it sees. Pin / cup / stack come from the model's class names
// (PIN_WORD / CUP_WORD / STACK_WORD below): a name with STACK_WORD, or with both words, is a stack. A pin and a cup
// whose boxes overlap in the picture also count as a stack (in case the model sees the two separately).
//
// Controller screen:
//   line 1  what's in front: PIN / CUP / STACK, the model's name for it, its score, and where it is (L / M / R)
//   line 2  over the last 2 s: how often it saw something, and the average / lowest score
//   line 3  how many pins / cups / stacks are in view right now
// The PROS terminal also gets one summary line a second.
// =====================================================================
#include "robot.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#ifndef RW_SIM_PROS
#include "pros/ai_vision.hpp"
#endif

namespace {
// Words in the model's class names (any case) that say what a detection is
const char* const PIN_WORD = "pin";
const char* const CUP_WORD = "cup";
const char* const STACK_WORD = "stack";

constexpr int LOOP_MS = 50;              // the controller takes one print / rumble every 50 ms
constexpr int AWB_WAIT_MS = 1500;        // white balance settling time
constexpr int WINDOW_FRAMES = 20;        // 2 s of readings (one every 100 ms)
constexpr int RUMBLE_GAP_MS = 700;       // at most one buzz pattern this often

enum class Kind { none, pin, cup, stack, other };
const char* kindName(Kind kind) {
    switch (kind) {
        case Kind::pin: return "PIN";
        case Kind::cup: return "CUP";
        case Kind::stack: return "STACK";
        case Kind::other: return "?";
        default: return "-";
    }
}
const char* kindRumble(Kind kind) {
    switch (kind) {
        case Kind::pin: return ".";      // once
        case Kind::cup: return ". .";    // twice
        case Kind::stack: return "-";    // long
        default: return nullptr;
    }
}
bool hasWord(std::string name, const char* word) {
    std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::tolower(c); });
    return name.find(word) != std::string::npos;
}
Kind kindFromName(const std::string& name) {
    bool pin = hasWord(name, PIN_WORD), cup = hasWord(name, CUP_WORD);
    if (hasWord(name, STACK_WORD) || (pin && cup)) return Kind::stack;
    if (cup) return Kind::cup;
    if (pin) return Kind::pin;
    return Kind::other;
}
}  // namespace

void aiVisionTuningAuton() {
#ifdef RW_SIM_PROS
    std::printf("AI Vision Tuning: the simulator has no AI Vision sensor\n");
#else
    pros::AIVision camera(AI_VISION_PORT);
    pros::Controller& pad = controller_1;
    camera.enable_detection_types(pros::AivisionModeType::objects);

    auto whiteBalance = [&]() {
        pad.clear();
        pros::delay(LOOP_MS);
        pad.print(0, 0, "WHITE BALANCE...");
        camera.start_awb();
        pros::delay(AWB_WAIT_MS);
        std::printf("AI Vision Tuning: white balance done\n");
    };
    whiteBalance();

    std::vector<std::string> classNames;
    auto className = [&](int id) -> std::string {
        if (classNames.empty()) {
            char scratch[32] = {0};
            int32_t count = camera.get_class_name(-1, scratch);   // -1 = how many class names the model has
            for (int i = 0; i < count && i < 32; i++) {
                std::optional<std::string> name = camera.get_class_name(i);
                if (!name) { classNames.clear(); break; }
                classNames.push_back(*name);
            }
        }
        if (id >= 0 && id < static_cast<int>(classNames.size())) return classNames[id];
        return "obj#" + std::to_string(id);
    };

    // Last 2 s: did it see something, and the best score that frame
    bool hit[WINDOW_FRAMES] = {};
    int score[WINDOW_FRAMES] = {};
    int frame = 0;

    char lines[3][20] = {"", "", ""};
    int nextLine = 0;
    Kind inFront = Kind::none;      // what was in front last reading
    const char* rumbleWant = nullptr;
    uint32_t lastRumbleMs = 0, lastLogMs = 0;
    bool readThisLoop = false;

    struct Seen { Kind kind; std::string name; int score; double left, right, top, bottom; };

    while (true) {
        uint32_t now = pros::millis();
        if (pad.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            whiteBalance();
            continue;
        }

        readThisLoop = !readThisLoop;   // the sensor every 100 ms, the controller every 50 ms
        if (readThisLoop) {
            int32_t count = camera.get_object_count();
            if (count == PROS_ERR || count < 0 || count > 64) {
                std::snprintf(lines[0], sizeof(lines[0]), "NO AI VISION P%d", AI_VISION_PORT);
                std::snprintf(lines[1], sizeof(lines[1]), "check the port");
                std::snprintf(lines[2], sizeof(lines[2]), " ");
            } else {
                std::vector<Seen> seen;
                for (int32_t i = 0; i < count; i++) {
                    pros::AIVision::Object object = camera.get_object(i);
                    if (!(object.type & pros::E_AIVISION_DETECTED_OBJECT)) continue;
                    const auto& box = object.object.element;
                    std::string name = className(object.id);
                    seen.push_back({kindFromName(name), name, box.score, static_cast<double>(box.xoffset),
                                    static_cast<double>(box.xoffset + box.width), static_cast<double>(box.yoffset),
                                    static_cast<double>(box.yoffset + box.height)});
                }
                // A pin and a cup whose boxes overlap = one stack (the model saw the two parts separately)
                for (size_t a = 0; a < seen.size(); a++) {
                    for (size_t b = a + 1; b < seen.size(); b++) {
                        bool pinCup = (seen[a].kind == Kind::pin && seen[b].kind == Kind::cup) ||
                                      (seen[a].kind == Kind::cup && seen[b].kind == Kind::pin);
                        bool overlap = seen[a].left < seen[b].right && seen[b].left < seen[a].right &&
                                       seen[a].top < seen[b].bottom && seen[b].top < seen[a].bottom;
                        if (pinCup && overlap) {
                            seen[a].kind = Kind::stack;
                            seen[a].left = std::min(seen[a].left, seen[b].left);
                            seen[a].right = std::max(seen[a].right, seen[b].right);
                            seen[a].top = std::min(seen[a].top, seen[b].top);
                            seen[a].bottom = std::max(seen[a].bottom, seen[b].bottom);
                            seen[a].score = std::max(seen[a].score, seen[b].score);
                            seen[a].name += "+" + seen[b].name;
                            seen.erase(seen.begin() + b);
                            b--;
                        }
                    }
                }

                // What's in front: the biggest box (closest)
                int front = -1;
                double biggest = -1;
                int pins = 0, cups = 0, stacks = 0, bestScore = -1;
                for (size_t i = 0; i < seen.size(); i++) {
                    double area = (seen[i].right - seen[i].left) * (seen[i].bottom - seen[i].top);
                    if (area > biggest) { biggest = area; front = static_cast<int>(i); }
                    if (seen[i].kind == Kind::pin) pins++;
                    if (seen[i].kind == Kind::cup) cups++;
                    if (seen[i].kind == Kind::stack) stacks++;
                    bestScore = std::max(bestScore, seen[i].score);
                }
                Kind nowFront = front >= 0 ? seen[front].kind : Kind::none;

                // Line 1: what's in front
                if (front >= 0) {
                    double center = (seen[front].left + seen[front].right) / 2.0;
                    char side = center < 107 ? 'L' : (center > 213 ? 'R' : 'M');
                    std::snprintf(lines[0], sizeof(lines[0]), "%-5s %-7.7s%3d %c", kindName(nowFront), seen[front].name.c_str(),
                                  seen[front].score, side);
                } else {
                    std::snprintf(lines[0], sizeof(lines[0]), "nothing seen");
                }

                // Line 2: the last 2 s
                hit[frame] = bestScore >= 0;
                score[frame] = bestScore;
                frame = (frame + 1) % WINDOW_FRAMES;
                int hits = 0, sum = 0, low = 1000000;
                for (int f = 0; f < WINDOW_FRAMES; f++) {
                    if (!hit[f]) continue;
                    hits++;
                    sum += score[f];
                    low = std::min(low, score[f]);
                }
                if (hits > 0) std::snprintf(lines[1], sizeof(lines[1]), "%3d%% av%d lo%d", hits * 100 / WINDOW_FRAMES, sum / hits, low);
                else std::snprintf(lines[1], sizeof(lines[1]), "  0%% in last 2s");

                // Line 3: counts
                std::snprintf(lines[2], sizeof(lines[2]), "P%d C%d S%d", pins, cups, stacks);

                // Buzz when what's in front changes to a pin / cup / stack
                if (nowFront != inFront && kindRumble(nowFront) != nullptr) rumbleWant = kindRumble(nowFront);
                inFront = nowFront;

                if (now - lastLogMs >= 1000) {
                    lastLogMs = now;
                    std::printf("AI Vision Tuning: front %s (%s, score %d), pins %d cups %d stacks %d, seen %d%% of last 2 s\n",
                                kindName(nowFront), front >= 0 ? seen[front].name.c_str() : "-", front >= 0 ? seen[front].score : -1,
                                pins, cups, stacks, hits * 100 / WINDOW_FRAMES);
                }
            }
        }

        // One controller write per loop: a waiting buzz first, otherwise the next screen line
        if (rumbleWant != nullptr && now - lastRumbleMs >= static_cast<uint32_t>(RUMBLE_GAP_MS)) {
            pad.rumble(rumbleWant);
            rumbleWant = nullptr;
            lastRumbleMs = now;
        } else {
            pad.print(nextLine, 0, "%-18s", lines[nextLine]);
            nextLine = (nextLine + 1) % 3;
        }
        pros::delay(LOOP_MS);
    }
#endif
}
