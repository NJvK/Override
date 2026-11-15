// =====================================================================
//   BRAIN SCREEN
// =====================================================================
// What's on it (480 x 240):
//   X / Y / heading       where odometry thinks the robot is (field inches, 0,0 = field center; heading 0 = up)
//   MCL                   on / off, how spread out its guesses are (small = sure), and how long an update takes
//   AI VISION             everything the AI Vision sensor (AI_VISION_PORT, include/robot.hpp) sees right now:
//                           pins / cups by the name the sensor's model gives them, with its confidence and where in
//                           the picture it is (left / middle / right), and AprilTags by their number
// It's off during autonomous (the tuning tests draw there). After an auton it waits for a tap on the screen before
// it draws again, so a tuning test's results stay up until you've read them.
// =====================================================================
#include "robot.hpp"
#include "rw/hardware.h"   // hw::screenText / screenClear / screenLine / screenTouch

#include <cstdio>
#include <cstring>

#ifndef RW_SIM_PROS
#include "pros/ai_vision.hpp"   // the simulator has no AI Vision sensor
#endif

namespace {

constexpr int REFRESH_MS = 100;
constexpr int LIST_ROWS = 7;            // detections listed (the rest are counted)
constexpr int LIST_TOP_Y = 100;         // bottom of the first list row
constexpr int LIST_ROW_H = 18;
constexpr double CAM_WIDTH_PX = 320.0;  // the AI Vision picture is 320 wide
constexpr int MAX_CLASSES = 16;

constexpr uint32_t BLACK = 0x000000;
constexpr uint32_t WHITE = 0xFFFFFF;
constexpr uint32_t GREY = 0x9CA3AF;
constexpr uint32_t GREEN = 0x4ADE80;
constexpr uint32_t YELLOW = 0xFACC15;
constexpr uint32_t CYAN = 0x22D3EE;

// One line of text, padded to the full width so it covers whatever was there before
void line(int x, int y, bool big, uint32_t color, const char* text) {
    char padded[64];
    int width = big ? 48 : 60;   // characters that fit across from x = 0
    width -= x / (big ? 10 : 8);
    std::snprintf(padded, sizeof(padded), "%-*.*s", width, width, text);
    hw::screenText(x, y, big, color, BLACK, padded);
}

void drawFrame() {
    hw::screenClear(BLACK);
    hw::screenLine(0, 54, 479, 54, GREY);
}

void drawPose() {
    char text[64];
    std::snprintf(text, sizeof(text), "X %7.1f  Y %7.1f  H %6.1f", x_pos, y_pos, getInertialHeading());
    line(8, 24, true, WHITE, text);
    if (mcl_running) {
        std::snprintf(text, sizeof(text), "MCL on   spread %.1f in   %.1f ms   (MCL x %.1f  y %.1f)", mcl_spread, mcl_cycle_msec,
                      mcl_x, mcl_y);
        line(8, 46, false, GREEN, text);
    } else {
        line(8, 46, false, GREY, use_mcl ? "MCL starting..." : "MCL off (use_mcl in src/rw-config.cpp)");
    }
}

#ifndef RW_SIM_PROS
pros::AIVision& camera() {
    static pros::AIVision sensor(AI_VISION_PORT);
    return sensor;
}

// The model's class names (what it calls each kind of pin / cup), read once the sensor reports them
char classNames[MAX_CLASSES][24];
int classCount = 0;
void loadClassNames() {
    if (classCount > 0) return;
    int32_t count = camera().get_class_name(-1, classNames[0]);
    if (count <= 0 || count > MAX_CLASSES) return;   // not ready yet (it takes a moment after plugging in): try next time
    for (int i = 0; i < count; i++) {
        char name[24] = {0};
        if (camera().get_class_name(i, name) == PROS_ERR) return;
        std::snprintf(classNames[i], sizeof(classNames[i]), "%s", name);
    }
    classCount = count;
}

const char* sideOf(double centerPx) {
    if (centerPx < CAM_WIDTH_PX / 3.0) return "left";
    if (centerPx > CAM_WIDTH_PX * 2.0 / 3.0) return "right";
    return "middle";
}

void drawVision() {
    char text[64];
    int32_t count = camera().get_object_count();
    if (count == PROS_ERR || count < 0 || count > 64) {
        line(8, 76, true, YELLOW, "AI VISION: not connected");
        std::snprintf(text, sizeof(text), "Nothing answering on port %d", AI_VISION_PORT);
        line(8, LIST_TOP_Y, false, GREY, text);
        for (int row = 1; row < LIST_ROWS; row++) line(8, LIST_TOP_Y + row * LIST_ROW_H, false, GREY, "");
        line(8, 232, false, GREY, "");
        return;
    }
    loadClassNames();
    std::snprintf(text, sizeof(text), "AI VISION  port %d   sees %d", AI_VISION_PORT, static_cast<int>(count));
    line(8, 76, true, CYAN, text);

    int row = 0;
    int pins = 0, tags = 0, other = 0;
    for (int32_t i = 0; i < count; i++) {
        pros::AIVision::Object seen = camera().get_object(i);
        uint32_t color = WHITE;
        text[0] = '\0';
        if (seen.type & pros::E_AIVISION_DETECTED_OBJECT) {
            // A pin / cup the model knows: its class name, the model's confidence, and where it is in the picture
            pins++;
            if (seen.object.element.score < AI_VISION_MIN_SCORE) color = GREY;   // below AI_VISION_MIN_SCORE: ignored by the auton
            const char* name = (seen.id < classCount) ? classNames[seen.id] : nullptr;
            double center = seen.object.element.xoffset + seen.object.element.width / 2.0;
            if (name != nullptr) {
                std::snprintf(text, sizeof(text), "%-16s conf %3u   %-6s  %3ux%-3u px", name, seen.object.element.score, sideOf(center),
                              seen.object.element.width, seen.object.element.height);
            } else {
                std::snprintf(text, sizeof(text), "object #%-8d conf %3u   %-6s  %3ux%-3u px", seen.id, seen.object.element.score,
                              sideOf(center), seen.object.element.width, seen.object.element.height);
            }
        } else if (seen.type & pros::E_AIVISION_DETECTED_TAG) {
            tags++;
            color = YELLOW;
            double center = (seen.object.tag.x0 + seen.object.tag.x1 + seen.object.tag.x2 + seen.object.tag.x3) / 4.0;
            std::snprintf(text, sizeof(text), "APRILTAG #%d   %s", seen.id, sideOf(center));
        } else {
            other++;   // color / code detections (not used): counted, not listed
            continue;
        }
        if (row < LIST_ROWS) line(8, LIST_TOP_Y + row * LIST_ROW_H, false, color, text);
        row++;
    }
    if (row == 0) line(8, LIST_TOP_Y, false, GREY, "Nothing in view");
    for (int r = (row == 0 ? 1 : row); r < LIST_ROWS; r++) line(8, LIST_TOP_Y + r * LIST_ROW_H, false, GREY, "");
    std::snprintf(text, sizeof(text), "%d pin/cup   %d tag   %d other%s", pins, tags, other, row > LIST_ROWS ? "   (more not shown)" : "");
    line(8, 232, false, GREY, text);
}
#endif

void screenLoop() {
#ifndef RW_SIM_PROS
    // Pins / cups (the model's objects) and AprilTags. The 4-pin auton only needs objects, and only looks at those.
    camera().enable_detection_types(pros::AivisionModeType::objects, pros::AivisionModeType::tags);
#endif
    bool drawnFrame = false;
    bool wasAuton = false;
    bool waitForTap = false;
    while (true) {
        if (pros::competition::is_autonomous()) {
            // The tuning tests and their results use the screen in auton
            wasAuton = true;
            drawnFrame = false;
            pros::delay(REFRESH_MS);
            continue;
        }
        if (wasAuton) {
            wasAuton = false;
            waitForTap = true;
        }
        if (waitForTap) {
            int x = 0, y = 0;
            if (!hw::screenTouch(x, y)) {
                pros::delay(REFRESH_MS);
                continue;
            }
            waitForTap = false;
        }
        if (!drawnFrame) {
            drawFrame();
            drawnFrame = true;
        }
        drawPose();
#ifndef RW_SIM_PROS
        drawVision();
#else
        line(8, 76, true, GREY, "AI VISION: not in the simulator");
#endif
        pros::delay(REFRESH_MS);
    }
}

}  // namespace

void startBrainScreen() {
    static pros::Task task(screenLoop);
}
