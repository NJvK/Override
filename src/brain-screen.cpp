// =====================================================================
//   BRAIN SCREEN
// =====================================================================
// What's on it (480 x 240):
//   X / Y / heading       where odometry thinks the robot is (field inches, 0,0 = field center; heading 0 = up)
//   MCL                   on / off, how spread out its guesses are (small = sure), and how long an update takes
//   FIELD CAM             what the field camera (GOAL_VISION_PORT, front-right corner) sees right now: pins / cups by the
//                           name the sensor's model gives them, with its confidence and where in the picture they are
//                           (left / middle / right), and AprilTags by their number with a distance (from how big the tag
//                           looks and GOAL_TAG_SIZE_IN: check it against a tape measure) and the "raw" bearing (degrees
//                           right of the camera's own straight ahead), which is what sets FP_GOAL_CAM_YAW
//   CLAW CAM              what the claw camera (AI_VISION_PORT, in the claw) sees: pins / cups. Ones that start in the
//                           bottom third of its picture are the red gears and are shown grey
// It's off during autonomous (the tuning tests draw there). After an auton it waits for a tap on the screen before
// it draws again, so a tuning test's results stay up until you've read them.
// =====================================================================
#include "robot.hpp"
#include "rw/hardware.h"   // hw::screenText / screenClear / screenLine / screenTouch

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#ifndef RW_SIM_PROS
#include "pros/ai_vision.hpp"   // the simulator has no AI Vision sensor
#endif

namespace {

constexpr int REFRESH_MS = 100;
constexpr int LIST_ROW_H = 18;
constexpr int FIELD_HEAD_Y = 76, FIELD_LIST_Y = 98, FIELD_ROWS = 4;   // field camera: heading line + 4 rows
constexpr int CLAW_HEAD_Y = 178, CLAW_LIST_Y = 200, CLAW_ROWS = 2;    // claw camera: heading line + 2 rows
constexpr double CAM_WIDTH_PX = 320.0;  // the AI Vision picture is 320 x 240
constexpr double CAM_FOV_DEG = 74.0;    // side to side (same as FP_CAM_FOV_DEG in include/auton-core.hpp)
constexpr int CLAW_CAM_BLOCKED_FROM_PX = 160;   // the claw camera's picture is gears from this row down (= FP_CLAW_CAM_BLOCKED_FROM_PX)
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
pros::AIVision& clawCamera() {
    static pros::AIVision sensor(AI_VISION_PORT);
    return sensor;
}
pros::AIVision& fieldCamera() {
    static pros::AIVision sensor(GOAL_VISION_PORT);
    return sensor;
}

// The model's class names (what it calls each kind of pin / cup), read once a sensor reports them
char classNames[MAX_CLASSES][24];
int classCount = 0;
void loadClassNames(pros::AIVision& camera) {
    if (classCount > 0) return;
    int32_t count = camera.get_class_name(-1, classNames[0]);
    if (count <= 0 || count > MAX_CLASSES) return;   // not ready yet (it takes a moment after plugging in): try next time
    for (int i = 0; i < count; i++) {
        char name[24] = {0};
        if (camera.get_class_name(i, name) == PROS_ERR) return;
        std::snprintf(classNames[i], sizeof(classNames[i]), "%s", name);
    }
    classCount = count;
}

const char* sideOf(double centerPx) {
    if (centerPx < CAM_WIDTH_PX / 3.0) return "left";
    if (centerPx > CAM_WIDTH_PX * 2.0 / 3.0) return "right";
    return "middle";
}

// One camera: a heading line, then up to `rows` lines of what it sees.
//   field = true: pins / cups AND AprilTags, each tag with a distance (from how big it looks and GOAL_TAG_SIZE_IN) and its
//                 "raw" bearing (degrees right of the camera's own straight ahead: for setting FP_GOAL_CAM_YAW)
//   field = false: the claw camera: pins / cups; anything that starts in the bottom third of its picture is a gear
void drawCamera(pros::AIVision& camera, int port, const char* name, bool field, int headerY, int firstRowY, int rows) {
    char text[64];
    int32_t count = camera.get_object_count();
    if (count == PROS_ERR || count < 0 || count > 64) {
        std::snprintf(text, sizeof(text), "%s: not connected", name);
        line(8, headerY, true, YELLOW, text);
        std::snprintf(text, sizeof(text), "Nothing answering on port %d", port);
        line(8, firstRowY, false, GREY, text);
        for (int row = 1; row < rows; row++) line(8, firstRowY + row * LIST_ROW_H, false, GREY, "");
        return;
    }
    loadClassNames(camera);
    const double focalPx = (CAM_WIDTH_PX / 2.0) / std::tan(CAM_FOV_DEG / 2.0 * 3.14159265358979323846 / 180.0);
    int row = 0, shown = 0;
    for (int32_t i = 0; i < count; i++) {
        pros::AIVision::Object seen = camera.get_object(i);
        uint32_t color = WHITE;
        text[0] = '\0';
        if (seen.type & pros::E_AIVISION_DETECTED_OBJECT) {
            const auto& e = seen.object.element;
            const bool gear = !field && e.yoffset >= CLAW_CAM_BLOCKED_FROM_PX;   // starts in the gears: ignored by the autons
            if (e.score < AI_VISION_MIN_SCORE || gear) color = GREY;
            const char* label = (seen.id < classCount) ? classNames[seen.id] : "object";
            double center = e.xoffset + e.width / 2.0;
            std::snprintf(text, sizeof(text), "%-14s conf %3u  %-6s %3ux%-3u%s", label, e.score, sideOf(center), e.width, e.height,
                          gear ? " (gears)" : "");
        } else if (field && (seen.type & pros::E_AIVISION_DETECTED_TAG)) {
            const auto& t = seen.object.tag;
            double center = (t.x0 + t.x1 + t.x2 + t.x3) / 4.0;
            double sizePx = std::max({std::hypot(t.x1 - t.x0, t.y1 - t.y0), std::hypot(t.x2 - t.x1, t.y2 - t.y1),
                                      std::hypot(t.x3 - t.x2, t.y3 - t.y2), std::hypot(t.x0 - t.x3, t.y0 - t.y3)});
            double inches = sizePx > 0.5 ? focalPx * GOAL_TAG_SIZE_IN / sizePx : 0.0;
            double rawDeg = std::atan((center - CAM_WIDTH_PX / 2.0) / focalPx) * 180.0 / 3.14159265358979323846;
            color = YELLOW;
            std::snprintf(text, sizeof(text), "TAG #%-3d %-6s  %3.0f in   raw %+4.0f deg", seen.id, sideOf(center), inches, rawDeg);
        } else {
            continue;   // colors / codes (not used), and tags on the claw camera
        }
        shown++;
        if (row < rows) line(8, firstRowY + row * LIST_ROW_H, false, color, text);
        row++;
    }
    std::snprintf(text, sizeof(text), "%s  port %d   sees %d%s", name, port, shown, row > rows ? " (more)" : "");
    line(8, headerY, true, CYAN, text);
    if (row == 0) line(8, firstRowY, false, GREY, "Nothing in view");
    for (int r = (row == 0 ? 1 : row); r < rows; r++) line(8, firstRowY + r * LIST_ROW_H, false, GREY, "");
}

void drawVision() {
    drawCamera(fieldCamera(), GOAL_VISION_PORT, "FIELD CAM", true, FIELD_HEAD_Y, FIELD_LIST_Y, FIELD_ROWS);
    drawCamera(clawCamera(), AI_VISION_PORT, "CLAW CAM", false, CLAW_HEAD_Y, CLAW_LIST_Y, CLAW_ROWS);
}
#endif

void screenLoop() {
#ifndef RW_SIM_PROS
    // Field camera: pins / cups (the model's objects) and AprilTags. Claw camera: pins / cups.
    fieldCamera().enable_detection_types(pros::AivisionModeType::objects, pros::AivisionModeType::tags);
    clawCamera().enable_detection_types(pros::AivisionModeType::objects);
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
        line(8, FIELD_HEAD_Y, true, GREY, "FIELD CAM: not in the simulator");
        line(8, CLAW_HEAD_Y, true, GREY, "CLAW CAM: not in the simulator");
#endif
        pros::delay(REFRESH_MS);
    }
}

}  // namespace

void startBrainScreen() {
    static pros::Task task(screenLoop);
}
