// =====================================================================
//   AI VISION TUNING: "Claw Cam" (port 5) and "Field Cam" (port 21)
// =====================================================================
// Picked on the controller like any auton (src/auton-select.cpp). The robot doesn't drive. Everything is on the
// CONTROLLER screen; the PROS terminal gets one summary line a second. A = auto white balance for the light it's in
// (PROS has no brightness / exposure setting for this sensor: white balance is the closest thing). It also runs by
// itself at the start. Bench mode: hold B to stop.
//
// Both tools see what the AUTOS see: CUPS only (FP_CUP_CLASS_IDS; pins are ignored, the model finds them too badly),
// with the autos' filters, and "sure" only when there is exactly one cup (boxes within FP_AIM_SAME_IN of each other at
// about a foot are one cup). Two cups that disagree = the autos skip that frame. The model flickers, so the bearings
// shown are averages over the last 2 s of sure frames.
//
// CLAW CAM (AI_VISION_PORT, in the claw). The claw motor stays OFF: open the claw by hand first (the autos only read
// this camera with the claw open, and closed its arms fill the picture).
//   line 1  CUP / 2+ CUPS / no cup right now, and a = the cup's average bearing (degrees right of the robot's straight
//           ahead, with FP_PIN_CAM_YAW). Centering check: a cup on the claw's center line should read about 0.
//   line 2  sure% = frames with exactly one cup over the last 2 s, and the average / lowest cup score
//           -> AI_VISION_MIN_SCORE (include/robot.hpp) a bit under the lowest score a real cup gets
//   line 3  C = cups it uses now, pin = other things it sees (ignored), g = boxes centered in the gears / claw
//           (FP_CLAW_CAM_MAX_CENTER_Y), x = cups further round than FP_AIM_AHEAD_DEG, m = AI_VISION_MIN_SCORE
//   One short rumble when it becomes sure of a cup.
//
// FIELD CAM (GOAL_VISION_PORT, front-left corner of the chassis: tags + cups)
//   line 1  the biggest AprilTag: its id, raw bearing (degrees right of the camera's own straight ahead) and distance
//           from its size (GOAL_TAG_SIZE_IN)
//   line 2  set YAW = the FP_GOAL_CAM_YAW that makes this tag read dead ahead of the robot IF the tag is placed dead
//           ahead of the robot's center line, 36" from the tracking center (include/auton-core.hpp explains it),
//           and sq = how square the tag faces the camera (the autos only trust >= FP_GOAL_TAG_MIN_SQUARENESS)
//   line 3  cup check: put ONE cup dead ahead of the robot's center line, D inches from the tracking center
//           (UP / DOWN change D in 6" steps, 12-36; it starts at 18 = 7.5" in front of the claw tip).
//           +x.xin = how far right of that spot the camera puts it (0 = right), Y = the FP_GOAL_CAM_YAW that would make
//           it 0. "2+" = more than one cup in view (the autos would skip), "--" = no cup this frame (average kept).
// =====================================================================
#include "auton-core.hpp"
#include "auton-select.hpp"
#include "pad.hpp"
#include "robot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#ifndef RW_SIM_PROS
#include "pros/ai_vision.hpp"
#endif

namespace {
constexpr int READ_MS = 100;
constexpr int AWB_WAIT_MS = 1500;        // white balance settling time
constexpr int WINDOW_FRAMES = 20;        // 2 s of readings
constexpr int RUMBLE_GAP_MS = 700;       // at most one buzz this often
constexpr double CUP_CHECK_IN = 18.0;    // field cam cup check: the cup this far straight ahead of the tracking center to
                                         // start with. The front of the claw's flex wheels is 8.6" ahead of the tracking
                                         // center (measured 2026-11-23), so D = 18 / 30 / 36 is 9.4 / 21.4 / 27.4" in
                                         // front of the flex wheels

// Degrees right of the camera's own straight ahead for a picture column (the same math as the autos' aiming)
double focalPx() { return (ac::FP_CAM_WIDTH_PX / 2.0) / std::tan(ac::rad(ac::FP_CAM_FOV_DEG / 2.0)); }
double columnDeg(double centerPx, double tiltDeg) {
    double tanAcross = (centerPx - ac::FP_CAM_WIDTH_PX / 2.0) / focalPx();
    tanAcross /= std::cos(ac::rad(tiltDeg));
    return std::atan(tanAcross) * 180.0 / ac::FP_PI;
}
// Two cup boxes closer together than this (degrees) are one cup: FP_AIM_SAME_IN at about a foot
double sameDeg() { return std::atan(ac::FP_AIM_SAME_IN / 12.0) * 180.0 / ac::FP_PI; }

// The cups one camera sees in one frame, filtered the way the autos filter them (cameraSighting in auton-core.cpp)
struct CupLook {
    int cups = 0;         // cups the autos would use
    int pins = 0;         // anything that isn't a cup: ignored
    int gears = 0;        // claw camera: boxes centered in the gears / claw
    int aside = 0;        // claw camera: cups further round than FP_AIM_AHEAD_DEG
    int weak = 0;         // cups under AI_VISION_MIN_SCORE
    int bestScore = -1;   // best cup score this frame, weak ones included (for setting AI_VISION_MIN_SCORE)
    double low = 1e9, high = -1e9, sum = 0;   // raw bearings of the cups used (degrees right of the camera's straight ahead)
    bool sure() const { return cups > 0 && high - low <= sameDeg(); }
    double raw() const { return cups > 0 ? sum / cups : 0.0; }
};

// The last 2 s of frames: how often it was sure, the average raw bearing of the sure frames, cup scores
struct Window {
    bool sure[WINDOW_FRAMES] = {};
    double raw[WINDOW_FRAMES] = {};
    int score[WINDOW_FRAMES] = {};
    int frame = 0;
    Window() { std::fill(score, score + WINDOW_FRAMES, -1); }
    void add(const CupLook& look) {
        sure[frame] = look.sure();
        raw[frame] = look.raw();
        score[frame] = look.bestScore;
        frame = (frame + 1) % WINDOW_FRAMES;
    }
    int surePercent() const { return static_cast<int>(std::count(sure, sure + WINDOW_FRAMES, true)) * 100 / WINDOW_FRAMES; }
    bool averageRaw(double& out) const {
        double total = 0;
        int n = 0;
        for (int f = 0; f < WINDOW_FRAMES; f++) {
            if (!sure[f]) continue;
            total += raw[f];
            n++;
        }
        if (n == 0) return false;
        out = total / n;
        return true;
    }
    bool scores(int& average, int& lowest) const {
        int total = 0, n = 0;
        lowest = 1000;
        for (int s : score) {
            if (s < 0) continue;
            total += s;
            n++;
            lowest = std::min(lowest, s);
        }
        if (n == 0) return false;
        average = total / n;
        return true;
    }
};

// One short rumble when it becomes sure of a cup: it has to stay sure 300 ms, and it only counts as gone after 1 s
// with no sure frame (the model drops out for a frame or two all the time)
struct Buzz {
    bool on = false;
    uint32_t sureSince = 0, lastSureMs = 0, lastRumbleMs = 0;
    void update(bool sure, uint32_t now) {
        if (sure) {
            if (now - lastSureMs > 200) sureSince = now;   // a new run of sure frames
            lastSureMs = now;
            if (!on && now - sureSince >= 300) {
                on = true;
                if (now - lastRumbleMs >= static_cast<uint32_t>(RUMBLE_GAP_MS)) {
                    pad::rumble(".");
                    lastRumbleMs = now;
                }
            }
        } else if (now - lastSureMs >= 1000) {
            on = false;
        }
    }
};

#ifndef RW_SIM_PROS
void whiteBalance(pros::AIVision& camera, const char* title) {
    pad::lines(title, "WHITE BALANCE...", "");
    camera.start_awb();
    pros::delay(AWB_WAIT_MS);
    std::printf("%s: white balance done\n", title);
}

bool cameraMissing(int32_t count, int port, const char* title) {
    if (count != PROS_ERR && count >= 0 && count <= 64) return false;
    pad::lines(title, "NOT CONNECTED", "");
    pad::line(2, "check port %d", port);
    return true;
}

CupLook lookForCups(pros::AIVision& camera, bool clawCam, int32_t count) {
    CupLook look;
    for (int32_t i = 0; i < count; i++) {
        pros::AIVision::Object object = camera.get_object(i);
        if (!(object.type & pros::E_AIVISION_DETECTED_OBJECT)) continue;
        const auto& box = object.object.element;
        if (clawCam && box.yoffset + box.height / 2 >= ac::FP_CLAW_CAM_MAX_CENTER_Y) {
            look.gears++;
            continue;
        }
        if (!ac::isCupClass(object.id)) {
            look.pins++;
            continue;
        }
        double center = box.xoffset + box.width / 2.0;
        if (clawCam ? ac::FP_PIN_CAM_UPSIDE_DOWN : ac::FP_GOAL_CAM_UPSIDE_DOWN) center = ac::FP_CAM_WIDTH_PX - center;
        const double raw = columnDeg(center, clawCam ? ac::FP_PIN_CAM_TILT_DEG : 0.0);
        if (clawCam && std::fabs(raw + ac::FP_PIN_CAM_YAW) > ac::FP_AIM_AHEAD_DEG) {
            look.aside++;
            continue;
        }
        look.bestScore = std::max(look.bestScore, static_cast<int>(box.score));
        if (box.score < AI_VISION_MIN_SCORE) {
            look.weak++;
            continue;
        }
        look.cups++;
        look.low = std::min(look.low, raw);
        look.high = std::max(look.high, raw);
        look.sum += raw;
    }
    return look;
}

// "CUP" (sure), "2+" (cups that disagree: the autos skip), "--" (no usable cup this frame)
const char* cupState(const CupLook& look) { return look.sure() ? "CUP" : (look.cups > 1 ? "2+" : "--"); }
#endif

void ownTheController(const char* name) {
    ac::setRouteFrame(false, false);
    ac::coreStart(name, 3600000);
    ac::showStatusOnController(false);
}
}  // namespace

// =====================================================================
//   CLAW CAM
// =====================================================================
void tuneClawCam() {
    ownTheController("claw cam");
#ifdef RW_SIM_PROS
    std::printf("Claw Cam: the simulator has no AI Vision sensor\n");
#else
    const char* title = "CLAW CAM";
    pros::AIVision camera(AI_VISION_PORT);
    camera.enable_detection_types(pros::AivisionModeType::objects);
    // The claw motor is left OFF (so it can't overheat): open the claw by hand before testing, like the autos read it
    claw.move_voltage(0);
    whiteBalance(camera, title);

    Window window;
    Buzz buzz;
    uint32_t lastLogMs = 0;
    while (!tuneExitWanted()) {
        if (controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) whiteBalance(camera, title);
        const uint32_t now = pros::millis();
        const int32_t count = camera.get_object_count();
        if (cameraMissing(count, AI_VISION_PORT, title)) {
            pros::delay(READ_MS);
            continue;
        }
        const CupLook look = lookForCups(camera, true, count);
        window.add(look);
        buzz.update(look.sure(), now);

        double averageRaw = 0;
        const bool haveAverage = window.averageRaw(averageRaw);
        const double average = averageRaw + ac::FP_PIN_CAM_YAW;
        if (haveAverage) pad::line(0, "%-3s %3d a%+.1fd", cupState(look), look.bestScore, average);
        else pad::line(0, "%-3s no cup yet", cupState(look));
        int averageScore = 0, lowestScore = 0;
        if (window.scores(averageScore, lowestScore)) pad::line(1, "sure%3d%% av%d lo%d", window.surePercent(), averageScore, lowestScore);
        else pad::line(1, "sure  0%% no cups");
        pad::line(2, "C%d pin%d g%d x%d m%d", look.cups, look.pins, look.gears, look.aside, AI_VISION_MIN_SCORE);

        if (now - lastLogMs >= 1000) {
            lastLogMs = now;
            std::printf("Claw Cam: %s, cups %d (weak %d), pins %d, gears %d, aside %d, sure %d%% of 2 s, average bearing %+.2f deg\n",
                        cupState(look), look.cups, look.weak, look.pins, look.gears, look.aside, window.surePercent(),
                        haveAverage ? average : 0.0);
        }
        pros::delay(READ_MS);
    }
#endif
}

// =====================================================================
//   FIELD CAM
// =====================================================================
void tuneFieldCam() {
    ownTheController("field cam");
#ifdef RW_SIM_PROS
    std::printf("Field Cam: the simulator has no AI Vision sensor\n");
#else
    const char* title = "FIELD CAM";
    pros::AIVision camera(GOAL_VISION_PORT);
    camera.enable_detection_types(pros::AivisionModeType::objects, pros::AivisionModeType::tags);
    whiteBalance(camera, title);
    // The bearing a tag dead ahead of the robot, 36" from the tracking center, has from this camera
    const double shouldDeg = std::atan2(-ac::FP_GOAL_CAM_RIGHT, 36.0 - ac::FP_GOAL_CAM_FORWARD) * 180.0 / ac::FP_PI;
    double checkIn = CUP_CHECK_IN;   // the cup check's distance (UP / DOWN)
    Window window;
    Buzz buzz;
    uint32_t lastLogMs = 0;

    while (!tuneExitWanted()) {
        if (controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) whiteBalance(camera, title);
        if (controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) checkIn = std::min(checkIn + 6.0, 36.0);
        if (controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) checkIn = std::max(checkIn - 6.0, 12.0);
        // A cup dead ahead, checkIn from the tracking center: the bearing it should have from this camera, and how far it is
        const double cupShouldDeg = std::atan2(-ac::FP_GOAL_CAM_RIGHT, checkIn - ac::FP_GOAL_CAM_FORWARD) * 180.0 / ac::FP_PI;
        const double cupReach = std::hypot(ac::FP_GOAL_CAM_RIGHT, checkIn - ac::FP_GOAL_CAM_FORWARD);
        const uint32_t now = pros::millis();
        const int32_t count = camera.get_object_count();
        if (cameraMissing(count, GOAL_VISION_PORT, title)) {
            pros::delay(READ_MS);
            continue;
        }
        int tagId = -1;
        double tagTall = 0, tagWide = 0, tagCenter = 0, tagSize = 0;
        for (int32_t i = 0; i < count; i++) {
            pros::AIVision::Object seen = camera.get_object(i);
            if (!(seen.type & pros::E_AIVISION_DETECTED_TAG)) continue;
            const auto& t = seen.object.tag;
            // The pair of edges that runs across the picture and the pair that runs up it (as the autos do)
            const double e01 = std::hypot(t.x1 - t.x0, t.y1 - t.y0), e12 = std::hypot(t.x2 - t.x1, t.y2 - t.y1);
            const double e23 = std::hypot(t.x3 - t.x2, t.y3 - t.y2), e30 = std::hypot(t.x0 - t.x3, t.y0 - t.y3);
            const bool firstPairAcross = std::abs(t.x1 - t.x0) + std::abs(t.x3 - t.x2) >= std::abs(t.y1 - t.y0) + std::abs(t.y3 - t.y2);
            const double wide = firstPairAcross ? (e01 + e23) / 2.0 : (e12 + e30) / 2.0;
            const double tall = firstPairAcross ? (e12 + e30) / 2.0 : (e01 + e23) / 2.0;
            if (tall > tagTall) {
                tagId = seen.id;
                tagTall = tall;
                tagWide = wide;
                tagCenter = (t.x0 + t.x1 + t.x2 + t.x3) / 4.0;
                tagSize = std::max({e01, e12, e23, e30});
            }
        }
        if (ac::FP_GOAL_CAM_UPSIDE_DOWN) tagCenter = ac::FP_CAM_WIDTH_PX - tagCenter;
        double rawDeg = 0, squareness = 0;
        if (tagId >= 0) {
            rawDeg = columnDeg(tagCenter, 0.0);
            squareness = std::min(tagWide / std::max(tagTall, 1.0), 1.0);
            const double inches = tagSize > 0.5 ? focalPx() * GOAL_TAG_SIZE_IN / tagSize : 0.0;
            pad::line(0, "TAG%d %+.1fd %.0fin", tagId, rawDeg, inches);
            pad::line(1, "set YAW %.1f sq%.2f", shouldDeg - rawDeg, squareness);
        } else {
            pad::line(0, "no tag");
            pad::line(1, "YAW now %.1f", ac::FP_GOAL_CAM_YAW);
        }

        const CupLook look = lookForCups(camera, false, count);
        window.add(look);
        buzz.update(look.sure(), now);
        double cupRaw = 0;
        const bool haveCup = window.averageRaw(cupRaw);
        const double cupOff = cupReach * std::tan(ac::rad(cupRaw + ac::FP_GOAL_CAM_YAW - cupShouldDeg));
        if (haveCup) pad::line(2, "%.0f %-3s%+.1f Y%.1f", checkIn, cupState(look), cupOff, cupShouldDeg - cupRaw);
        else pad::line(2, "%.0f %-3s no cup yet", checkIn, cupState(look));

        if (now - lastLogMs >= 1000) {
            lastLogMs = now;
            if (tagId >= 0) {
                std::printf("Field Cam: tag %d raw %+.2f deg, squareness %.2f, %.0f px tall -> FP_GOAL_CAM_YAW %.2f if it's dead ahead at 36\"\n",
                            tagId, rawDeg, squareness, tagTall, shouldDeg - rawDeg);
            }
            std::printf("Field Cam: cups %s (%d, pins %d, weak %d), sure %d%% of 2 s", cupState(look), look.cups, look.pins, look.weak,
                        window.surePercent());
            if (haveCup) {
                std::printf(", raw %+.2f deg, cup %+.2f in right of the spot %.0f\" ahead -> FP_GOAL_CAM_YAW %.2f", cupRaw, cupOff, checkIn,
                            cupShouldDeg - cupRaw);
            }
            std::printf("\n");
        }
        pros::delay(READ_MS);
    }
#endif
}
