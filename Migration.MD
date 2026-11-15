# RW-Template migration notes (Oct 2026)

Robot code moved from LemLib 0.5 to RW-Template 4.1 (PROS kernel 4.2.1, liblvgl 9.2), so the robot can use Monte Carlo localization (MCL) and the autonomous simulator.

## What changed
- Driver control works exactly as before. main.cpp copies LemLib's arcade curve: deadband 3, min output 10, gain 1.019, desaturate 0.5.
- The autons still use `chassis.*` calls. These now go through a bridge (`rw-chassis.hpp/.cpp`) that maps them to RW:
  - turnToHeading → turnToAngle
  - moveToPoint → moveToPoint
  - moveToPose → boomerang
  - tank → driveChassis
- Drive, odometry, PID and localization settings are all in `src/rw-config.cpp`. The old LemLib tuning block is gone.
- Autons are picked from the brain-screen selector, or with the controller arrows while disabled. The choice is saved to the SD card.
- The selector list is set in `runPreAutonomous()` in `src/rw-user.cpp`.
- Tests only run when autonomous starts: from a competition switch, field control, or the controller's Timed Run.
- Old brain-screen readouts now show on the controller during driver control. Pick which set with `CONTROLLER_READOUT` in main.cpp.
- The Makefile was LemLib's library Makefile (`IS_LIBRARY:=1`, `LIBNAME:=LemLib`). It was replaced with a normal robot-program Makefile.

## Field frame
- Walls are at ±70.25 (RW's `field_half_size`).
- Robot: 16.35 long (center to back 8.3, center to front 8.05), 16.78 wide.
- Start spots:
  - Skills: (-61.95, 0, 90)
  - Quals: Red Left (-61.95, 0, 90), Red Bottom (0, -61.95, 0), Blue Right (61.95, 0, 270), Blue Top (0, 61.95, 180)
- Skills uses the field element positions from RW's Override field model, which is also what its simulator draws:
  - Goals at (±47.09, ±23.55) and (±23.55, ±47.09)
  - Matchloader centered at y = ±58.7, with its face 3.75" off the wall (x = ±66.5)
- The qual opening (src/quals.cpp → `runOpeningRoute` in skillsauto.cpp) reuses the skills toggle, preload and field-pin code.

## Odometry and PID values (in rw-config.cpp)
- Track width: 12.625
- Tracking wheels:

| Wheel | Port | Diameter | Offset |
|---|---|---|---|
| Vertical | -14 | 2.01258746176 | -0.1 |
| Horizontal | 13 | 2.00836820084 | 2.25 |

- Drive PID: kp 0.55, kd 0.57. Turn PID: kp 0.35, kd 2.41. These are the LemLib gains × 12/127.
- imu_scale: still 1.0 (TODO: run the IMU Scale test).
- Distance sensors: front 6, left 15, right 18, back 16. Back sensor reads 3.9" with the back flush on a wall.

## Skills route rules (current, Oct 24)
- **Never touches the field** except at the start (back against the wall) and the toggles. The end toggle #2 is rammed on purpose.
- **Start:** toggle lift 55° with a fixed 200 ms head start. The drive off the wall goes to 6.5" from the start spot (not from where the robot is), so a push off the wall by the lift doesn't move the turn spot.
- **Field pins:** skills eases from 60 to 20 between 14" and 8" from the grab spot, then 20 the rest of the way. Quals use the same gentle approach (`QUALS_GENTLE_PIN_APPROACH = true`).
- **Pickups:** fixed 1.2 s, claw at full speed.
  - Check during the pickup: claw ≥ 140 rpm for 80% of readings after 0.6 s = nothing grabbed. Then the lift doesn't kick up and it skips scoring.
  - Hold check afterward (≤ 50 rpm at the 100 rpm hold speed, 1 s) catches a pin that falls out later.
- **Loader line (Oct 24):** `SKILLS_LOADER_RIGHT_ADJUST` went from -1.5 to -0.75 and `SKILLS_LOADER_EXIT_WALL_SPARE` from 0.75 to 1.5. At -1.5 the robot's side ran only 1.66" off the bottom wall, and every loader trip brought a corner 0.7–1" from it in the sim. Now every trip keeps 1.5"+. It stops 0.75" further right at the loader, so if the claw misses the loader pin to the right, move both back toward the old values.
- **Missed matchload:** backs out to T with the lift down. The next trip clears the loader: from T straight in at T's height until the front is 4" from the loader face, sweep right (~24°), square, back to T, normal approach, pickup. On the loader line itself only ~4° of turn fits.
- **Pin lost with the lift up:** first drives toward the goal until every corner is 8" off the walls, then opens and lowers, then waits a fixed 600 ms before driving. (The DR4B swings past the frame while partly up and hooked the wall.)
- **Missed pin 2:** the next trip to T drives forwards.
- **Loader exit:** the climb back to T eases off (12°/inch still to climb), so it levels out at T's height. It used to overshoot T by ~2".
- **Before the sweep:** on the goal B side it backs to T for turning room. Room-to-turn moves go away from the goal along the heading, never toward it.
- **End:**
  1. The right corner hits toggle #1. It stops 1.5" off the wall by odometry.
  2. A full-power left-side swing rams toggle #2. It stops at 200°, when jammed, or after 600 ms.
  3. It backs at the park spot (toggle X, back of the robot 3" short of the center goal), steering, with a braked stop.
- **Sim (Oct 24):** in Claude's sim copy it parks at 56.9 s with goal A 6 and goal B 1. A missed matchload costs about 6 s.
  - In the team's own project sim it parks at about 59 s. Each `fieldAwareReset` there takes about 0.12 s, versus instant in Claude's copy, and there are about 20 per run. The route is the same.
  - The time guard (`SKILLS_PIN_ONLY_CYCLE_MS` 7500) still fits the goal B pin there. At 8500 it skipped goal B and parked at about 51.5 s.

## Qual autons (current, Oct 24)
- Four routes in src/quals.cpp (Red Left, Red Bottom, Blue Right, Blue Top), each with its own tuning, all running `runOpeningRoute`: toggle, preload, field pin 1, field pin 2, all on the alliance goal. auton.cpp / autonmirror.cpp are thin wrappers that call the Blue Right / Blue Top routes.
- Qual-only timing lives in skillsauto.cpp's `QUALS_` block. Skills keeps its own values.
  - Pickups: 600 ms.
  - Scoring after the preload is quicker: turn speed 127, accel 4.5, decel 7, lowering slew 700.
  - Pin 1: lowers for 250 ms, opens the claw, waits 100 ms, then backs off 12.5".
- **End of the qual autons:**
  - Pin 2 keeps the claw closed, and nothing waits for the lift. As soon as the claw is over the goal, `skillsCoastLift(true)` turns the lift PID off (`dr4bManualMode = true`) and puts both lift motors on coast. At the same moment it pushes the lift down at 3000 mV for 150 ms to get it falling (a lift on coast can stay put), then cuts the power to 0 V. The lift then falls onto the goal under its own weight, and keeps falling after auton ends.
  - `runOpeningRoute` also calls `skillsCoastLift()` at the end if pin 2 was missed or dropped.
- **Driver start:** the first thing `opcontrol()` does is turn the lift PID back on (`set_brake_mode_all(COAST)`, then `holdDR4B()`). The PID holds the lift wherever it ended up.
- **Sim (Oct 24):** the claw is over the goal at 14.69–14.86 s and auton ends at 14.80–14.96 s (after the push), depending on the route. The real robot is about 5–10% slower, so it can get cut off just short of the goal. In that case the lift stays up holding pin 2, and the driver scores it.
- The RW sim keeps recording for 1 s after auton ends. The robot standing still at the end of a replay is that extra second, not the code.

## Bring-up order
1. Odom Check
2. Tuning Test
3. Square Test
4. IMU Scale (sets imu_scale)
5. Skills

## Distance sensors and MCL
Sensors are plugged in and the ports are set. Set up MCL with the 5-step LOCALIZATION block in rw-config.cpp:
1. Measure the sensor offsets.
2. Run Sensor Check and confirm which port is which (hand in front of each sensor).
3. Tape-check the readings and set `distance_sensor_scale`.
4. Set `use_distance_resets = true`, then Sensor Check at the skills start. X/Y should stay at about -61.95, 0.
5. Set `use_mcl = true`. Do the pick-up-and-move-3" test: MCL should snap X/Y back.

Skills uses the field-aware MCL and `fieldAwareReset(3.0)` (src/field-mcl.cpp) at every stop. Keep people and bags away from the walls during runs: a beam that clears the wall reads them, and errors under 3" get accepted.

## Simulator
Run from the project folder:
- Skills: `sh sim/run-sim.sh "Skills" --start -61.95,0,90 --time 60 --size 16.8`
- Quals: `--start 61.95,0,270 --time 15` (use the start spot of the route you're running)
- `--ideal` turns off slip and noise.
- Pretend pickup failures (pickup numbers: 1 = pin 1, 2 = pin 2, 3+ = loader):
  - `SIM_MISS=3` grabs nothing.
  - `SIM_DROP=3` grabs, then loses it 0.3 s later.
- The sim needs stand-ins for `fieldAwareReset` and the `OpeningRoute` struct if the project's own files aren't copied in.

How well the sim matches the real robot:
- The claw never stalls, so pickup checks rely on SIM_MISS / SIM_DROP.
- It only blocks the robot at the walls, as a center clamp, so a corner can go 3" into a wall. Goals and loaders are drawn but don't stop it.

## Known items
- In PROS 4, `DR4B.set_brake_mode(...)` sets only the first motor (port -7). `set_brake_mode_all` sets both. Most of the code still uses `set_brake_mode`. The qual end coast and the driver-start PID re-enable use `set_brake_mode_all`.

## Robot rebuild (Nov 2026)
- **Lift:** one motor (port 2). Rotation sensor on port 4, reversed in `initialize()`.
- **Claw:** full 11 W motor (port 9). Driver hold is by voltage (`CLAW_HOLD_VOLTAGE` 2000 mV).
- **Intake:** half motor, port 7, on the back of the robot. X = forward on/off, B = reverse on/off.
- **Pin flipper:** half motor, port 3, on the lift's end effector. Hold A = flip (75 rpm), let go = back down (50 rpm) and off when stalled. Up/down arrows override at 200 rpm.
- **Gone:** the toggle mechanism (`toggleMechanism` sits on dummy port 0 because skillsauto.cpp still uses the name), the optical sensor and the auto-height feature.
- **AI Vision:** pin camera on port 5 (on the drivetrain under the claw, centered, facing forward). A second one is planned for port 21 (goal AprilTag).
- **Not updated for the rebuild yet:**
  - skillsauto.cpp and quals.cpp still run the toggle step and use fixed lift times tuned for two lift motors.
  - rw-config.cpp still has the old robot's sizes (distance sensor offsets, tracking wheel offsets, track width). MCL uses the sensor offsets.

## 4-pin qual auton (Nov 14)
- **Files:** `src/fourpin.cpp` (route, curve follower, mechanisms, tip guard, vision, 15 s timer) and `include/fourpin.hpp`. It shares nothing with skillsauto.cpp and needs no change in main.cpp.
- **Selector:** "4Pin Red Bot" and "4Pin Blue Top" replaced "Qual Left" (a duplicate of "Red Left"; the selector holds 12). `fourPinRedLeft` and `fourPinBlueRight` exist in fourpin.cpp but aren't on the selector.
- **Route (red, bottom wall toggle, start (0, -61.25, 0)):**
  1. Lift up 50° and straight back down, then drive off the toggle.
  2. Curve left into the red goal (-23.55, -47.09), claw open, lift down. Flip the preload onto the goal.
  3. Back up into the red pin of the cluster at (23.55, -23.55) with the intake (back of the robot). Short wait.
  4. Forward around the small pin at (0, -23.55), through pin 3 (-23.55, -23.55) without stopping, left into the goal with the lift going up. Let go.
  5. Back out, swing the nose, through pin 4 (-47.09, -47.09), left around into the goal, one height higher.
  6. Back out, into the left wall cup (-26.2, -68.7) and the wall with the claw spinning closed.
  7. Back off with the lift at 30°, flip the intake pin into the cup, drive into the goal, flipper back, let go, lower. Lift goes limp at 14.85 s.
- **How it drives:** every move is a curve through "nodes" (spot + heading). The follower plans speed ahead and steers with the curve's bend plus a correction. It stops only at the goal, at the intake pin, and where it switches between forwards and backwards.
- **Sizes are estimates:** 18 long, 17 wide, claw reach 7.5" from center, intake reach 10.5". All at the top of fourpin.cpp.
- **Sim (Nov 14):** `sh sim/run-sim.sh "4Pin Red Bot" --start 0,-61.25,0 --time 15 --size 18` (Blue Top: `--start 0,61.25,180`).
  - Done at about 11.1 s. Every score lands within 0.7" of the goal center. No contact with goals, pins or walls in 7 seeds and all four starts.
  - With a slower, slippier drive profile it finishes at 13.2 s.
  - The sim only models driving. Lift, claw, flipper and intake times are guesses until they run on the robot.
- **Open problems found in the sim:**
  - **Wall cup:** with an 18 × 17 body the front corner reaches the wall while the claw is still about 5" from the cup (heading 140). Coming in squarer puts the back corner into the goal. Whether it gets the cup depends on how far the real claw sticks out.
  - **Cluster pin:** the robot can't line up straight with the pin (heading 270). Its nose would sit on the small pin at (0, -23.55), so it backs in at 228. Clearances there are under 1".
- **Vision:** pin camera aiming is on (`FP_USE_PIN_VISION`). It nudges the curve sideways by up to 3". Goal AprilTag aiming is written but off (`FP_USE_GOAL_TAG`).