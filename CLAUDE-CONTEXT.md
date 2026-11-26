# Context for a new chat (from the 2026-11-23 tuning session)

Paste this to start: "Read CLAUDE-CONTEXT.md in OverrideNewRW and continue from where it leaves off."

## Project
- VEX V5 robot code (PROS + RW template), folder `OverrideNewRW`, branch `New-RW-Code`. `Override` next to it is the old LemLib project; ignore it.
- Match autos: `src/skills.cpp` (60 s), `src/fourpin.cpp` (15 s quals). Shared drive / lift / claw / vision code: `src/auton-core.cpp` + `include/auton-core.hpp` (all `FP_*` settings). Robot config: `src/rw-config.cpp`, `include/robot.hpp`.
- Tuning tools: `src/tuning.cpp`, `src/ai-vision-tuning.cpp`, listed in `ROUTINES` in `src/auton-select.cpp`.
- Build: PowerShell with `$env:TMP = $env:TEMP = "$env:LOCALAPPDATA\Temp"` and the PROS toolchain on PATH (`%APPDATA%\Code\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr\bin` and `...\pros-cli-windows`), then `make`. The simulator runs in WSL: `sh sim/run-sim.sh "<routine>" --start x,y,heading --time N`. It has no cameras (fake them with `SIM_PIN_SHIFT` / `SIM_GOAL_SHIFT` / `SIM_CUP`). Details are in Claude's memory files.
- Python is `py -3`. Edit files that scripts have changed with small Python replace scripts or after a fresh Read.

## How the user works
- All readouts go on the **controller** (`pad::line`, 3 lines x 19 chars), never the brain screen.
- Don't trust the simulator alone. Say plainly what was only simulated and what was tested on the robot.
- Explain changes in plain words, with the numbers the user should send back.

## Done in the 2026-11-23 session (built, not committed)
- **Bench mode** (`src/auton-select.cpp`, `src/main.cpp`): with nothing connected, in driver control LEFT/RIGHT pick a routine and Y runs it. Hold B to end a tuning tool. Once field control or a competition switch has been seen, bench mode is off until the program restarts. `driverSetup()` in main.cpp resets the robot after a bench run.
- **imu_scale = 1.00385** (3 hand-spin runs: 1.00315 / 1.00453 / 1.00387).
- **Odom Check:** tracking wheel signs and offsets are good (a full spin returns X/Y to about 0). Distance read ~1.5-2% high, but that's the user's non-VEX mat, so it was **not changed**. Redo the 96" push on VEX tiles.
- **Claw camera** (port 5): `FP_PIN_CAM_YAW = 2.0` (a cup reads 0.0 deg at 12"). Filters:
  - cups only (`FP_CUP_CLASS_IDS = {4}`; pins are 2 and 3 and ignored; a stack is a separate cup + pin, so its cup counts)
  - boxes centered at row >= 160 are the gears or claw (`FP_CLAW_CAM_MAX_CENTER_Y`)
  - read only with the claw open, from 200 ms after it starts opening (`FP_CLAW_CAM_OPEN_MS`)
  - within ±12 deg of straight ahead
  - range 18" from the camera (`FP_AIM_MAX_DIST`)
  - no detection at 4" in front of the claw (normal)
- **Field camera** (port 21): lens at 6.625" forward / 5.125" left (`FP_GOAL_CAM_FORWARD/RIGHT`). `FP_GOAL_CAM_YAW = 33.5` (was 23.58; cup checks at 18"/30" gave 34.0/33.1). Cup aiming on, range 20" (`FP_FIELD_PIN_MAX_DIST`). **The tag check at 36" has NOT been done yet (no goal), and goal approaches use this angle.**
- **Aiming rules** (`cameraSighting` in auton-core.cpp):
  - sideways correction only, at most 3" (`FP_AIM_MAX_SHIFT`)
  - only a cup within 3" of its expected spot (`FP_PIN_WINDOW_IN`; the 4-pin wall cup uses 2.5)
  - exactly one cup; cups more than 1.5" apart (`FP_AIM_SAME_IN`) or none = skip that frame
  - distance always comes from odometry + MCL, never from the cameras
- **`AI_VISION_MIN_SCORE = 55`** (`include/robot.hpp`). Real cups scored at least 62; nothing fake is detected with an empty view.
- **The drive never slows for sensors.** Distance resets at goals now run inside the existing release/flip waits (`letGo` in auton-core.cpp, goal A preload in skills.cpp).
- **Tools:**
  - Claw Cam: claw motor off; the user opens the claw by hand to avoid overheating.
  - Field Cam: cup check at a distance set with UP/DOWN.
  - New **Vision Pickup**: both cameras locate the cup by triangulation (`locateCup`), then the real `pickUp()` grabs it. The claw motor is off while it waits.
- Measured but deliberately left alone, because the routes were tuned with the old values:
  - front of the flex wheels = 8.6" from the tracking center (code `FP_CLAW_TIP_FORWARD` = 10.5)
  - cup middle in the claw = 6.75" (code `FP_CLAW_REACH` = 7.0)
- Simulator after all changes: Skills 50.4 s, 4P Red Bottom 14.7 s, camera corrections on every pickup.

## Still to do
1. **Tag check** with a goal: tag dead ahead, 36" from the tracking center (27.4" in front of the flex wheels). Field Cam line 1 `set YAW` should be about 33.5. Do this before a real match run.
2. On VEX tiles / a field: the 96" odom push (tracking wheel diameter, `wheel_distance_in`), `distance_sensor_scale`, the distance sensor face offsets, and "Sensors + MCL" at the Skills start. The distance sensor ports in rw-config.cpp still carry a TODO; check that all 4 read.
3. Lift Test after the user adds rubber bands (`FP_LIFT_UP_DPS`, `FP_LIFT_START_S`).
4. Feedforward / Drive / Spin / Curve tests once the middle left drive motor (port 1, removed for now) is back, on VEX tiles.
5. Optional: more Vision Pickup runs. The user saw good pickups straight ahead, and misses about 2" sideways / early closes when the cup is more than 4" off-center. In the autos, distance comes from odometry, so that's expected to be much better.
6. Commit the session's changes when the user asks (not committed yet).

## Done in the 2026-11-24 session (built + simulated, NOT run on the robot, not committed)
- **Stop before scoring fixed** (`runScore` in auton-core.cpp): it waited up to `FP_GATE_MAX_MS` (1.5 s) at a goal for the
  lift to get within 15 deg of its height, then let go anyway. Now `liftUpOrStuck()`: it scores as soon as the lift is
  there OR has stopped climbing (rose < `FP_LIFT_RISE_DEG` 2 deg in `FP_LIFT_STUCK_MS` 120 ms). The lift gate on the
  way in uses the same test (no 8 in/s creep behind a lift that topped out). The terminal prints
  "lift stopped at X (wanted Y)" when that happens: tells which height in main.cpp `ALLIANCE_HEIGHTS` the lift can't reach.
- **New skills ending** (`middleGoalFinish` in skills.cpp, settings `SK_MID_*`, switch `SK_MIDDLE_GOAL_FINISH`):
  toggle -> 8" forward -> spin to 205 -> back into the red cluster pin (tip 17,-23.55, heading 228, 4" in, same as the
  4-pin) with the intake, no dwell (`SK_MID_PIN_DWELL_MS` 0) -> curve out to (8,-41) -> spin to 30 -> up between the small red
  pin (0,-23.55) and the cluster into the middle goal at heading 328, lift at spot 3, claw opens 8" before -> flipper
  1 s -> back off to 17" from the goal + 35 deg swing -> between the middle goal and the small blue pin (23.55,0) ->
  yellow stack (23.55,23.55) at heading 35 -> round it -> middle goal at heading 215, lift at spot 4 -> stops there
  holding it (`ScorePlan::release = false`; no end dash once it goes for the stack).
- Time guard: middle goal only if `SK_SIM_MID_FLIP_MS` (6.2 s) fits after the toggle, the stack only if
  `SK_SIM_MID_STACK_MS` (3.8 s, no spare) fits after the flip; otherwise it parks / stays at the goal.
- Clearances were planned with a 15" chassis (16.5" at the gears), pins 1.3" radius, goal base 3.5" radius. Tightest in
  the sim: ~0.3-0.6" past the small red pin at the back-off, ~1.5" past it on the way in, ~1.8" past the small blue pin.
- Follow-up same day: the user asked to curve straight from the cluster pin into the middle goal (no drive-then-turn).
  Checked: turning right at/after the pin sweeps the front into the small red pin (0,-23.55) (pivot at the pin = ~0.2"
  clearance), a left loop hits the cluster, a loop round the left of the small pin = ~0.5" and enters the goal from the
  lower left (stack trip much longer). Kept out + turn (shortened). Added `ScorePlan::liftNotBeforeMs`: the lift stays
  down until 1.5 s (`SK_MID_FEED_MS`) after the pin so the intake can feed the flipper. Waiting on the user's answer.
- Then the user said knocking the small red pin over is fine, but with the robot's SIDE while curving into the cluster
  pin (not the back: it would jam the intake), and to curve straight from the cluster pin into the middle goal. Done:
  10" off the toggle, spin to 190, back through (8,-30) heading 205 and swing into the cluster pin at heading 242 (3" in);
  the swing sweeps the right side (3-6" ahead of center, the gear step) across the small pin ~3" before the cluster pin,
  planned to still be the side with 1"/3 deg error. Then one forward curve into the middle goal at heading 336 (tangent 12),
  lift held down 1.5 s after the pin. Sim: pin 51.8 s, flipped 55.9 s, stack then doesn't fit in the sim (needs 3.9 s).
  Scratchpad planners (plan.py, knock.py, robust2.py) did the clearance/contact checks; they're temporary.

## 2026-11-25: lift bottom hold (built, NOT tested on the robot, not committed)
- Problem: at the bottom the lift rose ~0.5" (motors off in the 0-10 deg deadzone, bands pull it up), and holding L2
  there jiggled (`move_velocity(0)` fighting the bands).
- Fix in main.cpp: `bottomHoldVoltage()` presses it onto the hard stop (1200 mV + 200 mV/deg, max 4000;
  `DR4B_BOTTOM_HOLD_*`). Used by the PID task's bottom deadzone, by L2 at the bottom, and after the stall zero.
  Letting go of L2 at the bottom skips the 250 ms brake/settle and sets target 0 straight away.
- Robot test: jiggle gone. The "L 13.7 / 26.8" the user saw was the LEFT DISTANCE SENSOR (CONTROLLER_READOUT was 3),
  not the lift. A 30 deg L2 re-zero added because of that was reverted. CONTROLLER_READOUT set to 0 (lift "L H C").

## 2026-11-25: skills start lift, loader approach, bench mode removed (built + simulated, NOT run on the robot, not committed)
- Start: lift goes up to spot 2's height (240, where P1 goes onto goal A), back down to 20, then the shove (`SK_TOGGLE_LIFT_SPOT`).
- Loader approach: the turn spot stays 1.5" above the line (a back corner would hit the bottom wall mid-turn on the
  line). The turn now points the nose a little below the line (`spinForLoader`), so the drive in is one gentle right
  curve onto 270, no S-curve / sudden left turn. Loader camera kept ON (the turn was seen on a plain floor: no cups).
  `SK_LOADER_WAIT_MS` 500 -> 250 to fit the stack. Sim: meets the loader at 270 every trip; full run (stack on the
  middle goal) at 59.2 s; slow profile parks at 56.8 s.
- Bench mode (Y runs the pick) removed: autons/tools run ONLY from field control / a competition switch.
  `tuneExitWanted()` is always false now (tools end with autonomous).
