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
