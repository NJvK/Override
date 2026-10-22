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
- Old brain-screen readouts now show on the controller during driver control. Pick which set with `CONTROLLER_READOUT` in main.cpp.
- The Makefile was LemLib's library Makefile (`IS_LIBRARY:=1`, `LIBNAME:=LemLib`). It was replaced with a normal robot-program Makefile.

## Field frame
- Walls are at ±70.25 (RW's `field_half_size`).
- Start spots:
  - Skills: (-61.25, 0, 90)
  - Quals, right wall: (61.25, 0, 270)
  - Quals, top wall: (0, 61.25, 180)
- Skills uses the field element positions from RW's Override field model, which is also what its simulator draws:
  - Goals at (±47.09, ±23.55) and (±23.55, ±47.09)
  - Matchloader centered at y = ±58.7, with its face 3.75" off the wall (x = ±66.5)
- The qual autons still use 24" tile multiples (48, 24).
- All field-tuned auton offsets were reset to 0.

## Odometry and PID values (in rw-config.cpp)
- Track width: 12.625
- Tracking wheels:

| Wheel | Port | Diameter | Offset |
|---|---|---|---|
| Vertical | -14 | 2.01258746176 | -0.1 |
| Horizontal | 13 | 2.00836820084 | 2.25 |

- Drive PID: kp 0.55, kd 0.57. Turn PID: kp 0.35, kd 2.41. These are the LemLib gains × 12/127.

## Skills route rules (Oct 21 rewrite)
- **The robot never touches the field** except:
  - at the start, with its back against the wall
  - during the end toggle sweep
- **Start:** after the toggle finishes, it drives 3.9" straight forward before turning. This keeps the back corners from hitting the wall.
- **Path into the matchloader:** all straight lines.
  - Turn spot T is at (-40.4, -55.7).
  - From T, a shallow line (about 18°) goes to line-up spot P at (-49, -58.7).
  - At P it makes a small turn to 270, then drives dead straight in.
  - The claw stops `SKILLS_LOADER_GAP` (1") short of the loader face.
  - It leaves the same way.
- **At the loader:** it holds for 1.5 s and lifts 15°, then moves.
- **Park:** a braked stop, measured from where the sweep touched the toggle. The back of the robot stops 2" short of the center goal base.
- **In the sim:** 3 goal A loads and 3 goal B loads, finishing at 55.6 s. There's no contact except the toggle.
- **Tightest clearance in the sim:** 1.2" to the bottom wall when turning at T.

## Bring-up order
1. Odom Check
2. Tuning Test
3. Square Test
4. IMU Scale (sets imu_scale)
5. Skills

## Distance sensors and MCL
Sensors are on placeholder ports 6, 8, 15 and 16. Set up MCL with the 5-step LOCALIZATION block in rw-config.cpp:
1. Mount the sensors.
2. Set the sensor offsets.
3. Run Sensor Check.
4. Set `use_distance_resets = true`.
5. Set `use_mcl = true`.

RW's MCL only knows the walls. Readings that hit goals are treated as "cut short", but in a sim test with made-up sensor spots they still pulled the position about 2" off near goal B. Mount sensors so they don't look at goals along the route.

## Simulator
Run from the project folder:
- Skills: `sh sim/run-sim.sh "Skills" --start -61.25,0,90 --time 60 --size 18`
- Quals: `--start 61.25,0,270 --time 15`

How well the sim matches the real robot:
- It doesn't model the DR4B lift, so lift waits run to their timeouts.
- It only blocks the robot at the walls. Goals and loaders are drawn but don't stop it.
- Run "Drive Calibration" on the real robot and fit a profile so the sim's drive matches the robot.

## Known items
- In PROS 4, `DR4B.set_brake_mode(...)` sets only the first motor (port -7). `set_brake_mode_all` sets both. This was left as it was.
- The qual autons (auton.cpp, autonmirror.cpp) wrap headings with `fmod(x + 540, 360) - 180`. That breaks once the heading has counted past 540°. Skills was switched to `std::remainder`; the quals were left as they were.