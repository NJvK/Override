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
  - Quals, right wall: (61.95, 0, 270)
  - Quals, top wall: (0, 61.95, 180)
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
- **Field pins:** skills eases from 60 to 20 between 14" and 8" from the grab spot, then 20 the rest of the way. Quals keep the old approach (`QUALS_GENTLE_PIN_APPROACH = false`): the gentle one costs about 1 s for the two pins.
- **Pickups:** fixed 1.2 s, claw at full speed.
  - Check during the pickup: claw ≥ 140 rpm for 80% of readings after 0.6 s = nothing grabbed. Then the lift doesn't kick up and it skips scoring.
  - Hold check afterward (≤ 50 rpm at the 100 rpm hold speed, 1 s) catches a pin that falls out later.
- **Missed matchload:** backs out to T with the lift down. The next trip clears the loader: from T straight in at T's height until the front is 4" from the loader face, sweep right (~24°), square, back to T, normal approach, pickup. On the loader line itself only ~4° of turn fits.
- **Pin lost with the lift up:** first drives toward the goal until every corner is 8" off the walls, then opens and lowers, then waits a fixed 600 ms before driving. (The DR4B swings past the frame while partly up and hooked the wall.)
- **Missed pin 2:** the next trip to T drives forwards.
- **Loader exit:** the climb back to T eases off (12°/inch still to climb), so it levels out at T's height. It used to overshoot T by ~2".
- **Before the sweep:** on the goal B side it backs to T for turning room. Room-to-turn moves go away from the goal along the heading, never toward it.
- **End:**
  1. The right corner hits toggle #1. It stops 1.5" off the wall by odometry.
  2. A full-power left-side swing rams toggle #2. It stops at 200°, when jammed, or after 600 ms.
  3. It backs at the park spot (toggle X, back of the robot 3" short of the center goal), steering, with a braked stop.
- **Sim (Oct 24):** parks at 56.9 s with goal A 6 and goal B 1. A missed matchload costs about 6 s.

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
- Quals: `--start 61.95,0,270 --time 15`
- `--ideal` turns off slip and noise.
- Pretend pickup failures (pickup numbers: 1 = pin 1, 2 = pin 2, 3+ = loader):
  - `SIM_MISS=3` grabs nothing.
  - `SIM_DROP=3` grabs, then loses it 0.3 s later.
- The sim needs stand-ins for `fieldAwareReset` and the `OpeningRoute` struct if the project's own files aren't copied in.

How well the sim matches the real robot:
- The claw never stalls, so pickup checks rely on SIM_MISS / SIM_DROP.
- It only blocks the robot at the walls, as a center clamp, so a corner can go 3" into a wall. Goals and loaders are drawn but don't stop it.

## Known items
- In PROS 4, `DR4B.set_brake_mode(...)` sets only the first motor (port -7). `set_brake_mode_all` sets both. This was left as it was.
- The qual autons (auton.cpp, autonmirror.cpp) wrap headings with `fmod(x + 540, 360) - 180`. That breaks once the heading has counted past 540°. Skills was switched to `std::remainder`; the quals were left as they were.