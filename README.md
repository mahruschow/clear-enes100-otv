# CLEAR ENES100 software

Arduino C++ starter for Mahrus and Matt. The current sketch controls the **course practice tank** (Arduino Mega), not the final UNO R4 WiFi vehicle. Keep this repository private while the team iterates.

## Install on each teammate's computer

1. Install the [Arduino IDE](https://www.arduino.cc/en/software/) if it is not already installed.
2. Install the [ENES100 Arduino library](https://github.com/umdenes100/ENES100ArduinoLibrary/archive/master.zip) (course documentation currently describes v6.2.4) and the [Tank Arduino library](https://github.com/umdenes100/TankArduinoLibrary/archive/master.zip). In Arduino IDE use **Sketch > Include Library > Add .ZIP Library...** for each ZIP, then restart the IDE. If an older ENES100 library is installed, remove the old copy first. See the [ENES100 setup](https://enes100.umd.edu/documentation/arduino/) and [tank guide](https://enes100.umd.edu/documentation/tank/).
3. Open `practice_tank/practice_tank.ino` in Arduino IDE. Leave `Navigation.h` in the same `practice_tank` folder.
4. For a physical course tank, set **Tools > Board > Arduino Mega or Mega 2560**, select its USB port, and replace `MARKER_ID` and `ROOM_NUMBER` in the sketch with the tank's assigned values. The tank uses the existing external Wi-Fi module on Mega pins 52 (TX) and 50 (RX).
5. First upload with `ENABLE_MOTION = false`. Verify the vision system reports a visible pose and the front range changes when a white block is moved in front of the tank. Then, in a clear test area, set `ENABLE_MOTION = true` and test one short waypoint.

For the **web simulator**, use the simulator's Arduino editor and the same sketch, adjusting the vision connection setup to the simulator's provided example if necessary. The course says `Tank.h` supports both environments, but its physical tank directions assume the Mega and external Wi-Fi module. Verify the simulator's required `Enes100.begin(...)` form in its example before running.

## What this first version does

The robot reads x, y and heading from overhead vision, faces a single target, drives slowly, and stops inside a target radius. It also stops if the marker is invisible, the position is outside arena bounds, the front ultrasonic reading is too close or invalid, or the pose does not change for too long while motors are commanded. It **does not choose a detour**: that requires measured robot dimensions, a confirmed obstacle layout, and a known clear path. Change `TARGET_X` and `TARGET_Y` only after verifying their location on your course map and a clear route.

The practice tank has the course's own motor and ultrasonic wiring. Your final UNO R4 WiFi vehicle will use the R4's onboard Wi-Fi and its own DRV8871 motor driver pins; do not upload the tank sketch to the final robot. The R4 call will be `Enes100.begin("CLEAR", WATER, markerId, roomNumber)`.

## Working together with GitHub

Create a **private** repository named `clear-enes100-otv`, upload the contents of this folder, then invite Matt under **Settings > Collaborators** using his GitHub username. Each person can clone the repository, work on a branch (`mahrus/navigation`, `matt/vision`, etc.), and open a pull request so the other can review changes. Do not commit Wi-Fi passwords, personal credentials, or generated build folders.

## Next changes after the first tank test

- Record tank marker ID, room number, actual starting pose, and safe waypoint coordinates.
- Log a few front-sensor readings to decide a realistic stop threshold for that tank.
- Add waypoint sequencing after a single target works consistently.
- Create a separate `final_otv` sketch when the actual UNO R4, motor drivers, sensor pins, and chassis measurements are verified.

## Source documentation

- [ENES100 Arduino library](https://enes100.umd.edu/documentation/arduino/)
- [Tank library reference](https://enes100.umd.edu/documentation/tank/)
- [Tank practice guide](https://enes100.umd.edu/documentation/tank_guide/)
- [Examples](https://enes100.umd.edu/documentation/examples/)
