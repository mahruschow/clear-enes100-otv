# ENES100 browser simulator route

Paste the entire `clear_simulator.ino` into the [ENES100 Arduino simulator](https://umdenes100.github.io/enes100-simulator/). Use the obstacle arrangement from our screenshot and start at X `0.55`, Y `0.55`, heading `0`. Click **Run code**, then **Continue code** if the debugger pauses on the first line.

The six targets (meters) are `(0.90, 1.55)`, `(1.90, 1.55)`, `(1.90, 0.85)`, `(2.85, 0.85)`, `(3.02, 1.40)`, and `(3.75, 1.40)`. The tank drives above the first white block, goes between the two, then approaches the upper limbo section at X about `3.4`. It turns in place, checks the front ultrasonic sensor before driving, waits if its vision marker is missing, and stops at the last target.

This is a **fixed-layout prototype**. `Randomize` changes the block locations, so these coordinates need a new route or a mapping/planning layer. The ultrasonic sensor cannot detect arena walls and may not detect the log/limbo. Do not attempt the limbo with the team's actual OTV unless its full height and width clear it. The screenshot does not establish that mechanical clearance. This simulator sketch also uses `Tank.h`; the physical robot's motor drivers and sensor pins need a separate hardware implementation.

Test the route one waypoint at a time. If a turn is too slow, tune `turnPwm`; if the robot overshoots, reduce the forward PWM or waypoint radius. Keep the front stop threshold larger than the distance from the sensor to the front of the robot plus a braking margin. Log the actual X, Y, and heading during trials, then update the waypoint list from the measured obstacle positions.
