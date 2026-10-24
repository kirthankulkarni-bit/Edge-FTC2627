# FTC Robot Project Brief

This file gives Claude Code the context for our FTC (FIRST Tech Challenge) robot code. Read all of it before making changes.

## Team and goal

- Rookie FTC team.
- We are moving from OnBot Java (browser editor on the Control Hub) to Android Studio.
- We want to use **Pedro Pathing** for autonomous, set up through the official **Pedro-Pathing/Quickstart** repo.
- We need one working autonomous and one fast, reliable TeleOp.

## Environment (do not change without asking)

- **Android Studio:** Narwhal 3 Feature Drop (2025.1.3). Do not suggest upgrading Android Studio.
- **Do NOT upgrade** the Android Gradle Plugin (AGP), Gradle, or any FTC SDK dependency versions unless we ask. Upgrades can break the build.
- **FTC SDK:** 12.0.0 (from the Pedro Quickstart). The Robot Controller app on our Control Hub must match that version. Flag any mismatch.
- **Pedro Pathing:** 3.0.2 (`com.pedropathing:revhub`), tuning 1.0.1. Do NOT use Pedro 1.x APIs or imports (the legacy Quickstart-1.0.9 repo is outdated).
- **Build tools:** AGP 8.13.2, Gradle 9.1.0.
- **OS:** Windows laptop.
- **Deploy:** USB-C to REV Control Hub, run configuration "TeamCode".

## Repo

- GitHub (private): https://github.com/kirthankulkarni-bit/Edge-FTC2627 (`origin`)
- Pedro Quickstart: `upstream`, for pulling Pedro updates.

## Hardware

- **Controller:** REV Control Hub
- **Driver station:** REV Driver Hub
- **Chassis:** goBILDA Strafer (mecanum)
- **Drive motors:** goBILDA 312 RPM (537.7 ticks per rev). Confirm before relying on this.
- **Wheels:** goBILDA 96 mm mecanum (3.78 in diameter). Confirm before relying on this.
- **Encoders:** drive encoder cables may NOT be plugged in yet. Check before writing encoder-dependent code. Pedro needs working localization.
- **Localizer:** goBILDA Pinpoint (chosen). I2C port and config name not confirmed yet.

### Motor config (Driver Hub robot configuration)

| Port | Name         | Direction |
|------|--------------|-----------|
| 0    | `leftFront`  | REVERSE   |
| 1    | `leftBack`   | REVERSE   |
| 2    | `rightFront` | FORWARD   |
| 3    | `rightBack`  | FORWARD   |

All code must use these exact names.

## Known issues and history

1. **Strafing felt inverted** in TeleOp. Fixed with `STRAFE_DIRECTION = -1.0`.
2. **Auto kept driving straight and never stopped.** Likely cause: `RUN_TO_POSITION` with no encoder cables, so the target was never reached. Fixed with timed driving by default plus a safety timeout.
3. **Timed 180 degree turn was inaccurate.** `SECONDS_PER_ROTATION` is a guess and needs tuning. Long-term fix: IMU heading or Pedro.
4. **TeleOp felt slow.** Normal mode was already 100% power. Added stick gain (1.25) and strafe boost (1.15). Remaining slowness is hardware (motor RPM, battery, friction).

## Tasks for Claude Code

Do these in order. Ask before moving to the next one if anything is unclear.

1. ~~**Verify the Quickstart project** builds and syncs.~~ Done: builds on FTC SDK 12.0.0, Pedro 3.0.2.
2. **Fill in the Pedro constants file** (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`). Motor names and directions are done. Every `TODO` is a placeholder to replace with tuner output once the robot is available (Pinpoint name, pod type, offsets and directions; all Foresight values). Pedro 3.0.2 has no robot-mass setting.
3. ~~**Port our two OpModes**~~ Done: `BasicAuto.java` and `StraferTeleOp.java` are in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
4. **Write a Pedro version of the autonomous** with the same routine:
   - Drive forward 12 inches
   - Turn around 180 degrees
   - Drive forward 12 inches (ends back at the start, facing the opposite way)
   - Run once, then stop. Call `follower.update()` every loop. Use a simple state machine.
5. **Keep the TeleOp working** as plain motor control. Pedro TeleOp is optional and only if we ask.
6. Explain what each file does in plain language. We are learning.

## Code style rules

- Java.
- Package: `org.firstinspires.ftc.teamcode`.
- Keep all tunable numbers in a clearly marked constants section at the top of each file.
- Every loop must check `opModeIsActive()` (LinearOpMode) or run inside `loop()` (OpMode) so the stop button always works.
- Add short comments explaining *why*, not just *what*.
- When giving us code, give the **complete file**, ready to copy and paste.

## Path planning

We plan to draw paths in the Pedro Pathing web visualizer (linked from the Pedro docs; we believe it is at `visualizer.pedropathing.com`, but confirm). Pedro uses a regular inch-based field grid. Confirm the origin and heading units (radians vs degrees) from the docs before converting visualizer points to code.
