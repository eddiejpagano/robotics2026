# robotics2026

Robot code for the 2026 season. This is a Java/WPILib project that uses GradleRIO, PathPlanner, PhotonVision/PhotonLib, AdvantageKit logging, and MapleSim-based simulation tests.

## Required downloads

Install the same major WPILib year used by the project. This branch is currently built with:

- WPILib / GradleRIO: `2025.3.2`
- Java: `17` through the WPILib installer
- PathPlannerLib vendordep: `2025.2.7`
- PhotonLib vendordep: `v2025.3.2`
- AdvantageKit vendordep: `4.1.2`
- MapleSim vendordep: `0.3.14`

The vendordep files are already checked into `vendordeps/`, so you do not need to install those libraries by hand. Gradle downloads them when the project builds.

### Everyone

Install these on both Windows and macOS:

- [Git](https://git-scm.com/downloads)
- [WPILib installer](https://docs.wpilib.org/en/2025/docs/zero-to-robot/step-2/wpilib-setup.html)
- [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope/releases)
- [PathPlanner](https://pathplanner.dev/gui-getting-started.html)
- An Xbox-compatible controller for desktop simulation testing

WPILib includes the correct Java runtime, Gradle tools, the WPILib VS Code build, simulation tools, Glass, SysId, Shuffleboard, and other standard FRC utilities. Do not install a separate Java or Gradle just for this repository unless you have a specific reason.

### Windows

Use 64-bit Windows 10 or Windows 11. Windows 11 is preferred for new installs.

1. Install [Git for Windows](https://git-scm.com/download/win).
2. Install WPILib `2025.3.2` and use the WPILib-provided VS Code shortcut.
3. Install AdvantageScope from the Mechanical Advantage releases page.
4. Install PathPlanner. The Microsoft Store install is the easiest option because it auto-updates, but the GitHub release also works.
5. Install the [FRC Game Tools / Driver Station](https://docs.wpilib.org/en/2025/docs/zero-to-robot/step-2/frc-game-tools.html) if this computer will connect to or drive a real robot. The Driver Station is Windows-only.

Recommended first validation:

```powershell
git clone <REPOSITORY_URL>
cd robotics2026
.\gradlew.bat clean test
.\gradlew.bat build
.\gradlew.bat simulateJava
```

For the full Windows simulation checklist, use [docs/WINDOWS_VALIDATION.md](docs/WINDOWS_VALIDATION.md).

### macOS

Use macOS 13.3 or newer. Both Apple Silicon and Intel Macs are supported by WPILib 2025.

1. Install Git. The easiest route is usually the Xcode command line tools:

   ```bash
   xcode-select --install
   ```

2. Install WPILib `2025.3.2` and use the WPILib-provided VS Code app.
3. Install AdvantageScope from the Mechanical Advantage releases page.
4. Install PathPlanner from the PathPlanner download instructions.
5. Do not look for the FRC Driver Station on macOS. Use Windows for official Driver Station operation with a real robot.

Recommended first validation:

```bash
git clone <REPOSITORY_URL>
cd robotics2026
./gradlew clean test
./gradlew build
./gradlew simulateJava
```

If the simulator controller axes are wrong on macOS Bluetooth, try the known override:

```bash
FRC_SIM_CONTROLLER_PROFILE=mac-bluetooth ./gradlew simulateJava
```

## Useful tools for this project

- WPILib VS Code: main editor and robot-code command palette.
- WPILib Simulator: launched with `simulateJava`.
- AdvantageScope: view NetworkTables/logged telemetry, including vision and pose diagnostics.
- PathPlanner: edit paths and autos under `src/main/deploy/pathplanner/`.
- PhotonVision: only needed if you are configuring or testing a real vision coprocessor. The robot code already includes PhotonLib for Java integration.

## Common commands

Windows:

```powershell
.\gradlew.bat clean test
.\gradlew.bat build
.\gradlew.bat simulateJava
```

macOS:

```bash
./gradlew clean test
./gradlew build
./gradlew simulateJava
```

Gradle is run through the checked-in Gradle Wrapper. That keeps the project on the expected Gradle version regardless of what is installed globally.

## Project layout

- `src/main/java/frc/robot/` - robot code
- `src/main/deploy/pathplanner/` - PathPlanner autos, paths, and navgrid files
- `src/test/java/frc/robot/` - simulation and configuration tests
- `vendordeps/` - checked-in FRC vendor dependencies
- `docs/` - setup and validation notes
- `scripts/validate-windows.ps1` - Windows validation helper script

## Notes

- Always open the repository with the WPILib version of VS Code, not a separately installed stock VS Code, unless you have manually configured the WPILib extensions.
- If dependencies fail to download, confirm you are online and then rerun the same Gradle command.
- If simulation starts but controls behave strangely, check `Driver/RawAxis*` and `Driver/RotationCommand` in AdvantageScope.
