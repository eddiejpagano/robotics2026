# Windows Validation Guide

This guide is for a Windows user validating the `robotics2026` simulation branch from a clean checkout.

The goal is to confirm that the project works on Windows without machine-specific code changes. Please do the first test pass exactly as written and do **not** modify the code unless asked afterward.

## 1. Prerequisites

Install the WPILib release that matches this branch and use the WPILib-provided VS Code environment.

You will also need:

- Git
- An Xbox-compatible controller, preferably connected the same way you normally use it for FRC development
- AdvantageScope

Gradle does **not** need to be installed separately. The repository includes the Gradle Wrapper.

## 2. Clone the repository and check out the test branch

Open PowerShell and run:

```powershell
git clone <REPOSITORY_URL>
cd robotics2026
git fetch origin
git switch <BRANCH_NAME>
```

If the branch only exists on the remote and `git switch <BRANCH_NAME>` does not work, use:

```powershell
git switch --track origin/<BRANCH_NAME>
```

Confirm the branch:

```powershell
git branch --show-current
```

Expected result: the branch name being tested.

## 3. Open the project in WPILib VS Code

Open the cloned repository in the WPILib version of VS Code.

Open a new PowerShell terminal from the repository root.

## 4. Run the automated validation

Run:

```powershell
.\gradlew.bat clean test
```

Then:

```powershell
.\gradlew.bat build
```

Both commands must complete successfully.

The test suite should include the PathPlanner configuration/trajectory tests and the autonomous MapleSim integration test.

The autonomous integration test is expected to verify that `StraightTestAuto` reaches approximately:

```text
X = 4.0 m
Y = 1.0 m
```

with reasonable physics-simulation tolerances.

## 5. Start the simulator

Run:

```powershell
.\gradlew.bat simulateJava
```

The WPILib simulator should open.

Assign the Xbox controller to `Joystick[0]` in the simulator if it is not assigned automatically.

## 6. Verify the robot while Disabled

Before enabling Teleop, confirm:

| Check                                   | Expected                                                           |
| --------------------------------------- | ------------------------------------------------------------------ |
| Robot appears on the field              | Yes                                                                |
| Robot starts inside the field           | Yes, near the configured simulation start pose around `(1.0, 1.0)` |
| Robot remains stationary while Disabled | Yes                                                                |
| Robot does not jitter or drift          | Yes                                                                |
| Vision simulation remains active        | Yes                                                                |

If the physical MapleSim robot moves while Disabled, record what happened and stop the test.

## 7. Verify Teleop controls

Enable Teleop.

Expected controller behavior:

| Control                | Expected behavior                 |
| ---------------------- | --------------------------------- |
| Left stick up/down     | Robot translates forward/backward |
| Left stick left/right  | Robot strafes left/right          |
| Right stick left/right | Robot rotates                     |
| Sticks released        | Robot stops                       |

The robot uses field-relative driving, so translation behavior is relative to the field rather than the robot's current heading.

### Controller mapping check

This project includes configurable simulation controller mappings because Xbox axis ordering can differ between operating systems, controller models, drivers, and Bluetooth/USB connections.

On Windows, test the default configuration first. Do **not** set a controller-profile override unless needed.

If the robot immediately spins when Teleop is enabled, or the right stick does not control rotation, check AdvantageScope for:

```text
Driver/RightXRaw
Driver/RotationCommand
```

If raw-axis diagnostics are present, also inspect:

```text
Driver/RawAxis0
Driver/RawAxis1
Driver/RawAxis2
Driver/RawAxis3
Driver/RawAxis4
Driver/RawAxis5
```

With the controller untouched, the axis used for rotation should be approximately `0`, not `-1` or `+1`.

Move only the right stick horizontally and note which raw axis changes.

Do not change source code during this test. Record the axis behavior so the controller profile can be updated centrally if necessary.

### Optional controller-profile override

If instructed to test an explicit profile, set it in PowerShell before starting simulation:

```powershell
$env:FRC_SIM_CONTROLLER_PROFILE="standard"
.\gradlew.bat simulateJava
```

The currently known Mac Bluetooth mapping can also be selected for comparison:

```powershell
$env:FRC_SIM_CONTROLLER_PROFILE="mac-bluetooth"
.\gradlew.bat simulateJava
```

Clear the override with:

```powershell
Remove-Item Env:FRC_SIM_CONTROLLER_PROFILE
```

## 8. Verify PhotonVision simulation

Open AdvantageScope and connect to the local simulation.

Confirm these fields, or their equivalent names, are present:

```text
Vision/Connected
Vision/HasTargets
Vision/TargetCount
Vision/BestTargetId
Vision/BestTargetYawDegrees
Vision/EstimatedPose
Vision/SimulationGroundTruthPose
Drive/EstimatedPose
```

Drive and rotate the robot.

Expected behavior:

- `Vision/Connected` is true.
- AprilTag targets appear when the simulated camera faces visible tags.
- `Vision/HasTargets` becomes false when the camera is pointed away from visible tags.
- `BestTargetYawDegrees` changes as the robot rotates.
- `Vision/EstimatedPose` stays reasonably close to `Vision/SimulationGroundTruthPose`.
- `Drive/EstimatedPose` stays reasonably close to ground truth.
- The ground-truth pose must not be driven by the vision-estimated pose.

## 9. Verify autonomous operation

Disable the robot, then select Autonomous.

Run `StraightTestAuto`.

Expected behavior:

- The robot resets to the autonomous starting pose.
- The robot follows the PathPlanner trajectory.
- The robot finishes near `(4.0, 1.0)`.
- The robot comes substantially to rest at the end.
- The autonomous command completes rather than running indefinitely.

The automated integration test already checks this numerically; this manual step verifies the Windows desktop simulation behaves the same way.

## 10. Report the results

Please send back the following:

| Item                                                 | Result                  |
| ---------------------------------------------------- | ----------------------- |
| Windows version                                      |                         |
| Controller model                                     |                         |
| Controller connection                                | USB / Bluetooth / other |
| Branch tested                                        |                         |
| `clean test`                                         | PASS / FAIL             |
| `build`                                              | PASS / FAIL             |
| `simulateJava` starts                                | PASS / FAIL             |
| Robot stationary while Disabled                      | PASS / FAIL             |
| Left-stick translation                               | PASS / FAIL             |
| Right-stick rotation                                 | PASS / FAIL             |
| Default controller profile                           | PASS / FAIL             |
| PhotonVision target detection                        | PASS / FAIL             |
| Vision estimated pose reasonable                     | PASS / FAIL             |
| `StraightTestAuto`                                   | PASS / FAIL             |
| Final autonomous location approximately `(4.0, 1.0)` | PASS / FAIL             |

If the controller mapping is incorrect, also report the idle values for raw axes `0` through `5` and identify which raw axis responds to right-stick horizontal movement.

If any Gradle command fails, copy the relevant terminal error output into the report.

## Success Criteria

The Windows validation is considered successful when a clean checkout can:

```text
clone branch
    ↓
.\gradlew.bat clean test
    ↓
.\gradlew.bat build
    ↓
.\gradlew.bat simulateJava
    ↓
Teleop + rotation work
    ↓
PhotonVision simulation works
    ↓
StraightTestAuto completes correctly
```

without source-code changes specific to that Windows machine.
