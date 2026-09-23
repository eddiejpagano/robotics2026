# Windows Validation

Use this checklist on a fresh Windows checkout.

1. Install/open the matching WPILib version.

2. Clone the repository into a new empty directory.

3. Open the project in WPILib VS Code.

4. Open PowerShell in the repo root.

5. Run:

   ```powershell
   .\scripts\validate-windows.ps1
   ```

6. Confirm all JUnit tests pass, including:

   - PathPlanner configuration/trajectory tests
   - AutonomousSimulationTest

7. Run:

   ```powershell
   .\gradlew.bat simulateJava
   ```

8. Verify:

   - WPILib simulator opens
   - robot begins at the configured in-field SIM start pose
   - robot remains stationary while Disabled
   - Teleop translation works
   - Teleop rotation works after confirming controller mapping
   - PhotonVision Connected = true
   - AprilTag targets appear when visible
   - Autonomous StraightTestAuto runs
   - robot ends near (4.0, 1.0)

   If the controller axes do not match the expected mapping, set
   `FRC_SIM_CONTROLLER_PROFILE` before launching simulation. Supported values are
   `standard` and `mac-bluetooth`.

9. Record any Windows-specific differences.
