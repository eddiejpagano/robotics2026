// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOReal;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DrivePathPlanner;
import frc.robot.subsystems.drive.DriveSimulationConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final double kDriveDeadband = 0.1;
  private static final double kDriveRotationDeadband = 0.2;
  private static final double kMaxDemoTranslationSpeedMetersPerSec = 3.0;
  private static final double kMaxDemoAngularSpeedRadiansPerSec = 3.0;

  // The robot's subsystems and commands are defined here...
  private final Drive m_drive = new Drive(createDriveIO(), getInitialDrivePose());
  private final Vision m_vision = new Vision(createVisionIO(), m_drive::addVisionMeasurement);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DrivePathPlanner.configureAutoBuilder(m_drive);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      SimControllerMapping simControllerMapping = SimControllerMapping.select();
      Logger.recordOutput("Driver/ControllerProfile", simControllerMapping.getProfileName());

      m_drive.setDefaultCommand(
          m_drive.run(
              () -> {
                var controller = m_driverController.getHID();
                double xSpeed =
                    -MathUtil.applyDeadband(simControllerMapping.getLeftY(controller), kDriveDeadband)
                        * kMaxDemoTranslationSpeedMetersPerSec;
                double ySpeed =
                    -MathUtil.applyDeadband(simControllerMapping.getLeftX(controller), kDriveDeadband)
                        * kMaxDemoTranslationSpeedMetersPerSec;
                double omega =
                    -MathUtil.applyDeadband(
                            simControllerMapping.getRightX(controller), kDriveRotationDeadband)
                        * kMaxDemoAngularSpeedRadiansPerSec;

                simControllerMapping.recordDiagnostics(controller);
                Logger.recordOutput("Driver/RotationCommand", omega);

                m_drive.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, omega, m_drive.getHeading()));
              }));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    DrivePathPlanner.logStraightTestDiagnostics();
    return AutoBuilder.buildAuto("StraightTestAuto");
  }

  private static DriveIO createDriveIO() {
    switch (Constants.currentMode) {
      case REAL:
        return new DriveIOReal();
      case SIM:
        return new DriveIOSim();
      case REPLAY:
        return new DriveIO() {};
      default:
        return new DriveIO() {};
    }
  }

  private static edu.wpi.first.math.geometry.Pose2d getInitialDrivePose() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return DriveSimulationConstants.SIM_START_POSE;
    }
    return new edu.wpi.first.math.geometry.Pose2d();
  }

  private VisionIO createVisionIO() {
    switch (Constants.currentMode) {
      case REAL:
        return new VisionIOReal();
      case SIM:
        return new VisionIOSim(m_drive::getSimulationGroundTruthPose);
      case REPLAY:
        return new VisionIO() {};
      default:
        return new VisionIO() {};
    }
  }
}
