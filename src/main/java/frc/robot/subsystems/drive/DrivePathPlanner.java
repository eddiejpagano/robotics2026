// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public final class DrivePathPlanner {
  private static final double kMaxPathPlannerSpeedMetersPerSec = 3.0;
  private static final double kDriveCurrentLimitAmps = 60.0;

  public static void configureAutoBuilder(Drive drive) {
    logStraightTestDiagnostics();

    AutoBuilder.configure(
        drive::getPose,
        drive::resetPose,
        drive::getRobotRelativeSpeeds,
        drive::drive,
        new PPHolonomicDriveController(new PIDConstants(5.0), new PIDConstants(5.0)),
        createRobotConfig(),
        DrivePathPlanner::shouldFlipPath,
        drive);
  }

  public static void logStraightTestDiagnostics() {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("StraightTest");
      RobotConfig robotConfig = createRobotConfig();

      Logger.recordOutput("PathPlanner/WaypointCount", path.getWaypoints().size());

      var pathPoses = path.getPathPoses();
      Logger.recordOutput(
          "PathPlanner/StartPose",
          path.getStartingHolonomicPose()
              .orElse(pathPoses.isEmpty() ? new Pose2d() : pathPoses.get(0)));

      var trajectory = path.getIdealTrajectory(robotConfig);
      Logger.recordOutput("PathPlanner/TrajectoryAvailable", trajectory.isPresent());

      if (trajectory.isPresent()) {
        PathPlannerTrajectory generatedTrajectory = trajectory.get();
        Logger.recordOutput(
            "PathPlanner/TrajectoryDurationSeconds",
            generatedTrajectory.getTotalTimeSeconds());
        Logger.recordOutput("PathPlanner/StateCount", generatedTrajectory.getStates().size());
        Logger.recordOutput(
            "PathPlanner/EndPose", generatedTrajectory.getEndState().pose);
      } else {
        if (!pathPoses.isEmpty()) {
          Logger.recordOutput("PathPlanner/EndPose", pathPoses.get(pathPoses.size() - 1));
        } else {
          Logger.recordOutput("PathPlanner/EndPose", new Pose2d());
        }
        Logger.recordOutput("PathPlanner/TrajectoryDurationSeconds", 0.0);
        Logger.recordOutput("PathPlanner/StateCount", 0);
      }
    } catch (Exception exception) {
      Logger.recordOutput("PathPlanner/DiagnosticsError", exception.toString());
    }
  }

  private static RobotConfig createRobotConfig() {
    return new RobotConfig(
        DriveSimulationConstants.ROBOT_MASS,
        estimateMomentOfInertia(),
        new ModuleConfig(
            DriveSimulationConstants.WHEEL_RADIUS,
            MetersPerSecond.of(kMaxPathPlannerSpeedMetersPerSec),
            DriveSimulationConstants.WHEEL_COEFFICIENT_OF_FRICTION,
            DriveSimulationConstants.DRIVE_MOTOR,
            DriveSimulationConstants.DRIVE_GEAR_RATIO,
            Amps.of(kDriveCurrentLimitAmps),
            1),
        DriveSimulationConstants.MODULE_TRANSLATIONS);
  }

  private static MomentOfInertia estimateMomentOfInertia() {
    double massKg = DriveSimulationConstants.ROBOT_MASS.in(edu.wpi.first.units.Units.Kilograms);
    double lengthMeters = DriveSimulationConstants.BUMPER_LENGTH_X.in(Meters);
    double widthMeters = DriveSimulationConstants.BUMPER_WIDTH_Y.in(Meters);
    return KilogramSquareMeters.of(massKg * (lengthMeters * lengthMeters + widthMeters * widthMeters) / 12.0);
  }

  private static boolean shouldFlipPath() {
    return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red)
        .orElse(false);
  }

  private DrivePathPlanner() {}
}
