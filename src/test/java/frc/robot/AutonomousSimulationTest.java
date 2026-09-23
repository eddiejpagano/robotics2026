package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DrivePathPlanner;
import frc.robot.subsystems.drive.DriveSimulationConstants;
import java.util.Locale;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AutonomousSimulationTest {
  private static final Pose2d kExpectedFinalPose = new Pose2d(4.0, 1.0, Pose2d.kZero.getRotation());
  private static final double kLoopPeriodSeconds = 0.02;
  private static final double kAutoTimeoutSeconds = 8.0;
  private static final double kSettleTimeoutSeconds = 1.0;
  private static final double kPositionToleranceMeters = 0.20;
  private static final double kEstimatedPoseToleranceMeters = 0.20;
  private static final double kFinalSpeedToleranceMetersPerSec = 0.20;
  private static final double kFinalOmegaToleranceRadiansPerSec = 0.20;

  private Drive drive;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    SimHooks.restartTiming();
    CommandScheduler.getInstance().cancelAll();

    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    if (drive != null) {
      drive.stop();
    }
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    SimulatedArena.getInstance().shutDown();
    SimHooks.resumeTiming();
  }

  @Test
  void straightTestAutoDrivesSimulatedRobotToExpectedDestination() {
    drive = new Drive(new DriveIOSim(), DriveSimulationConstants.SIM_START_POSE);
    DrivePathPlanner.configureAutoBuilder(drive);

    Command autonomousCommand = AutoBuilder.buildAuto("StraightTestAuto");
    autonomousCommand.schedule();

    double elapsedSeconds = 0.0;
    while (autonomousCommand.isScheduled() && elapsedSeconds < kAutoTimeoutSeconds) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kLoopPeriodSeconds);
      elapsedSeconds += kLoopPeriodSeconds;
    }

    CommandScheduler.getInstance().run();

    assertFalse(autonomousCommand.isScheduled(), "StraightTestAuto timed out before finishing");

    double settleElapsedSeconds = 0.0;
    while (getTranslationSpeed(drive) > kFinalSpeedToleranceMetersPerSec
        && settleElapsedSeconds < kSettleTimeoutSeconds) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(kLoopPeriodSeconds);
      settleElapsedSeconds += kLoopPeriodSeconds;
    }
    CommandScheduler.getInstance().run();

    Pose2d groundTruthPose = drive.getSimulationGroundTruthPose();
    assertEquals(kExpectedFinalPose.getX(), groundTruthPose.getX(), kPositionToleranceMeters);
    assertEquals(kExpectedFinalPose.getY(), groundTruthPose.getY(), kPositionToleranceMeters);
    assertTrue(
        getTranslationError(groundTruthPose, kExpectedFinalPose) <= kPositionToleranceMeters,
        "MapleSim ground truth did not reach the StraightTest target");

    ChassisSpeeds finalSpeeds = drive.getRobotRelativeSpeeds();
    assertEquals(0.0, getTranslationSpeed(finalSpeeds), kFinalSpeedToleranceMetersPerSec);
    assertEquals(0.0, finalSpeeds.omegaRadiansPerSecond, kFinalOmegaToleranceRadiansPerSec);

    Pose2d estimatedPose = drive.getPose();
    assertTrue(
        getTranslationError(estimatedPose, groundTruthPose) <= kEstimatedPoseToleranceMeters,
        "Drive estimated pose is too far from MapleSim ground truth");

    System.out.printf(
        Locale.US,
        "StraightTestAuto sim result: elapsed=%.2fs settle=%.2fs groundTruth=(%.3f, %.3f, %.3f rad) estimated=(%.3f, %.3f, %.3f rad) speeds=(vx %.3f, vy %.3f, omega %.3f)%n",
        elapsedSeconds,
        settleElapsedSeconds,
        groundTruthPose.getX(),
        groundTruthPose.getY(),
        groundTruthPose.getRotation().getRadians(),
        estimatedPose.getX(),
        estimatedPose.getY(),
        estimatedPose.getRotation().getRadians(),
        finalSpeeds.vxMetersPerSecond,
        finalSpeeds.vyMetersPerSecond,
        finalSpeeds.omegaRadiansPerSecond);
  }

  private static double getTranslationSpeed(Drive drive) {
    return getTranslationSpeed(drive.getRobotRelativeSpeeds());
  }

  private static double getTranslationSpeed(ChassisSpeeds speeds) {
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  private static double getTranslationError(Pose2d pose, Pose2d reference) {
    return pose.getTranslation().getDistance(reference.getTranslation());
  }
}
