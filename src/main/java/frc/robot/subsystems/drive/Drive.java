// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveSimulationConstants.MODULE_TRANSLATIONS);
  private final SwerveModulePosition[] modulePositions = createZeroModulePositions();
  private final SwerveDrivePoseEstimator poseEstimator;
  private double previousOdometryTimestampSeconds = Timer.getFPGATimestamp();

  public Drive(DriveIO io) {
    this(io, new Pose2d());
  }

  public Drive(DriveIO io, Pose2d initialPose) {
    this.io = io;
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, initialPose.getRotation(), copyModulePositions(), initialPose);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updateOdometry();
    Logger.processInputs("Drive", inputs);
    Logger.recordOutput("Drive/EstimatedPose", getPose());
    Logger.recordOutput("Drive/RobotPose", getSimulationGroundTruthPose());
    Logger.recordOutput("Drive/SimulationGroundTruthPose", getSimulationGroundTruthPose());
  }

  public void drive(ChassisSpeeds speeds) {
    Logger.recordOutput("Drive/CommandedVxMetersPerSec", speeds.vxMetersPerSecond);
    Logger.recordOutput("Drive/CommandedVyMetersPerSec", speeds.vyMetersPerSecond);
    Logger.recordOutput("Drive/CommandedOmegaRadiansPerSec", speeds.omegaRadiansPerSecond);
    io.drive(speeds);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return new ChassisSpeeds(
        inputs.chassisVxMetersPerSec,
        inputs.chassisVyMetersPerSec,
        inputs.chassisOmegaRadiansPerSec);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getSimulationGroundTruthPose() {
    return new Pose2d(
        inputs.robotXPositionMeters,
        inputs.robotYPositionMeters,
        new Rotation2d(inputs.robotHeadingRadians));
  }

  public Rotation2d getHeading() {
    return new Rotation2d(inputs.robotHeadingRadians);
  }

  public void resetPose(Pose2d pose) {
    resetEstimatorPose(pose);
    io.resetPose(pose);
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    if (!isFinitePose(pose) || !Double.isFinite(timestampSeconds) || timestampSeconds <= 0.0) {
      return;
    }

    poseEstimator.addVisionMeasurement(pose, timestampSeconds);
  }

  public void stop() {
    io.stop();
  }

  private void updateOdometry() {
    double timestampSeconds = Timer.getFPGATimestamp();
    double dtSeconds = timestampSeconds - previousOdometryTimestampSeconds;
    previousOdometryTimestampSeconds = timestampSeconds;

    if (!Double.isFinite(dtSeconds) || dtSeconds <= 0.0 || dtSeconds > 0.1) {
      return;
    }

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(getRobotRelativeSpeeds());
    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i].distanceMeters += moduleStates[i].speedMetersPerSecond * dtSeconds;
      modulePositions[i].angle = moduleStates[i].angle;
    }

    poseEstimator.updateWithTime(
        timestampSeconds, new Rotation2d(inputs.robotHeadingRadians), copyModulePositions());
  }

  private void resetEstimatorPose(Pose2d pose) {
    resetModulePositions();
    poseEstimator.resetPosition(pose.getRotation(), copyModulePositions(), pose);
    previousOdometryTimestampSeconds = Timer.getFPGATimestamp();
  }

  private static SwerveModulePosition[] createZeroModulePositions() {
    SwerveModulePosition[] positions =
        new SwerveModulePosition[DriveSimulationConstants.MODULE_TRANSLATIONS.length];
    for (int i = 0; i < positions.length; i++) {
      positions[i] = new SwerveModulePosition();
    }
    return positions;
  }

  private SwerveModulePosition[] copyModulePositions() {
    SwerveModulePosition[] copy = new SwerveModulePosition[modulePositions.length];
    for (int i = 0; i < modulePositions.length; i++) {
      copy[i] = modulePositions[i].copy();
    }
    return copy;
  }

  private void resetModulePositions() {
    for (SwerveModulePosition modulePosition : modulePositions) {
      modulePosition.distanceMeters = 0.0;
      modulePosition.angle = new Rotation2d();
    }
  }

  private static boolean isFinitePose(Pose2d pose) {
    return Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }
}
