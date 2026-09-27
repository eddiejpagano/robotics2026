// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.BiConsumer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final BiConsumer<Pose2d, Double> visionMeasurementConsumer;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private double lastSubmittedTimestampSeconds = -1.0;

  public Vision(VisionIO io, BiConsumer<Pose2d, Double> visionMeasurementConsumer) {
    this.io = io;
    this.visionMeasurementConsumer = visionMeasurementConsumer;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    Logger.recordOutput("Vision/EstimatedPose", inputs.estimatedPose);
    Logger.recordOutput("Vision/EstimatedPoseValid", inputs.estimatedPoseValid);
    Logger.recordOutput("Vision/EstimatedTimestampSeconds", inputs.estimatedTimestampSeconds);
    Logger.recordOutput("Vision/SimulationGroundTruthPose", inputs.simulationGroundTruthPose);

    if (isValidVisionMeasurement()) {
      visionMeasurementConsumer.accept(
          inputs.estimatedPose.toPose2d(), inputs.estimatedTimestampSeconds);
      lastSubmittedTimestampSeconds = inputs.estimatedTimestampSeconds;
    }
  }

  private boolean isValidVisionMeasurement() {
    Pose2d pose = inputs.estimatedPose.toPose2d();
    return inputs.estimatedPoseValid
        && Double.isFinite(inputs.estimatedTimestampSeconds)
        && inputs.estimatedTimestampSeconds > lastSubmittedTimestampSeconds
        && Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians())
        && pose.getX() >= 0.0
        && pose.getX() <= 18.0
        && pose.getY() >= 0.0
        && pose.getY() <= 9.0;
  }
}
