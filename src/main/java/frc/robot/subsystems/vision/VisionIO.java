// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public boolean hasTargets = false;
    public int targetCount = 0;
    public int bestTargetId = -1;
    public double bestTargetYawDegrees = 0.0;
    public double bestTargetPitchDegrees = 0.0;
    public double latestTimestampSeconds = 0.0;
    public boolean estimatedPoseValid = false;
    public Pose3d estimatedPose = new Pose3d();
    public double estimatedTimestampSeconds = 0.0;
    public Pose2d simulationGroundTruthPose = new Pose2d();
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
