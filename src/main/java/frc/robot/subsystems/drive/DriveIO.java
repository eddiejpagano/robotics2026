// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public double robotXPositionMeters = 0.0;
    public double robotYPositionMeters = 0.0;
    public double robotHeadingRadians = 0.0;
    public double chassisVxMetersPerSec = 0.0;
    public double chassisVyMetersPerSec = 0.0;
    public double chassisOmegaRadiansPerSec = 0.0;
  }

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void drive(ChassisSpeeds speeds) {}

  public default void stop() {}
}
