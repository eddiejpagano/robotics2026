// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  public Drive(DriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);
  }

  public void drive(ChassisSpeeds speeds) {
    io.drive(speeds);
  }

  public void stop() {
    io.stop();
  }
}
