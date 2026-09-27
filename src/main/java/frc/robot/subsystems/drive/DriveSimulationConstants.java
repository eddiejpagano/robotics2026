// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public final class DriveSimulationConstants {
  public static final Mass ROBOT_MASS = Pounds.of(115.0);
  public static final Distance BUMPER_LENGTH_X = Inches.of(30.0);
  public static final Distance BUMPER_WIDTH_Y = Inches.of(30.0);
  public static final Distance TRACK_LENGTH_X = Inches.of(22.0);
  public static final Distance TRACK_WIDTH_Y = Inches.of(22.0);
  public static final Distance WHEEL_RADIUS = Inches.of(2.0);
  public static final Pose2d SIM_START_POSE =
      new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0.0));

  public static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(TRACK_LENGTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0),
    new Translation2d(TRACK_LENGTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0),
    new Translation2d(-TRACK_LENGTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0),
    new Translation2d(-TRACK_LENGTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0)
  };

  public static final DCMotor DRIVE_MOTOR =
      new DCMotor(12.0, 2.6, 105.0, 1.8, rotationsPerMinuteToRadiansPerSecond(5600.0), 1);
  public static final DCMotor STEER_MOTOR =
      new DCMotor(12.0, 0.9, 100.0, 1.4, rotationsPerMinuteToRadiansPerSecond(11000.0), 1);
  public static final double DRIVE_GEAR_RATIO = 6.75;
  public static final double STEER_GEAR_RATIO = 12.8;
  public static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);
  public static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
  public static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.03);
  public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.2;

  public static SwerveModuleSimulationConfig createModuleConfig() {
    return new SwerveModuleSimulationConfig(
        DRIVE_MOTOR,
        STEER_MOTOR,
        DRIVE_GEAR_RATIO,
        STEER_GEAR_RATIO,
        DRIVE_FRICTION_VOLTAGE,
        STEER_FRICTION_VOLTAGE,
        WHEEL_RADIUS,
        STEER_INERTIA,
        WHEEL_COEFFICIENT_OF_FRICTION);
  }

  private static double rotationsPerMinuteToRadiansPerSecond(double rotationsPerMinute) {
    return rotationsPerMinute * 2.0 * Math.PI / 60.0;
  }

  private DriveSimulationConstants() {}
}
