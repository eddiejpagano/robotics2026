// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;

public class DriveIOSim implements DriveIO {
  private final SimulatedArena arena;
  private final SelfControlledSwerveDriveSimulation driveSimulation;

  public DriveIOSim() {
    arena = new Arena2025Reefscape();
    SimulatedArena.overrideInstance(arena);

    SwerveDriveSimulation swerveDriveSimulation =
        new SwerveDriveSimulation(
            createDriveTrainConfig(), DriveSimulationConstants.SIM_START_POSE);
    arena.addDriveTrainSimulation(swerveDriveSimulation);
    driveSimulation = new SelfControlledSwerveDriveSimulation(swerveDriveSimulation);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    arena.simulationPeriodic();
    driveSimulation.periodic();

    Pose2d pose = driveSimulation.getActualPoseInSimulationWorld();
    ChassisSpeeds speeds = driveSimulation.getActualSpeedsRobotRelative();

    inputs.robotXPositionMeters = pose.getX();
    inputs.robotYPositionMeters = pose.getY();
    inputs.robotHeadingRadians = pose.getRotation().getRadians();
    inputs.chassisVxMetersPerSec = speeds.vxMetersPerSecond;
    inputs.chassisVyMetersPerSec = speeds.vyMetersPerSecond;
    inputs.chassisOmegaRadiansPerSec = speeds.omegaRadiansPerSecond;
  }

  @Override
  public void drive(ChassisSpeeds speeds) {
    driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, true);
  }

  @Override
  public void resetPose(Pose2d pose) {
    driveSimulation.setSimulationWorldPose(pose);
    driveSimulation.resetOdometry(pose);
  }

  @Override
  public void stop() {
    drive(new ChassisSpeeds());
  }

  private static DriveTrainSimulationConfig createDriveTrainConfig() {
    return DriveTrainSimulationConfig.Default()
        .withRobotMass(DriveSimulationConstants.ROBOT_MASS)
        .withBumperSize(
            DriveSimulationConstants.BUMPER_LENGTH_X, DriveSimulationConstants.BUMPER_WIDTH_Y)
        .withCustomModuleTranslations(DriveSimulationConstants.MODULE_TRANSLATIONS)
        .withSwerveModule(() -> DriveSimulationConstants.createModuleConfig().get())
        .withGyro(COTS.ofGenericGyro());
  }
}
