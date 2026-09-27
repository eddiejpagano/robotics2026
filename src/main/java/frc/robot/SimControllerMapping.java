// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import java.util.Locale;
import org.littletonrobotics.junction.Logger;

public final class SimControllerMapping {
  public static final String kProfileEnvironmentVariable = "FRC_SIM_CONTROLLER_PROFILE";

  public enum Profile {
    STANDARD_XBOX("standard", 0, 1, 4, 5),
    MAC_BLUETOOTH_XBOX("mac-bluetooth", 0, 1, 2, 3);

    private final String environmentValue;
    private final int leftXAxis;
    private final int leftYAxis;
    private final int rightXAxis;
    private final int rightYAxis;

    Profile(
        String environmentValue, int leftXAxis, int leftYAxis, int rightXAxis, int rightYAxis) {
      this.environmentValue = environmentValue;
      this.leftXAxis = leftXAxis;
      this.leftYAxis = leftYAxis;
      this.rightXAxis = rightXAxis;
      this.rightYAxis = rightYAxis;
    }
  }

  private final Profile profile;

  private SimControllerMapping(Profile profile) {
    this.profile = profile;
  }

  public static SimControllerMapping select() {
    String override = System.getenv(kProfileEnvironmentVariable);
    if (override != null && !override.isBlank()) {
      String normalizedOverride = override.trim().toLowerCase(Locale.ROOT);
      for (Profile profile : Profile.values()) {
        if (profile.environmentValue.equals(normalizedOverride)) {
          return new SimControllerMapping(profile);
        }
      }
      return new SimControllerMapping(Profile.STANDARD_XBOX);
    }

    String osName = System.getProperty("os.name", "").toLowerCase(Locale.ROOT);
    if (osName.contains("mac")) {
      return new SimControllerMapping(Profile.MAC_BLUETOOTH_XBOX);
    }

    return new SimControllerMapping(Profile.STANDARD_XBOX);
  }

  public String getProfileName() {
    return profile.environmentValue;
  }

  public double getLeftX(GenericHID controller) {
    return controller.getRawAxis(profile.leftXAxis);
  }

  public double getLeftY(GenericHID controller) {
    return controller.getRawAxis(profile.leftYAxis);
  }

  public double getRightX(GenericHID controller) {
    return controller.getRawAxis(profile.rightXAxis);
  }

  public double getRightY(GenericHID controller) {
    return controller.getRawAxis(profile.rightYAxis);
  }

  public void recordDiagnostics(GenericHID controller) {
    Logger.recordOutput("Driver/LeftXRaw", getLeftX(controller));
    Logger.recordOutput("Driver/LeftYRaw", getLeftY(controller));
    Logger.recordOutput("Driver/RightXRaw", getRightX(controller));
    Logger.recordOutput("Driver/RightYRaw", getRightY(controller));

    for (int axis = 0; axis <= 5; axis++) {
      Logger.recordOutput("Driver/RawAxis" + axis, controller.getRawAxis(axis));
    }
  }
}
