package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import frc.robot.subsystems.drive.DrivePathPlanner;
import org.junit.jupiter.api.Test;

class PathPlannerConfigurationTest {
  private static final double kPoseToleranceMeters = 0.05;

  @Test
  void straightTestPathLoadsSuccessfully() throws Exception {
    PathPlannerPath path = PathPlannerPath.fromPathFile("StraightTest");

    assertNotNull(path);
    assertEquals(2, path.getWaypoints().size());
  }

  @Test
  void straightTestTrajectoryUsesConfiguredRobotConfig() throws Exception {
    PathPlannerPath path = PathPlannerPath.fromPathFile("StraightTest");
    PathPlannerTrajectory trajectory =
        path.getIdealTrajectory(DrivePathPlanner.createRobotConfig()).orElse(null);

    assertNotNull(trajectory);
    assertTrue(trajectory.getStates().size() > 1);
    assertTrue(trajectory.getTotalTimeSeconds() > 0.1);

    assertEquals(1.0, trajectory.getInitialPose().getX(), kPoseToleranceMeters);
    assertEquals(1.0, trajectory.getInitialPose().getY(), kPoseToleranceMeters);
    assertEquals(4.0, trajectory.getEndState().pose.getX(), kPoseToleranceMeters);
    assertEquals(1.0, trajectory.getEndState().pose.getY(), kPoseToleranceMeters);
  }
}
