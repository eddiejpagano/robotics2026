// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSim implements VisionIO {
  private final Supplier<Pose2d> robotPoseSupplier;
  private final PhotonCamera camera = new PhotonCamera(VisionConstants.kFrontCameraName);
  private final VisionSystemSim visionSystem = new VisionSystemSim("main");
  private final PhotonPoseEstimator poseEstimator;
  private PhotonPipelineResult latestResult = new PhotonPipelineResult();

  public VisionIOSim(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    visionSystem.addAprilTags(fieldLayout);
    poseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, VisionConstants.kRobotToFrontCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(90.0));
    cameraProperties.setCalibError(0.25, 0.08);
    cameraProperties.setFPS(30.0);
    cameraProperties.setAvgLatencyMs(35.0);
    cameraProperties.setLatencyStdDevMs(5.0);

    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProperties);
    cameraSim.setMaxSightRange(8.0);
    cameraSim.setMinTargetAreaPercent(0.02);
    cameraSim.enableDrawWireframe(true);
    visionSystem.addCamera(cameraSim, VisionConstants.kRobotToFrontCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Pose2d groundTruthPose = robotPoseSupplier.get();
    inputs.simulationGroundTruthPose = groundTruthPose;
    visionSystem.update(groundTruthPose);

    inputs.connected = camera.isConnected();

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      latestResult = results.get(results.size() - 1);
    }

    PhotonPipelineResult result = latestResult;
    inputs.hasTargets = result.hasTargets();
    inputs.targetCount = result.getTargets().size();
    inputs.latestTimestampSeconds = result.getTimestampSeconds();

    if (result.hasTargets()) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      inputs.bestTargetId = bestTarget.getFiducialId();
      inputs.bestTargetYawDegrees = bestTarget.getYaw();
      inputs.bestTargetPitchDegrees = bestTarget.getPitch();
    } else {
      inputs.bestTargetId = -1;
      inputs.bestTargetYawDegrees = 0.0;
      inputs.bestTargetPitchDegrees = 0.0;
    }

    Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
    inputs.estimatedPoseValid = estimatedPose.isPresent();
    if (estimatedPose.isPresent()) {
      EstimatedRobotPose estimate = estimatedPose.get();
      inputs.estimatedPose = estimate.estimatedPose;
      inputs.estimatedTimestampSeconds = estimate.timestampSeconds;
    } else {
      inputs.estimatedTimestampSeconds = 0.0;
    }
  }
}
