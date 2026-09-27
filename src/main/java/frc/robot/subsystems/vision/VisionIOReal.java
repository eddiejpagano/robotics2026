// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOReal implements VisionIO {
  private final PhotonCamera camera = new PhotonCamera(VisionConstants.kFrontCameraName);
  private final PhotonPoseEstimator poseEstimator;
  private PhotonPipelineResult latestResult = new PhotonPipelineResult();

  public VisionIOReal() {
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    poseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, VisionConstants.kRobotToFrontCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
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
