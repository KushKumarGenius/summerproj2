package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOReal implements VisionIO {
  private final String cameraName;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final Supplier<Pose2d> referencePoseSupplier;
  private double lastTimestampSeconds;

  public VisionIOReal(
      String cameraName, Transform3d robotToCamera, Supplier<Pose2d> referencePoseSupplier) {
    this.cameraName = cameraName;
    this.referencePoseSupplier = referencePoseSupplier;
    camera = new PhotonCamera(cameraName);
    camera.setDriverMode(false);
    poseEstimator =
        new PhotonPoseEstimator(
            loadFieldLayout(), PoseStrategy.MULTI_TAG_PNP_ON_RIO, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    PhotonPipelineResult result = camera.getLatestResult();
    double timestampSeconds = result.getTimestampSeconds();

    inputs.connected = camera.isConnected();
    inputs.hasEstimate = false;
    inputs.tagCount = result.getTargets().size();
    inputs.timestampSeconds = timestampSeconds;

    if (timestampSeconds <= lastTimestampSeconds || inputs.tagCount == 0) {
      return;
    }

    poseEstimator.setReferencePose(referencePoseSupplier.get());
    Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
    if (estimatedPose.isPresent()) {
      inputs.hasEstimate = true;
      inputs.estimatedPose = estimatedPose.get().estimatedPose.toPose2d();
      inputs.timestampSeconds = estimatedPose.get().timestampSeconds;
      lastTimestampSeconds = inputs.timestampSeconds;
    }
  }

  public String getCameraName() {
    return cameraName;
  }

  private static AprilTagFieldLayout loadFieldLayout() {
    try {
      return AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (RuntimeException exception) {
      return new AprilTagFieldLayout(
          java.util.List.of(),
          frc.robot.FieldConstants.FIELD_LENGTH_METERS,
          frc.robot.FieldConstants.FIELD_WIDTH_METERS);
    }
  }
}
