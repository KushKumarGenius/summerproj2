package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Vision extends SubsystemBase {
  private final Drivetrain drivetrain;
  private final Camera[] cameras;
  private final AprilTagFieldLayout fieldLayout;
  private boolean enabled = true;

  public Vision(Drivetrain drivetrain, VisionIO... io) {
    if (io.length == 0) {
      throw new IllegalArgumentException("Vision needs at least one camera");
    }
    this.drivetrain = drivetrain;
    cameras = new Camera[io.length];
    for (int i = 0; i < io.length; i++) {
      cameras[i] = new Camera(io[i]);
    }
    fieldLayout = loadFieldLayout();
  }

  private static AprilTagFieldLayout loadFieldLayout() {
    try {
      return AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (Exception exception) {
      return new AprilTagFieldLayout(java.util.List.of(), 16.54175, 8.0137);
    }
  }

  @Override
  public void periodic() {
    int estimatesUsed = 0;
    int visibleTags = 0;
    for (Camera camera : cameras) {
      camera.io.updateInputs(camera.inputs);
      visibleTags += camera.inputs.tagCount;
      if (enabled && camera.inputs.hasEstimate) {
        drivetrain.addVisionMeasurement(
            camera.inputs.estimatedPose, camera.inputs.timestampSeconds);
        estimatesUsed++;
      }
    }

    SmartDashboard.putBoolean("Vision/Enabled", enabled);
    SmartDashboard.putNumber("Vision/EstimatesUsed", estimatesUsed);
    SmartDashboard.putNumber("Vision/VisibleTags", visibleTags);
    SmartDashboard.putNumber("Vision/FieldTagCount", fieldLayout.getTags().size());
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public Pose2d getRobotPose() {
    return drivetrain.getPose();
  }

  private static class Camera {
    private final VisionIO io;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    private Camera(VisionIO io) {
      this.io = io;
    }
  }
}
