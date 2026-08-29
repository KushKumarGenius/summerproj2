package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/** Hardware boundary for one AprilTag camera. */
public interface VisionIO {
  class VisionIOInputs {
    public boolean connected;
    public boolean hasEstimate;
    public int tagCount;
    public double timestampSeconds;
    public Pose2d estimatedPose = new Pose2d();
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
