package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/** Simulates a camera that produces a valid field pose at a fixed rate. */
public class VisionIOSim implements VisionIO {
  private final Supplier<Pose2d> poseSupplier;
  private final double updatePeriodSeconds;
  private double lastUpdate = -1.0;

  public VisionIOSim(Supplier<Pose2d> poseSupplier) {
    this(poseSupplier, 0.1);
  }

  public VisionIOSim(Supplier<Pose2d> poseSupplier, double updatePeriodSeconds) {
    this.poseSupplier = poseSupplier;
    this.updatePeriodSeconds = updatePeriodSeconds;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    inputs.connected = true;
    inputs.hasEstimate = now - lastUpdate >= updatePeriodSeconds;
    inputs.tagCount = inputs.hasEstimate ? 1 : 0;
    if (inputs.hasEstimate) {
      inputs.estimatedPose = poseSupplier.get();
      inputs.timestampSeconds = now;
      lastUpdate = now;
    }
  }
}
