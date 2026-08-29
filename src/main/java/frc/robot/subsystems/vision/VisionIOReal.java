package frc.robot.subsystems.vision;

/**
 * Safe placeholder for the PhotonVision adapter.
 *
 * <p>The camera names and network layout are kept in {@link VisionConstants}; the actual camera
 * library can be added here without changing the rest of the robot code.
 */
public class VisionIOReal implements VisionIO {
  private final String cameraName;

  public VisionIOReal(String cameraName) {
    this.cameraName = cameraName;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = false;
    inputs.hasEstimate = false;
    inputs.tagCount = 0;
  }

  public String getCameraName() {
    return cameraName;
  }
}
