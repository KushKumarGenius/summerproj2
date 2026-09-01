package frc.robot.subsystems.vision;

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
