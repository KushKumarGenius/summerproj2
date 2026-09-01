package frc.robot.subsystems.drivetrain;

public class GyroIOSim implements GyroIO {
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = false;
  }
}
