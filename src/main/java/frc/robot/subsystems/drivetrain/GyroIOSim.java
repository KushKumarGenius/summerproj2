package frc.robot.subsystems.drivetrain;

/** Simulation gyro boundary; the drivetrain derives heading from module kinematics. */
public class GyroIOSim implements GyroIO {
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Mark the gyro unavailable so Drivetrain uses the measured module twist as its heading.
    inputs.connected = false;
  }
}
