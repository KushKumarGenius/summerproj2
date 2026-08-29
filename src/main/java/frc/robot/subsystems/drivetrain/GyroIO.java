package frc.robot.subsystems.drivetrain;

/** Hardware boundary for the robot gyro. */
public interface GyroIO {
  class GyroIOInputs {
    public boolean connected;
    public double yawRadians;
    public double pitchRadians;
    public double rollRadians;
  }

  default void updateInputs(GyroIOInputs inputs) {}
}
