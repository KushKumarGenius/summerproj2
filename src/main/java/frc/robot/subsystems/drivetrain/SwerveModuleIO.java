package frc.robot.subsystems.drivetrain;

/** Hardware boundary for one drive motor, one steering motor, and one absolute encoder. */
public interface SwerveModuleIO {
  class SwerveModuleIOInputs {
    public double driveDistanceMeters;
    public double driveVelocityMetersPerSecond;
    public double driveAppliedVolts;
    public double angleRadians;
    public double angleVelocityRadiansPerSecond;
    public double angleAppliedVolts;
  }

  default void updateInputs(SwerveModuleIOInputs inputs) {}

  default void setDriveVoltage(double volts) {}

  default void setSteerAngle(double angleRadians) {}

  default void resetToAbsolute() {}
}
