package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  class SwerveModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePosition_m = 0.0;
    public double driveVelocity_mps = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean steerConnected = false;
    public boolean steerEncoderConnected = false;
    public Rotation2d steerAbsolutePosition_Rot2d = new Rotation2d();
    public Rotation2d steerPosition_Rot2d = new Rotation2d();
    public double steerVelocity_radps = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerCurrentAmps = 0.0;

    public double[] odometryTimestamps_s = new double[] {};
    public double[] odometryDrivePositions_m = new double[] {};
    public Rotation2d[] odometrySteerPositions_Rot2d = new Rotation2d[] {};
  }

  default void updateInputs(SwerveModuleIOInputs inputs) {}

  default void setDriveOpenLoop(double volts) {}

  default void setSteerOpenLoop(double volts) {}

  default void setSteerPosition(Rotation2d angle) {}

  default void setDriveVelocity(double velocityMetersPerSecond) {}

  default void setBrakeMode(boolean enabled) {}

  default void resetToAbsolute() {}

  default void stop() {}
}
