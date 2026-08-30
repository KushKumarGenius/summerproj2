package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware boundary for the robot gyro. */
public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yaw_Rot2d = new Rotation2d();
    public Rotation2d pitch_Rot2d = new Rotation2d();
    public Rotation2d roll_Rot2d = new Rotation2d();
    public double yawVelocity_radps = 0.0;
    public double[] odometryTimestamps_s = new double[] {};
    public Rotation2d[] odometryYawPositions_Rot2d = new Rotation2d[] {};
  }

  default void updateInputs(GyroIOInputs inputs) {}

  default void zero(double angleDegrees) {}
}
