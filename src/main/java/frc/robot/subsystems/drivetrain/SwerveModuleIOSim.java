package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;

/** Lightweight swerve module model for desktop simulation. */
public class SwerveModuleIOSim implements SwerveModuleIO {
  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private static final double MAX_SPEED_METERS_PER_SECOND = 4.5;
  private static final double MAX_STEER_RATE_RADIANS_PER_SECOND = 10.0;

  private final int moduleNumber;
  private double driveVoltage;
  private double steerVoltage;
  private double desiredAngle;
  private double angle;
  private double distance;
  private double velocity;

  public SwerveModuleIOSim(int moduleNumber) {
    this.moduleNumber = moduleNumber;
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    double targetVelocity = driveVoltage / 12.0 * MAX_SPEED_METERS_PER_SECOND;
    velocity += (targetVelocity - velocity) * 0.25;
    distance += velocity * LOOP_PERIOD_SECONDS;

    double error = Math.IEEEremainder(desiredAngle - angle, 2.0 * Math.PI);
    double angleStep = MathUtil.clamp(error, -MAX_STEER_RATE_RADIANS_PER_SECOND * LOOP_PERIOD_SECONDS,
        MAX_STEER_RATE_RADIANS_PER_SECOND * LOOP_PERIOD_SECONDS);
    angle += angleStep;

    inputs.driveDistanceMeters = distance;
    inputs.driveVelocityMetersPerSecond = velocity;
    inputs.driveAppliedVolts = driveVoltage;
    inputs.angleRadians = angle;
    inputs.angleVelocityRadiansPerSecond = angleStep / LOOP_PERIOD_SECONDS;
    inputs.angleAppliedVolts = steerVoltage;
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveVoltage = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setSteerAngle(double angleRadians) {
    desiredAngle = angleRadians;
    steerVoltage = MathUtil.clamp(Math.IEEEremainder(angleRadians - angle, 2.0 * Math.PI), -1.0, 1.0) * 12.0;
  }

  @Override
  public void resetToAbsolute() {
    angle = 0.0;
    desiredAngle = 0.0;
  }

  public int getModuleNumber() {
    return moduleNumber;
  }
}
