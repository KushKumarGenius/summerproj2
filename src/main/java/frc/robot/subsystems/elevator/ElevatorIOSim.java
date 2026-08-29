package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;

/** Simple gravity-aware elevator model for desktop simulation. */
public class ElevatorIOSim implements ElevatorIO {
  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private static final double MAX_HEIGHT_INCHES = 48.7;
  private static final double MAX_SPEED_INCHES_PER_SECOND = 100.0;

  private double appliedVolts;
  private double heightInches;
  private double velocityInchesPerSecond;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    double targetSpeed = appliedVolts / 12.0 * MAX_SPEED_INCHES_PER_SECOND;
    velocityInchesPerSecond += (targetSpeed - velocityInchesPerSecond) * 0.18;
    heightInches = MathUtil.clamp(heightInches + velocityInchesPerSecond * LOOP_PERIOD_SECONDS, 0.0, MAX_HEIGHT_INCHES);
    if (heightInches <= 0.0 || heightInches >= MAX_HEIGHT_INCHES) {
      velocityInchesPerSecond = 0.0;
    }

    inputs.heightInches = heightInches;
    inputs.velocityInchesPerSecond = velocityInchesPerSecond;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(appliedVolts) * 2.5;
    inputs.atLowerLimit = heightInches <= 0.0;
    inputs.atUpperLimit = heightInches >= MAX_HEIGHT_INCHES;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void resetPosition(double heightInches) {
    this.heightInches = MathUtil.clamp(heightInches, 0.0, MAX_HEIGHT_INCHES);
    velocityInchesPerSecond = 0.0;
  }
}
