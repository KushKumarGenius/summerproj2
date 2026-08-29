package frc.robot.subsystems.stinger;

import edu.wpi.first.math.MathUtil;

/** Simple extension model for desktop simulation. */
public class StingerIOSim implements StingerIO {
  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private static final double MAX_EXTENSION_INCHES = 25.0;
  private static final double MAX_SPEED_INCHES_PER_SECOND = 100.0;

  private double appliedVolts;
  private double extensionInches;
  private double velocityInchesPerSecond;

  @Override
  public void updateInputs(StingerIOInputs inputs) {
    double targetSpeed = appliedVolts / 12.0 * MAX_SPEED_INCHES_PER_SECOND;
    velocityInchesPerSecond += (targetSpeed - velocityInchesPerSecond) * 0.22;
    extensionInches =
        MathUtil.clamp(
            extensionInches + velocityInchesPerSecond * LOOP_PERIOD_SECONDS,
            0.0,
            MAX_EXTENSION_INCHES);
    if (extensionInches <= 0.0 || extensionInches >= MAX_EXTENSION_INCHES) {
      velocityInchesPerSecond = 0.0;
    }

    inputs.extensionInches = extensionInches;
    inputs.velocityInchesPerSecond = velocityInchesPerSecond;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(appliedVolts) * 2.0;
    inputs.atRetractedLimit = extensionInches <= 0.0;
    inputs.atExtendedLimit = extensionInches >= MAX_EXTENSION_INCHES;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void resetPosition(double extensionInches) {
    this.extensionInches = MathUtil.clamp(extensionInches, 0.0, MAX_EXTENSION_INCHES);
    velocityInchesPerSecond = 0.0;
  }
}
