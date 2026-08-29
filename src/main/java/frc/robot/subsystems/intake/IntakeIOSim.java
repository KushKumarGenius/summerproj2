package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;

/** Small first-order intake model used by desktop simulation and unit tests. */
public class IntakeIOSim implements IntakeIO {
  private static final double MAX_RPM = 5_000.0;
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private double appliedVolts;
  private double velocityRPM;
  private double intakeTime;
  private boolean hasPiece;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double targetRPM = appliedVolts / 12.0 * MAX_RPM;
    velocityRPM += (targetRPM - velocityRPM) * 0.2;

    if (appliedVolts > 6.0) {
      intakeTime += LOOP_PERIOD_SECONDS;
      if (intakeTime >= 0.75) {
        hasPiece = true;
      }
    } else if (appliedVolts < -1.0) {
      intakeTime = 0.0;
      hasPiece = false;
    } else if (Math.abs(appliedVolts) < 0.1) {
      intakeTime = 0.0;
    }

    inputs.appliedVolts = appliedVolts;
    inputs.velocityRPM = velocityRPM;
    inputs.currentAmps = Math.abs(appliedVolts) * 2.5 + (hasPiece ? 8.0 : 0.0);
    inputs.hasPiece = hasPiece;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
