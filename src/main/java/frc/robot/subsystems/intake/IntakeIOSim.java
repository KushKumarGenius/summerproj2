package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;

public class IntakeIOSim implements IntakeIO {
  // Stores the voltage that the simulated motor is receiving.
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
    // Fill the same inputs object that a real IO class would fill with
    // data from a motor controller and sensors.
    inputs.appliedVolts = appliedVolts;

    // Simple educational model: more voltage produces more RPM.
    // This is not an exact model of the real 2930 mechanism.
    inputs.velocityRPM = appliedVolts / 12.0 * 5000.0;
  }

  @Override
  public void setVoltage(double volts) {
    // Keep the requested voltage inside the normal -12 to +12 volt range.
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}