package frc.robot.subsystems.intake;

public interface IntakeIO {
  class IntakeIOInputs {
    public double appliedVolts;
    public double velocityRPM;
    public double currentAmps;
    public boolean hasPiece;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0.0);
  }
}
