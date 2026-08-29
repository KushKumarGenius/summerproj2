package frc.robot.subsystems.intake;

/** Hardware boundary for the intake motor and game-piece sensor. */
public interface IntakeIO {
  /** Values read from the motor controller and sensors. */
  class IntakeIOInputs {
    public double appliedVolts;
    public double velocityRPM;
    public double currentAmps;
    public boolean hasPiece;
  }

  /** Refresh the inputs object with the latest hardware values. */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Command the intake motor with a voltage from -12 to +12 volts. */
  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0.0);
  }
}
