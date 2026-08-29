package frc.robot.subsystems.elevator;

/** Hardware boundary for the vertical elevator. */
public interface ElevatorIO {
  class ElevatorIOInputs {
    public double heightInches;
    public double velocityInchesPerSecond;
    public double appliedVolts;
    public double currentAmps;
    public boolean atLowerLimit;
    public boolean atUpperLimit;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {
    setVoltage(0.0);
  }

  default void resetPosition(double heightInches) {}
}
