package frc.robot.subsystems.stinger;

/** Hardware boundary for the horizontal extension mechanism. */
public interface StingerIO {
  class StingerIOInputs {
    public double extensionInches;
    public double velocityInchesPerSecond;
    public double appliedVolts;
    public double currentAmps;
    public boolean atRetractedLimit;
    public boolean atExtendedLimit;
  }

  default void updateInputs(StingerIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void resetPosition(double extensionInches) {}

  default void stop() {
    setVoltage(0.0);
  }
}
