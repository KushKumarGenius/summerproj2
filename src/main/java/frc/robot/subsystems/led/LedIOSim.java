package frc.robot.subsystems.led;

/** Stores the selected LED pattern for simulation and dashboard inspection. */
public class LedIOSim implements LedIO {
  private double pattern;

  @Override
  public void setPattern(double pattern) {
    this.pattern = pattern;
  }

  @Override
  public double getPattern() {
    return pattern;
  }
}
