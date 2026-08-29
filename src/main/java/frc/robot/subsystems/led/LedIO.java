package frc.robot.subsystems.led;

/** Hardware boundary for the REV Blinkin LED controller. */
public interface LedIO {
  default void setPattern(double pattern) {}

  default double getPattern() {
    return 0.0;
  }
}
