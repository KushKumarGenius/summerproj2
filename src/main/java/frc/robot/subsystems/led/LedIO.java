package frc.robot.subsystems.led;

public interface LedIO {
  default void setPattern(double pattern) {}

  default double getPattern() {
    return 0.0;
  }
}
