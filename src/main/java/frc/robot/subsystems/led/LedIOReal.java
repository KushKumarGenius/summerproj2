package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LedIOReal implements LedIO {
  private final Spark controller;

  public LedIOReal(int pwmPort) {
    controller = new Spark(pwmPort);
  }

  @Override
  public void setPattern(double pattern) {
    controller.set(pattern);
  }

  @Override
  public double getPattern() {
    return controller.get();
  }
}
