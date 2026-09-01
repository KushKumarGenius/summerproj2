package frc.robot.devices.motor;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
  @AutoLog
  class MotorIOInputs {
    public boolean connected;
    public double position;
    public double velocity;
    public double acceleration;
    public double appliedVolts;
    public double supplyCurrent;
    public double statorCurrent;
    public double temperatureCelsius;
  }

  default void updateInputs(MotorIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void setOpenLoop(double dutyCycle) {}

  default void setVelocity(double velocity) {}

  default void setPositionVoltage(double position) {}

  default void setMotionMagic(double position) {}

  default void setMotionMagicVelocity(double velocity) {}

  default void stop() {}

  default void setBrakeMode(boolean brake) {}

  default void zeroPosition(double position) {}
}
