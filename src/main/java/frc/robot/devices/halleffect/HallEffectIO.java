package frc.robot.devices.halleffect;

import org.littletonrobotics.junction.AutoLog;

public interface HallEffectIO {
  @AutoLog
  class HallEffectIOInputs {
    public boolean connected;
    public boolean detected;
  }

  default void updateInputs(HallEffectIOInputs inputs) {}

  default void setSimDetected(boolean detected) {}
}
