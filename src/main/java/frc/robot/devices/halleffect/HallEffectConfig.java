package frc.robot.devices.halleffect;

import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class HallEffectConfig {
  public final int channel;
  public boolean inverted;
  public double debounceSeconds;
  public DebounceType debounceType = DebounceType.kBoth;

  public HallEffectConfig(int channel) {
    this.channel = channel;
  }

  public HallEffectConfig withInverted(boolean inverted) {
    this.inverted = inverted;
    return this;
  }

  public HallEffectConfig withDebounce(double seconds, DebounceType type) {
    debounceSeconds = seconds;
    debounceType = type;
    return this;
  }
}
