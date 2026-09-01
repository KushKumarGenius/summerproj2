package frc.robot.devices.halleffect;

import edu.wpi.first.wpilibj.DigitalInput;

public class HallEffectIODIO implements HallEffectIO {
  private final HallEffectConfig config;
  private final DigitalInput input;

  public HallEffectIODIO(HallEffectConfig config) {
    this.config = config;
    input = config.channel >= 0 ? new DigitalInput(config.channel) : null;
  }

  @Override
  public void updateInputs(HallEffectIOInputs inputs) {
    if (input == null) {
      inputs.connected = false;
      inputs.detected = false;
      return;
    }
    boolean rawValue = input.get();
    inputs.connected = true;
    inputs.detected = config.inverted ? !rawValue : rawValue;
  }
}
