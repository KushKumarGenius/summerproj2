package frc.robot.devices.halleffect;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public class HallEffect {
  private final String name;
  private final HallEffectIO io;
  private final HallEffectIOInputsAutoLogged inputs = new HallEffectIOInputsAutoLogged();
  private final Debouncer debouncer;
  private final boolean debounceEnabled;
  private boolean detected;

  public HallEffect(String name, HallEffectConfig config) {
    this.name = name;
    io = RobotBase.isReal() ? new HallEffectIODIO(config) : new HallEffectIOSim();
    debounceEnabled = config.debounceSeconds > 0.0;
    debouncer = new Debouncer(config.debounceSeconds, config.debounceType);
  }

  public void readInputs() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    detected = debounceEnabled ? debouncer.calculate(inputs.detected) : inputs.detected;
    Logger.recordOutput(name + "/Detected", detected);
  }

  public boolean get() {
    return detected;
  }

  public boolean getRaw() {
    return inputs.detected;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public void setSimState(boolean detected) {
    io.setSimDetected(detected);
  }
}
