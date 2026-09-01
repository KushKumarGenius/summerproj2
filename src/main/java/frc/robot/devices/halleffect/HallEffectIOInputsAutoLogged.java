package frc.robot.devices.halleffect;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HallEffectIOInputsAutoLogged extends HallEffectIO.HallEffectIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("Detected", detected);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    detected = table.get("Detected", detected);
  }

  @Override
  public HallEffectIOInputsAutoLogged clone() {
    HallEffectIOInputsAutoLogged copy = new HallEffectIOInputsAutoLogged();
    copy.connected = connected;
    copy.detected = detected;
    return copy;
  }
}
