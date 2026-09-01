package frc.robot.devices.motor;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MotorIOInputsAutoLogged extends MotorIO.MotorIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("Position", position);
    table.put("Velocity", velocity);
    table.put("Acceleration", acceleration);
    table.put("AppliedVolts", appliedVolts);
    table.put("SupplyCurrent", supplyCurrent);
    table.put("StatorCurrent", statorCurrent);
    table.put("TemperatureCelsius", temperatureCelsius);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    position = table.get("Position", position);
    velocity = table.get("Velocity", velocity);
    acceleration = table.get("Acceleration", acceleration);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    supplyCurrent = table.get("SupplyCurrent", supplyCurrent);
    statorCurrent = table.get("StatorCurrent", statorCurrent);
    temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
  }

  @Override
  public MotorIOInputsAutoLogged clone() {
    MotorIOInputsAutoLogged copy = new MotorIOInputsAutoLogged();
    copy.connected = connected;
    copy.position = position;
    copy.velocity = velocity;
    copy.acceleration = acceleration;
    copy.appliedVolts = appliedVolts;
    copy.supplyCurrent = supplyCurrent;
    copy.statorCurrent = statorCurrent;
    copy.temperatureCelsius = temperatureCelsius;
    return copy;
  }
}
