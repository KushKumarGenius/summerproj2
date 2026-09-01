package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("Yaw_Rot2d", yaw_Rot2d);
    table.put("Pitch_Rot2d", pitch_Rot2d);
    table.put("Roll_Rot2d", roll_Rot2d);
    table.put("YawVelocity_radps", yawVelocity_radps);
    table.put("OdometryTimestamps_s", odometryTimestamps_s);
    table.put("OdometryYawPositions_Rot2d", odometryYawPositions_Rot2d);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    yaw_Rot2d = table.get("Yaw_Rot2d", yaw_Rot2d);
    pitch_Rot2d = table.get("Pitch_Rot2d", pitch_Rot2d);
    roll_Rot2d = table.get("Roll_Rot2d", roll_Rot2d);
    yawVelocity_radps = table.get("YawVelocity_radps", yawVelocity_radps);
    odometryTimestamps_s = table.get("OdometryTimestamps_s", odometryTimestamps_s);
    odometryYawPositions_Rot2d =
        table.get("OdometryYawPositions_Rot2d", odometryYawPositions_Rot2d);
  }

  @Override
  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = connected;
    copy.yaw_Rot2d = yaw_Rot2d;
    copy.pitch_Rot2d = pitch_Rot2d;
    copy.roll_Rot2d = roll_Rot2d;
    copy.yawVelocity_radps = yawVelocity_radps;
    copy.odometryTimestamps_s = odometryTimestamps_s.clone();
    copy.odometryYawPositions_Rot2d = odometryYawPositions_Rot2d.clone();
    return copy;
  }
}
