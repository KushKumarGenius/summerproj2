package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("HasEstimate", hasEstimate);
    table.put("TagCount", tagCount);
    table.put("TimestampSeconds", timestampSeconds);
    table.put("EstimatedPose", estimatedPose);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    hasEstimate = table.get("HasEstimate", hasEstimate);
    tagCount = table.get("TagCount", tagCount);
    timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
    estimatedPose = table.get("EstimatedPose", estimatedPose);
  }

  @Override
  public VisionIOInputsAutoLogged clone() {
    VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
    copy.connected = connected;
    copy.hasEstimate = hasEstimate;
    copy.tagCount = tagCount;
    copy.timestampSeconds = timestampSeconds;
    copy.estimatedPose = estimatedPose;
    return copy;
  }
}
