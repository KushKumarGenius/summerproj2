package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Generated-style AdvantageKit log wrapper for SwerveModuleIOInputs. */
public class SwerveModuleIOInputsAutoLogged extends SwerveModuleIO.SwerveModuleIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DriveConnected", driveConnected);
    table.put("DrivePosition_m", drivePosition_m);
    table.put("DriveVelocity_mps", driveVelocity_mps);
    table.put("DriveAppliedVolts", driveAppliedVolts);
    table.put("DriveCurrentAmps", driveCurrentAmps);
    table.put("SteerConnected", steerConnected);
    table.put("SteerEncoderConnected", steerEncoderConnected);
    table.put("SteerAbsolutePosition_Rot2d", steerAbsolutePosition_Rot2d);
    table.put("SteerPosition_Rot2d", steerPosition_Rot2d);
    table.put("SteerVelocity_radps", steerVelocity_radps);
    table.put("SteerAppliedVolts", steerAppliedVolts);
    table.put("SteerCurrentAmps", steerCurrentAmps);
    table.put("OdometryTimestamps_s", odometryTimestamps_s);
    table.put("OdometryDrivePositions_m", odometryDrivePositions_m);
    table.put("OdometrySteerPositions_Rot2d", odometrySteerPositions_Rot2d);
  }

  @Override
  public void fromLog(LogTable table) {
    driveConnected = table.get("DriveConnected", driveConnected);
    drivePosition_m = table.get("DrivePosition_m", drivePosition_m);
    driveVelocity_mps = table.get("DriveVelocity_mps", driveVelocity_mps);
    driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
    driveCurrentAmps = table.get("DriveCurrentAmps", driveCurrentAmps);
    steerConnected = table.get("SteerConnected", steerConnected);
    steerEncoderConnected = table.get("SteerEncoderConnected", steerEncoderConnected);
    steerAbsolutePosition_Rot2d =
        table.get("SteerAbsolutePosition_Rot2d", steerAbsolutePosition_Rot2d);
    steerPosition_Rot2d = table.get("SteerPosition_Rot2d", steerPosition_Rot2d);
    steerVelocity_radps = table.get("SteerVelocity_radps", steerVelocity_radps);
    steerAppliedVolts = table.get("SteerAppliedVolts", steerAppliedVolts);
    steerCurrentAmps = table.get("SteerCurrentAmps", steerCurrentAmps);
    odometryTimestamps_s = table.get("OdometryTimestamps_s", odometryTimestamps_s);
    odometryDrivePositions_m = table.get("OdometryDrivePositions_m", odometryDrivePositions_m);
    odometrySteerPositions_Rot2d =
        table.get("OdometrySteerPositions_Rot2d", odometrySteerPositions_Rot2d);
  }

  @Override
  public SwerveModuleIOInputsAutoLogged clone() {
    SwerveModuleIOInputsAutoLogged copy = new SwerveModuleIOInputsAutoLogged();
    copy.driveConnected = driveConnected;
    copy.drivePosition_m = drivePosition_m;
    copy.driveVelocity_mps = driveVelocity_mps;
    copy.driveAppliedVolts = driveAppliedVolts;
    copy.driveCurrentAmps = driveCurrentAmps;
    copy.steerConnected = steerConnected;
    copy.steerEncoderConnected = steerEncoderConnected;
    copy.steerAbsolutePosition_Rot2d = steerAbsolutePosition_Rot2d;
    copy.steerPosition_Rot2d = steerPosition_Rot2d;
    copy.steerVelocity_radps = steerVelocity_radps;
    copy.steerAppliedVolts = steerAppliedVolts;
    copy.steerCurrentAmps = steerCurrentAmps;
    copy.odometryTimestamps_s = odometryTimestamps_s.clone();
    copy.odometryDrivePositions_m = odometryDrivePositions_m.clone();
    copy.odometrySteerPositions_Rot2d = odometrySteerPositions_Rot2d.clone();
    return copy;
  }
}
