package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  public enum Mode {
    HIGH_SPEED,
    HIGH_CONTROL
  }

  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  private final int moduleNumber;
  private final double maxSpeedMetersPerSecond;

  private double driveSetpointMetersPerSecond;
  private Rotation2d steerSetpoint;
  private boolean absolutePositionInitialized;

  public SwerveModule(
      SwerveModuleIO io, int moduleNumber, double maxSpeedMetersPerSecond) {
    this.io = io;
    this.moduleNumber = moduleNumber;
    this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
    io.setBrakeMode(true);
  }

  public void inputPeriodic() {
    io.updateInputs(inputs);
    if (!absolutePositionInitialized) {
      io.resetToAbsolute();
      absolutePositionInitialized = true;
    }
    Logger.processInputs("Drive/Module" + moduleNumber, inputs);
  }

  public void periodic() {
    inputPeriodic();
  }

  public void outputPeriodic(Mode mode) {
    if (steerSetpoint == null) {
      io.stop();
      return;
    }

    io.setSteerPosition(steerSetpoint);
    io.setDriveOpenLoop(
        DrivetrainConstants.MAX_VOLTAGE
            * driveSetpointMetersPerSecond
            / maxSpeedMetersPerSecond);
  }

  @AutoLogOutput(key = "Drive/ModuleSetpoints/Module{moduleNumber}")
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
    optimized.cosineScale(getAngle());
    steerSetpoint = optimized.angle;
    driveSetpointMetersPerSecond = optimized.speedMetersPerSecond;
    return optimized;
  }

  public void setDesiredState(
      SwerveModuleState desiredState, boolean openLoop, boolean forceAngle) {
    SwerveModuleState optimized = SwerveModuleState.optimize(desiredState, getAngle());
    if (forceAngle || Math.abs(optimized.speedMetersPerSecond) > maxSpeedMetersPerSecond * 0.01) {
      steerSetpoint = optimized.angle;
    }
    driveSetpointMetersPerSecond = optimized.speedMetersPerSecond;
    outputPeriodic(Mode.HIGH_CONTROL);
  }

  public void runCharacterization(double volts) {
    steerSetpoint = new Rotation2d();
    driveSetpointMetersPerSecond = 0.0;
    io.setSteerPosition(steerSetpoint);
    io.setDriveOpenLoop(volts);
  }

  public void stop() {
    steerSetpoint = null;
    driveSetpointMetersPerSecond = 0.0;
    io.stop();
  }

  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  public Rotation2d getAngle() {
    return inputs.steerPosition_Rot2d;
  }

  @AutoLogOutput(key = "Drive/Module{moduleNumber}/Position")
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(inputs.drivePosition_m, getAngle());
  }

  @AutoLogOutput(key = "Drive/Module{moduleNumber}/State")
  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocity_mps, getAngle());
  }

  public SwerveModuleIO.SwerveModuleIOInputs getInputs() {
    return inputs;
  }

  public int getModuleNumber() {
    return moduleNumber;
  }
}
