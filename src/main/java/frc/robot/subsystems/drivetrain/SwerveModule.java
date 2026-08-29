package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Converts chassis requests into commands for one swerve module. */
public class SwerveModule {
  private final SwerveModuleIO io;
  private final SwerveModuleIO.SwerveModuleIOInputs inputs =
      new SwerveModuleIO.SwerveModuleIOInputs();
  private final int moduleNumber;
  private final double maxSpeed;
  private Rotation2d lastAngle = new Rotation2d();

  public SwerveModule(SwerveModuleIO io, int moduleNumber, double maxSpeed) {
    this.io = io;
    this.moduleNumber = moduleNumber;
    this.maxSpeed = maxSpeed;
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean openLoop, boolean forceAngle) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

    if (openLoop) {
      io.setDriveVoltage(desiredState.speedMetersPerSecond / maxSpeed * 12.0);
    } else {
      io.setDriveVoltage(desiredState.speedMetersPerSecond / maxSpeed * 12.0);
    }

    if (forceAngle || Math.abs(desiredState.speedMetersPerSecond) > maxSpeed * 0.01) {
      lastAngle = desiredState.angle;
    }
    io.setSteerAngle(lastAngle.getRadians());
  }

  public void setCharacterizationVoltage(double volts) {
    lastAngle = new Rotation2d();
    io.setSteerAngle(0.0);
    io.setDriveVoltage(volts);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocityMetersPerSecond, new Rotation2d(inputs.angleRadians));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(inputs.driveDistanceMeters, new Rotation2d(inputs.angleRadians));
  }

  public int getModuleNumber() {
    return moduleNumber;
  }

  public void resetToAbsolute() {
    io.resetToAbsolute();
  }
}
