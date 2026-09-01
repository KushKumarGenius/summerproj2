package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleIOSim implements SwerveModuleIO {
  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private static final double STEER_KP_VOLTS_PER_RADIAN = 8.5 / (2.0 * Math.PI);
  private static final double SIM_INERTIA_KG_M2 = 0.0001;

  private final DCMotorSim driveSim;
  private final DCMotorSim steerSim;
  private final PIDController driveController = new PIDController(0.05, 0.0, 0.0);
  private final PIDController steerController =
      new PIDController(STEER_KP_VOLTS_PER_RADIAN, 0.0, 0.0);

  private double driveAppliedVolts;
  private double steerAppliedVolts;
  private double driveVelocitySetpointRadPerSec;
  private boolean driveClosedLoop;
  private boolean steerClosedLoop;

  public SwerveModuleIOSim(int moduleNumber) {
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getFalcon500(1),
                SIM_INERTIA_KG_M2,
                DrivetrainConstants.DRIVE_GEAR_RATIO),
            DCMotor.getFalcon500(1));
    steerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getFalcon500(1), SIM_INERTIA_KG_M2, 1.0),
            DCMotor.getFalcon500(1));
    steerController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    if (driveClosedLoop) {
      driveAppliedVolts +=
          driveController.calculate(
              driveSim.getAngularVelocityRadPerSec(), driveVelocitySetpointRadPerSec);
    } else {
      driveController.reset();
    }

    if (steerClosedLoop) {
      steerAppliedVolts =
          steerController.calculate(steerSim.getAngularPositionRad(), steerController.getSetpoint());
    } else {
      steerController.reset();
    }

    driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
    steerAppliedVolts = MathUtil.clamp(steerAppliedVolts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
    steerSim.setInputVoltage(steerAppliedVolts);
    driveSim.update(LOOP_PERIOD_SECONDS);
    steerSim.update(LOOP_PERIOD_SECONDS);

    double driveRotations = Units.radiansToRotations(driveSim.getAngularPositionRad());
    double driveVelocityRotationsPerSecond =
        Units.radiansToRotations(driveSim.getAngularVelocityRadPerSec());
    inputs.driveConnected = true;
    inputs.drivePosition_m = driveRotations * DrivetrainConstants.WHEEL_CIRCUMFERENCE_METERS;
    inputs.driveVelocity_mps =
        driveVelocityRotationsPerSecond * DrivetrainConstants.WHEEL_CIRCUMFERENCE_METERS;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.steerConnected = true;
    inputs.steerEncoderConnected = true;
    inputs.steerPosition_Rot2d = new Rotation2d(steerSim.getAngularPositionRad());
    inputs.steerAbsolutePosition_Rot2d = inputs.steerPosition_Rot2d;
    inputs.steerVelocity_radps = steerSim.getAngularVelocityRadPerSec();
    inputs.steerAppliedVolts = steerAppliedVolts;
    inputs.steerCurrentAmps = Math.abs(steerSim.getCurrentDrawAmps());

    inputs.odometryTimestamps_s = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositions_m = new double[] {inputs.drivePosition_m};
    inputs.odometrySteerPositions_Rot2d = new Rotation2d[] {inputs.steerPosition_Rot2d};
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveClosedLoop = false;
    driveAppliedVolts = volts;
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSecond) {
    driveClosedLoop = true;
    driveVelocitySetpointRadPerSec =
        velocityMetersPerSecond
            / DrivetrainConstants.WHEEL_CIRCUMFERENCE_METERS
            * 2.0
            * Math.PI;
  }

  @Override
  public void setSteerOpenLoop(double volts) {
    steerClosedLoop = false;
    steerAppliedVolts = volts;
  }

  @Override
  public void setSteerPosition(Rotation2d angle) {
    steerClosedLoop = true;
    steerController.setSetpoint(angle.getRadians());
  }

  @Override
  public void setBrakeMode(boolean enabled) {}

  @Override
  public void stop() {
    driveClosedLoop = false;
    steerClosedLoop = false;
    driveAppliedVolts = 0.0;
    steerAppliedVolts = 0.0;
  }
}
