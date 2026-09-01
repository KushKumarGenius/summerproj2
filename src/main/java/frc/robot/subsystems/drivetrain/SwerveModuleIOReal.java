package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModuleIOReal implements SwerveModuleIO {
  private final DrivetrainConstants.ModuleConfig config;
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder steerEncoder;

  private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final VoltageOut steerVoltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final com.ctre.phoenix6.StatusSignal<Angle> drivePosition;
  private final com.ctre.phoenix6.StatusSignal<AngularVelocity> driveVelocity;
  private final com.ctre.phoenix6.StatusSignal<Voltage> driveAppliedVolts;
  private final com.ctre.phoenix6.StatusSignal<Current> driveCurrent;
  private final com.ctre.phoenix6.StatusSignal<Angle> steerPosition;
  private final com.ctre.phoenix6.StatusSignal<AngularVelocity> steerVelocity;
  private final com.ctre.phoenix6.StatusSignal<Voltage> steerAppliedVolts;
  private final com.ctre.phoenix6.StatusSignal<Current> steerCurrent;
  private final com.ctre.phoenix6.StatusSignal<Angle> steerAbsolutePosition;

  public SwerveModuleIOReal(DrivetrainConstants.ModuleConfig config) {
    this.config = config;
    driveMotor = new TalonFX(config.driveCanId(), DrivetrainConstants.CAN_BUS_NAME);
    steerMotor = new TalonFX(config.steerCanId(), DrivetrainConstants.CAN_BUS_NAME);
    steerEncoder = new CANcoder(config.encoderCanId(), DrivetrainConstants.CAN_BUS_NAME);

    configureDriveMotor();
    configureSteerMotor();
    configureEncoder();

    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getStatorCurrent();
    steerPosition = steerMotor.getPosition();
    steerVelocity = steerMotor.getVelocity();
    steerAppliedVolts = steerMotor.getMotorVoltage();
    steerCurrent = steerMotor.getStatorCurrent();
    steerAbsolutePosition = steerEncoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        steerPosition,
        steerVelocity,
        steerAppliedVolts,
        steerCurrent,
        steerAbsolutePosition);
    driveMotor.optimizeBusUtilization();
    steerMotor.optimizeBusUtilization();
    steerEncoder.optimizeBusUtilization();
  }

  private void configureDriveMotor() {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted =
        config.driveInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveConfig.Feedback.SensorToMechanismRatio = DrivetrainConstants.DRIVE_GEAR_RATIO;
    driveConfig.CurrentLimits.StatorCurrentLimit = 35.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.Slot0.kP = 0.10;
    driveConfig.Slot0.kS = 0.6 / 12.0;
    driveConfig.Slot0.kV = 2.344 / 12.0;
    driveMotor.getConfigurator().apply(driveConfig);
    driveMotor.setPosition(0.0);
  }

  private void configureSteerMotor() {
    TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    steerConfig.MotorOutput.Inverted =
        config.steerInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    steerConfig.Feedback.FeedbackRemoteSensorID = config.encoderCanId();
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfig.Feedback.RotorToSensorRatio = DrivetrainConstants.STEER_GEAR_RATIO;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.Slot0.kP = 8.5;
    steerConfig.Slot0.kD = 0.0;
    steerMotor.getConfigurator().apply(steerConfig);
  }

  private void configureEncoder() {
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset =
        Units.degreesToRotations(-config.encoderOffsetDegrees());
    encoderConfig.MagnetSensor.SensorDirection =
        config.encoderInverted()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    steerEncoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            steerPosition,
            steerVelocity,
            steerAppliedVolts,
            steerCurrent,
            steerAbsolutePosition);

    inputs.driveConnected = status.isOK();
    inputs.drivePosition_m =
        drivePosition.getValueAsDouble() * DrivetrainConstants.WHEEL_CIRCUMFERENCE_METERS;
    inputs.driveVelocity_mps =
        driveVelocity.getValueAsDouble() * DrivetrainConstants.WHEEL_CIRCUMFERENCE_METERS;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.steerConnected = status.isOK();
    inputs.steerEncoderConnected = status.isOK();
    inputs.steerAbsolutePosition_Rot2d =
        Rotation2d.fromRotations(steerAbsolutePosition.getValueAsDouble());
    inputs.steerPosition_Rot2d = Rotation2d.fromRotations(steerPosition.getValueAsDouble());
    inputs.steerVelocity_radps = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
    inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
    inputs.steerCurrentAmps = steerCurrent.getValueAsDouble();

    inputs.odometryTimestamps_s = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositions_m = new double[] {inputs.drivePosition_m};
    inputs.odometrySteerPositions_Rot2d = new Rotation2d[] {inputs.steerPosition_Rot2d};
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveMotor.setControl(driveVoltageRequest.withOutput(volts));
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSecond) {
    double wheelRotationsPerSecond =
        velocityMetersPerSecond / DrivetrainConstants.WHEEL_CIRCUMFERENCE_METERS;
    driveMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(wheelRotationsPerSecond));
  }

  @Override
  public void setSteerOpenLoop(double volts) {
    steerMotor.setControl(steerVoltageRequest.withOutput(volts));
  }

  @Override
  public void setSteerPosition(Rotation2d angle) {
    steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    driveMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    steerMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void resetToAbsolute() {
    steerMotor.setPosition(steerAbsolutePosition.getValueAsDouble());
  }

  @Override
  public void stop() {
    driveMotor.setControl(neutralRequest);
    steerMotor.setControl(neutralRequest);
  }
}
