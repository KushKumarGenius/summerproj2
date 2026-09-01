package frc.robot.devices.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class MotorIOReal implements MotorIO {
  private final MotorConfig config;
  private final TalonFX motor;
  private final TalonFX follower;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temperature;
  private final VoltageOut voltageRequest;
  private final DutyCycleOut dutyCycleRequest;
  private final VelocityVoltage velocityRequest;
  private final PositionVoltage positionRequest;
  private final MotionMagicVoltage motionMagicRequest;
  private final MotionMagicVelocityVoltage motionMagicVelocityRequest;
  private final NeutralOut neutralRequest = new NeutralOut();

  public MotorIOReal(MotorConfig config) {
    this.config = config;
    motor = new TalonFX(config.canId, config.canBus);
    follower = config.followerId == null ? null : new TalonFX(config.followerId, config.canBus);
    voltageRequest = new VoltageOut(0).withEnableFOC(config.foc);
    dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(config.foc);
    velocityRequest = new VelocityVoltage(0).withEnableFOC(config.foc);
    positionRequest = new PositionVoltage(0).withEnableFOC(config.foc);
    motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(config.foc);
    motionMagicVelocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(config.foc);
    configure(motor);
    if (follower != null) {
      configure(follower);
      follower.setControl(new Follower(motor.getDeviceID(), config.followerAlignment));
    }

    position = motor.getPosition();
    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    statorCurrent = motor.getStatorCurrent();
    temperature = motor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, acceleration, appliedVolts, supplyCurrent, statorCurrent, temperature);
    motor.optimizeBusUtilization();
    if (follower != null) {
      follower.optimizeBusUtilization();
    }
  }

  private void configure(TalonFX talon) {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.Feedback.SensorToMechanismRatio = config.sensorToMechanismRatio;
    motorConfig.MotorOutput.Inverted =
        config.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode =
        config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = config.motionMagicCruiseVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = config.motionMagicAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk = config.motionMagicJerk;
    motorConfig.Slot0.GravityType =
        config.gravity == MotorConfig.GravityType.ARM
            ? GravityTypeValue.Arm_Cosine
            : GravityTypeValue.Elevator_Static;
    motorConfig.Slot0.kS = config.kS;
    motorConfig.Slot0.kV = config.kV;
    motorConfig.Slot0.kA = config.kA;
    motorConfig.Slot0.kG = config.kG;
    motorConfig.Slot0.kP = config.kP;
    motorConfig.Slot0.kI = config.kI;
    motorConfig.Slot0.kD = config.kD;
    talon.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            position, velocity, acceleration, appliedVolts, supplyCurrent, statorCurrent, temperature);
    inputs.connected = status.isOK();
    inputs.position = position.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
    inputs.acceleration = acceleration.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
    inputs.statorCurrent = statorCurrent.getValueAsDouble();
    inputs.temperatureCelsius = temperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setOpenLoop(double dutyCycle) {
    motor.setControl(dutyCycleRequest.withOutput(dutyCycle));
  }

  @Override
  public void setVelocity(double velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setPositionVoltage(double position) {
    motor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void setMotionMagic(double position) {
    motor.setControl(motionMagicRequest.withPosition(position));
  }

  @Override
  public void setMotionMagicVelocity(double velocity) {
    motor.setControl(motionMagicVelocityRequest.withVelocity(velocity));
  }

  @Override
  public void stop() {
    motor.setControl(neutralRequest);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void zeroPosition(double position) {
    motor.setPosition(position);
  }
}
