package frc.robot.devices.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorIOSim implements MotorIO {
  private enum ControlMode {
    VOLTAGE,
    OPEN_LOOP,
    POSITION,
    VELOCITY,
    NEUTRAL
  }

  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final MotorConfig config;
  private final DCMotorSim motorSim;
  private final PIDController positionController;
  private final PIDController velocityController;
  private ControlMode controlMode = ControlMode.NEUTRAL;
  private double setpoint;
  private double appliedVolts;

  public MotorIOSim(MotorConfig config) {
    this.config = config;
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                config.simMotor, config.simMomentOfInertia, config.simGearing),
            config.simMotor);
    positionController = new PIDController(config.kP, config.kI, config.kD);
    velocityController = new PIDController(0.5, 0.0, 0.0);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    double position = motorSim.getAngularPositionRotations();
    double velocity = motorSim.getAngularVelocityRPM() / 60.0;
    appliedVolts = MathUtil.clamp(calculateVoltage(position, velocity), -12.0, 12.0);
    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(LOOP_PERIOD_SECONDS);

    inputs.connected = true;
    inputs.position = motorSim.getAngularPositionRotations();
    inputs.velocity = motorSim.getAngularVelocityRPM() / 60.0;
    inputs.acceleration = motorSim.getAngularAccelerationRadPerSecSq() / (2.0 * Math.PI);
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrent = Math.abs(motorSim.getCurrentDrawAmps());
    inputs.statorCurrent = Math.abs(motorSim.getCurrentDrawAmps());
    inputs.temperatureCelsius = 25.0;
  }

  private double calculateVoltage(double position, double velocity) {
    return switch (controlMode) {
      case VOLTAGE -> setpoint;
      case OPEN_LOOP -> setpoint * 12.0;
      case POSITION ->
          config.kG
              + positionController.calculate(position, setpoint)
              + Math.copySign(config.kS, setpoint - position);
      case VELOCITY ->
          config.kS * Math.signum(setpoint)
              + config.kV * setpoint
              + velocityController.calculate(velocity, setpoint);
      case NEUTRAL -> 0.0;
    };
  }

  @Override
  public void setVoltage(double volts) {
    controlMode = ControlMode.VOLTAGE;
    setpoint = volts;
  }

  @Override
  public void setOpenLoop(double dutyCycle) {
    controlMode = ControlMode.OPEN_LOOP;
    setpoint = dutyCycle;
  }

  @Override
  public void setVelocity(double velocity) {
    controlMode = ControlMode.VELOCITY;
    setpoint = velocity;
  }

  @Override
  public void setPositionVoltage(double position) {
    controlMode = ControlMode.POSITION;
    setpoint = position;
  }

  @Override
  public void setMotionMagic(double position) {
    controlMode = ControlMode.POSITION;
    setpoint = position;
  }

  @Override
  public void setMotionMagicVelocity(double velocity) {
    controlMode = ControlMode.VELOCITY;
    setpoint = velocity;
  }

  @Override
  public void stop() {
    controlMode = ControlMode.NEUTRAL;
    setpoint = 0.0;
  }

  @Override
  public void zeroPosition(double position) {
    motorSim.setState(position * 2.0 * Math.PI, 0.0);
  }
}
