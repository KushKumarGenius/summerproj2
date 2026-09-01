package frc.robot.devices.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public class Motor {
  public enum ControlType {
    VOLTAGE,
    OPEN_LOOP,
    VELOCITY,
    POSITION_VOLTAGE,
    MOTION_MAGIC,
    MOTION_MAGIC_VELOCITY,
    STOP
  }

  private final String name;
  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
  private ControlType controlType = ControlType.STOP;
  private double setpoint;

  public Motor(String name, MotorConfig config) {
    this.name = name;
    io = RobotBase.isReal() ? new MotorIOReal(config) : new MotorIOSim(config);
  }

  public Motor(String name, MotorConfig config, MotorIO io) {
    this.name = name;
    this.io = io;
  }

  public void readInputs() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public void setVoltage(double volts) {
    setOutput(ControlType.VOLTAGE, MathUtil.clamp(volts, -12.0, 12.0));
    io.setVoltage(setpoint);
  }

  public void setOpenLoop(double dutyCycle) {
    setOutput(ControlType.OPEN_LOOP, MathUtil.clamp(dutyCycle, -1.0, 1.0));
    io.setOpenLoop(setpoint);
  }

  public void setVelocity(double velocity) {
    setOutput(ControlType.VELOCITY, velocity);
    io.setVelocity(setpoint);
  }

  public void setPositionVoltage(double position) {
    setOutput(ControlType.POSITION_VOLTAGE, position);
    io.setPositionVoltage(setpoint);
  }

  public void setMotionMagic(double position) {
    setOutput(ControlType.MOTION_MAGIC, position);
    io.setMotionMagic(setpoint);
  }

  public void setMotionMagicVelocity(double velocity) {
    setOutput(ControlType.MOTION_MAGIC_VELOCITY, velocity);
    io.setMotionMagicVelocity(setpoint);
  }

  public void stop() {
    setOutput(ControlType.STOP, 0.0);
    io.stop();
  }

  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }

  public void zeroPosition(double position) {
    io.zeroPosition(position);
  }

  private void setOutput(ControlType controlType, double setpoint) {
    this.controlType = controlType;
    this.setpoint = setpoint;
    Logger.recordOutput(name + "/ControlType", controlType);
    Logger.recordOutput(name + "/Setpoint", setpoint);
  }

  public double getPosition() {
    return inputs.position;
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  public double getAcceleration() {
    return inputs.acceleration;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  public double getCurrent() {
    return inputs.statorCurrent;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public ControlType getControlType() {
    return controlType;
  }

  public double getSetpoint() {
    return setpoint;
  }
}
