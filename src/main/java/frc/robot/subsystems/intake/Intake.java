package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public enum State {
    IDLE,
    INTAKING,
    HOLDING,
    EJECTING
  }

  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private final Motor motor;
  private final IntakeIO legacyIO;
  private final IntakeIO.IntakeIOInputs legacyInputs;
  private State state = State.IDLE;
  private double requestedPercent;
  private double appliedVolts;
  private double velocityRPM;
  private double currentAmps;
  private double pieceTimer;
  private boolean hasPiece;

  public Intake() {
    motor =
        new Motor(
            "Intake/Motor",
            new MotorConfig(IntakeConstants.MOTOR_ID)
                .withInverted(false)
                .withBrake(true)
                .withSupplyCurrentLimit(50.0)
                .withSimulation(DCMotor.getFalcon500(1), 1.0, 0.001));
    legacyIO = null;
    legacyInputs = null;
  }

  public Intake(IntakeIO io) {
    motor = null;
    legacyIO = io;
    legacyInputs = new IntakeIO.IntakeIOInputs();
  }

  @Override
  public void periodic() {
    if (motor != null) {
      updateMotorInputs();
      applyMotorOutput();
    } else {
      legacyIO.updateInputs(legacyInputs);
      applyLegacyOutput();
      appliedVolts = legacyInputs.appliedVolts;
      velocityRPM = legacyInputs.velocityRPM;
      currentAmps = legacyInputs.currentAmps;
      hasPiece = legacyInputs.hasPiece;
    }

    Logger.recordOutput("Intake/State", state.name());
    Logger.recordOutput("Intake/AppliedVolts", appliedVolts);
    Logger.recordOutput("Intake/VelocityRPM", velocityRPM);
    Logger.recordOutput("Intake/CurrentAmps", currentAmps);
    Logger.recordOutput("Intake/HasPiece", hasPiece);
  }

  private void updateMotorInputs() {
    motor.readInputs();
    appliedVolts = motor.getAppliedVolts();
    velocityRPM = motor.getVelocity() * 60.0;
    currentAmps = motor.getCurrent();
  }

  private void applyMotorOutput() {
    switch (state) {
      case INTAKING:
        motor.setVoltage(requestedPercent * IntakeConstants.MAX_VOLTAGE);
        if (Math.abs(requestedPercent) > 0.4) {
          pieceTimer += LOOP_PERIOD_SECONDS;
          if (pieceTimer >= 0.75) {
            hasPiece = true;
          }
        }
        break;
      case HOLDING:
        motor.setVoltage(requestedPercent * IntakeConstants.MAX_VOLTAGE);
        break;
      case EJECTING:
        motor.setVoltage(requestedPercent * IntakeConstants.MAX_VOLTAGE);
        if (Math.abs(requestedPercent) > 0.2) {
          pieceTimer = 0.0;
          hasPiece = false;
        }
        break;
      case IDLE:
        motor.stop();
        pieceTimer = 0.0;
        break;
    }
  }

  private void applyLegacyOutput() {
    switch (state) {
      case INTAKING:
      case HOLDING:
      case EJECTING:
        legacyIO.setVoltage(requestedPercent * IntakeConstants.MAX_VOLTAGE);
        break;
      case IDLE:
        legacyIO.stop();
        break;
    }
  }

  public void intakeCone(double percent) {
    setState(State.INTAKING, percent * IntakeConstants.CONE_DIRECTION);
  }

  public void intake() {
    runPercent(1.0);
  }

  public void intakeCube(double percent) {
    setState(State.INTAKING, percent * IntakeConstants.CUBE_DIRECTION);
  }

  public void outtakeCone(double percent) {
    setState(State.EJECTING, percent * -IntakeConstants.CONE_DIRECTION);
  }

  public void eject() {
    setState(State.EJECTING, -1.0);
  }

  public void outtakeCube(double percent) {
    setState(State.EJECTING, percent * -IntakeConstants.CUBE_DIRECTION);
  }

  public void hold(double percent) {
    setState(State.HOLDING, percent);
  }

  public void runPercent(double percent) {
    setState(State.INTAKING, percent);
  }

  public void stop() {
    requestedPercent = 0.0;
    state = State.IDLE;
    if (motor != null) {
      motor.stop();
    } else {
      legacyIO.stop();
    }
  }

  public boolean isStalled() {
    return Math.abs(velocityRPM) <= IntakeConstants.STALL_VELOCITY_RPM
        && currentAmps >= IntakeConstants.STALL_CURRENT_AMPS;
  }

  public boolean hasPiece() {
    return hasPiece;
  }

  public double getVelocityRPM() {
    return velocityRPM;
  }

  public double getAppliedVolts() {
    return appliedVolts;
  }

  public State getState() {
    return state;
  }

  private void setState(State nextState, double percent) {
    state = nextState;
    requestedPercent = MathUtil.clamp(percent, -1.0, 1.0);
  }
}
