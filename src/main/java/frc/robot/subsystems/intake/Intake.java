package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Controls the roller that collects and scores cones and cubes. */
public class Intake extends SubsystemBase {
  public enum State {
    IDLE,
    INTAKING,
    HOLDING,
    EJECTING
  }

  private static final double MAX_VOLTAGE = 11.0;
  private static final double CONE_DIRECTION = 1.0;
  private static final double CUBE_DIRECTION = -1.0;
  private static final double STALL_VELOCITY_RPM = 250.0;
  private static final double STALL_CURRENT_AMPS = 18.0;

  private final IntakeIO io;
  private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
  private State state = State.IDLE;
  private double requestedPercent;

  /** Creates the simulation-backed intake used on the desktop. */
  public Intake() {
    this(new IntakeIOSim());
  }

  /** Creates an intake around a real or simulated IO implementation. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (state) {
      case INTAKING:
      case HOLDING:
      case EJECTING:
        io.setVoltage(requestedPercent * MAX_VOLTAGE);
        break;
      case IDLE:
        io.stop();
        break;
    }

    SmartDashboard.putString("Intake/State", state.name());
    SmartDashboard.putNumber("Intake/AppliedVolts", inputs.appliedVolts);
    SmartDashboard.putNumber("Intake/VelocityRPM", inputs.velocityRPM);
    SmartDashboard.putNumber("Intake/CurrentAmps", inputs.currentAmps);
    SmartDashboard.putBoolean("Intake/HasPiece", inputs.hasPiece);
  }

  public void intakeCone(double percent) {
    setState(State.INTAKING, percent * CONE_DIRECTION);
  }

  /** Generic inward command kept for simple driver bindings. */
  public void intake() {
    runPercent(1.0);
  }

  public void intakeCube(double percent) {
    setState(State.INTAKING, percent * CUBE_DIRECTION);
  }

  public void outtakeCone(double percent) {
    setState(State.EJECTING, percent * -CONE_DIRECTION);
  }

  /** Generic outward command kept for simple driver bindings. */
  public void eject() {
    setState(State.EJECTING, -1.0);
  }

  public void outtakeCube(double percent) {
    setState(State.EJECTING, percent * -CUBE_DIRECTION);
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
    io.stop();
  }

  public boolean isStalled() {
    return Math.abs(inputs.velocityRPM) <= STALL_VELOCITY_RPM
        && inputs.currentAmps >= STALL_CURRENT_AMPS;
  }

  public boolean hasPiece() {
    return inputs.hasPiece;
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  public State getState() {
    return state;
  }

  private void setState(State nextState, double percent) {
    state = nextState;
    requestedPercent = MathUtil.clamp(percent, -1.0, 1.0);
  }
}
