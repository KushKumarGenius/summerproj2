package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // Commands select one of these states. periodic() turns the selected state
  // into the actual motor output.
  public enum State {
    IDLE,
    INTAKING,
    HOLDING,
    EJECTING
  }

  // The subsystem uses this interface instead of talking directly to hardware.
  // That lets us use IntakeIOSim now and a real motor implementation later.
  private final IntakeIO io;

  // Updated by the IO layer every scheduler cycle.
  private final IntakeIO.IntakeIOInputs inputs =
      new IntakeIO.IntakeIOInputs();

  // The intake starts stopped for safety.
  private State state = State.IDLE;

  // RobotContainer uses this constructor for simulation.
  public Intake() {
    this(new IntakeIOSim());
  }

  // Later, a real IntakeIO object can be passed into this constructor.
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Read the latest motor and sensor information first.
    io.updateInputs(inputs);

    // Convert the current state into a motor action.
    // periodic() runs repeatedly while the robot program is running.
    switch (state) {
      case IDLE:
        // Nothing is asking the intake to run, so stop the motor.
        io.stop();
        break;

      case INTAKING:
        // Pull a game piece into the robot.
        io.setVoltage(IntakeConstants.INTAKE_VOLTS);
        break;

      case HOLDING:
        // Apply a small voltage to help keep a game piece in place.
        io.setVoltage(IntakeConstants.HOLD_VOLTS);
        break;

      case EJECTING:
        // Spin backward to send a game piece out of the robot.
        io.setVoltage(IntakeConstants.EJECT_VOLTS);
        break;
    }

    // These values appear in the simulation dashboard for debugging.
    SmartDashboard.putString("Intake/State", state.name());
    SmartDashboard.putNumber(
        "Intake/AppliedVolts",
        inputs.appliedVolts);
    SmartDashboard.putNumber(
        "Intake/VelocityRPM",
        inputs.velocityRPM);
  }

  // These methods select a state. periodic() performs the actual motor output.
  
  public void intake() {
    state = State.INTAKING;
  }

  public void hold() {
    state = State.HOLDING;
  }

  public void eject() {
    state = State.EJECTING;
  }

  public void stop() {
    state = State.IDLE;
  }

  // Helpful for commands, tests, and debugging.
  public State getState() {
    return state;
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  public boolean hasPiece() {
    return inputs.hasPiece;
  }
}