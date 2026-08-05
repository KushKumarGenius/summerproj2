package frc.robot.subsystems.intake;

public final class IntakeConstants {
  private IntakeConstants() {}

  // Temporary software values for the simulation version.
  // Later, these can be replaced with values measured from the real 2930 intake.

  // Positive voltage spins the intake in the direction that pulls a game piece in.
  public static final double INTAKE_VOLTS = 10.0;

  // A small positive voltage can keep a game piece held without pulling it in quickly.
  public static final double HOLD_VOLTS = 0.7;

  // Negative voltage spins the intake in reverse to eject a game piece.
  public static final double EJECT_VOLTS = -3.0;
}