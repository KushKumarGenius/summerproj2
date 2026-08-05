package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeIn extends Command {
  // The command holds the subsystem it controls.
  private final Intake intake;

  public IntakeIn(Intake intake) {
    this.intake = intake;

    // Prevent another command from controllzing the intake at the same time.
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    // Runs once when R is first pressed.
    intake.intake();
  }

  @Override
  public void execute() {
    // Runs repeatedly while R is held.
    intake.intake();
  }

  @Override
  public void end(boolean interrupted) {
    // Runs when R is released or the command is interrupted.
    // This is why the intake stops after releasing R.
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    // false keeps the command active until R is released.
    return false;
  }
}