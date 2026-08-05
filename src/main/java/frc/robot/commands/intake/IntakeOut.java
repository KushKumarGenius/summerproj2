package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeOut extends Command {
  // Controls the intake while the eject button is held.
  private final Intake intake;

  public IntakeOut(Intake intake) {
    this.intake = intake;

    // Reserve the intake so another intake command cannot run simultaneously.
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    // Runs once when E is first pressed.
    intake.eject();
  }

  @Override
  public void execute() {
    // Runs repeatedly while E is held.
    intake.eject();
  }

  @Override
  public void end(boolean interrupted) {
    // Stops the motor when E is released or the command is interrupted.
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    // Stay active until the button binding cancels this command on release.
    return false;
  }
}