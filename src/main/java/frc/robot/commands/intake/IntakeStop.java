package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeStop extends Command {
  // A one-time stop command. R and E already stop themselves in end(),
  // so this class is available for future button bindings or autonomous routines.
  private final Intake intake;

  public IntakeStop(Intake intake) {
    this.intake = intake;

    // The command needs control of the intake while it runs.
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    // Set the subsystem to IDLE immediately.
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    // Finish immediately after initialize() stops the intake.
    return true;
  }
}