package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeScoreCone extends Command {
  private final Intake intake;
  private final double percent;

  public IntakeScoreCone(Intake intake) {
    this(intake, 0.8);
  }

  public IntakeScoreCone(Intake intake, double percent) {
    this.intake = intake;
    this.percent = percent;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.outtakeCone(percent);
  }

  @Override
  public void execute() {
    intake.outtakeCone(percent);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
