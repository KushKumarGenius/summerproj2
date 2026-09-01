package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeScoreCube extends Command {
  private final Intake intake;
  private final double percent;

  public IntakeScoreCube(Intake intake) {
    this(intake, 1.0);
  }

  public IntakeScoreCube(Intake intake, double percent) {
    this.intake = intake;
    this.percent = percent;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.outtakeCube(percent);
  }

  @Override
  public void execute() {
    intake.outtakeCube(percent);
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
