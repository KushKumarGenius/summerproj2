package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeGrabCone extends Command {
  private final Intake intake;
  private final double percent;

  public IntakeGrabCone(Intake intake) {
    this(intake, 1.0);
  }

  public IntakeGrabCone(Intake intake, double percent) {
    this.intake = intake;
    this.percent = percent;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.intakeCone(percent);
  }

  @Override
  public void execute() {
    intake.intakeCone(percent);
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
