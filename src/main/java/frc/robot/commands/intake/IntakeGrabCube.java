package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeGrabCube extends Command {
  private final Intake intake;
  private final double percent;

  public IntakeGrabCube(Intake intake) {
    this(intake, 0.7);
  }

  public IntakeGrabCube(Intake intake, double percent) {
    this.intake = intake;
    this.percent = percent;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.intakeCube(percent);
  }

  @Override
  public void execute() {
    intake.intakeCube(percent);
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
