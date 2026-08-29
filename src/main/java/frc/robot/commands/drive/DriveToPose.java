package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

/** Drives to a field pose using simple x/y/heading feedback. */
public class DriveToPose extends Command {
  private final Drivetrain drivetrain;
  private final Pose2d target;
  private final PIDController xController = new PIDController(2.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(2.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(3.0, 0.0, 0.0);

  public DriveToPose(Drivetrain drivetrain, Pose2d target) {
    this.drivetrain = drivetrain;
    this.target = target;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    Pose2d current = drivetrain.getPose();
    double xVelocity = MathUtil.clamp(xController.calculate(current.getX(), target.getX()), -2.0, 2.0);
    double yVelocity = MathUtil.clamp(yController.calculate(current.getY(), target.getY()), -2.0, 2.0);
    double thetaVelocity =
        MathUtil.clamp(
            thetaController.calculate(current.getRotation().getRadians(), target.getRotation().getRadians()),
            -drivetrain.getMaxAngularSpeed(),
            drivetrain.getMaxAngularSpeed());
    drivetrain.drive(xVelocity, yVelocity, thetaVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    Pose2d current = drivetrain.getPose();
    return current.getTranslation().getDistance(target.getTranslation()) < 0.08
        && Math.abs(current.getRotation().minus(target.getRotation()).getRadians()) < 0.08;
  }
}
