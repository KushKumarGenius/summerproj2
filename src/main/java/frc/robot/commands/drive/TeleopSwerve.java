package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;

/** Maps the driver's Xbox controller sticks to field-relative swerve velocity. */
public class TeleopSwerve extends Command {
  private final Drivetrain drivetrain;
  private final DoubleSupplier forward;
  private final DoubleSupplier strafe;
  private final DoubleSupplier rotation;

  public TeleopSwerve(
      Drivetrain drivetrain,
      DoubleSupplier forward,
      DoubleSupplier strafe,
      DoubleSupplier rotation) {
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.strafe = strafe;
    this.rotation = rotation;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double x = -squareWithDeadband(forward.getAsDouble());
    double y = -squareWithDeadband(strafe.getAsDouble());
    double omega = -squareWithDeadband(rotation.getAsDouble());
    drivetrain.drive(
        x * 4.5,
        y * 4.5,
        omega * drivetrain.getMaxAngularSpeed());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static double squareWithDeadband(double value) {
    double adjusted = MathUtil.applyDeadband(value, Constants.OperatorConstants.AXIS_DEADBAND);
    return Math.copySign(adjusted * adjusted, adjusted);
  }
}
