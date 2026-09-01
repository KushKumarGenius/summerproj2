package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState.GamePiece;
import frc.robot.RobotState.ScoringRow;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.stinger.Stinger;

public final class SimpleAutos {
  private SimpleAutos() {}

  public static Command doNothing() {
    return Commands.none();
  }

  public static Command scoreCone(
      Drivetrain drivetrain, Elevator elevator, Stinger stinger, Intake intake) {
    return MechanismActions.scoreGamePiece(
        elevator, stinger, intake, GamePiece.CONE, ScoringRow.HIGH);
  }

  public static Command scoreCube(
      Drivetrain drivetrain, Elevator elevator, Stinger stinger, Intake intake) {
    return Commands.sequence(
        MechanismActions.scoreGamePiece(
            elevator, stinger, intake, GamePiece.CUBE, ScoringRow.HIGH));
  }

  public static Command scoreAndTaxi(
      Drivetrain drivetrain, Elevator elevator, Stinger stinger, Intake intake) {
    return Commands.sequence(
        scoreCone(drivetrain, elevator, stinger, intake),
        new DriveToPose(drivetrain, new Pose2d(2.0, 0.0, new Rotation2d())));
  }
}
