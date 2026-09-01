package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotState.GamePiece;
import frc.robot.RobotState.ScoringRow;
import frc.robot.commands.intake.IntakeGrabCone;
import frc.robot.commands.intake.IntakeGrabCube;
import frc.robot.commands.intake.IntakeScoreCone;
import frc.robot.commands.intake.IntakeScoreCube;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.stinger.Stinger;

public final class MechanismActions {
  private MechanismActions() {}

  public static Command grabCurrentGamePiece(Intake intake) {
    return RobotState.getInstance().getDesiredGamePiece() == GamePiece.CONE
        ? new IntakeGrabCone(intake)
        : new IntakeGrabCube(intake);
  }

  public static Command scoreCurrentGamePiece(
      Elevator elevator, Stinger stinger, Intake intake) {
    RobotState state = RobotState.getInstance();
    return scoreGamePiece(
        elevator, stinger, intake, state.getDesiredGamePiece(), state.getDesiredScoringRow());
  }

  public static Command scoreGamePiece(
      Elevator elevator,
      Stinger stinger,
      Intake intake,
      GamePiece gamePiece,
      ScoringRow row) {

    Command holdWhilePositioning =
        gamePiece == GamePiece.CONE
            ? new IntakeGrabCone(intake, 0.8).withTimeout(0.5)
            : new IntakeGrabCube(intake, 0.35).withTimeout(0.5);
    Command eject =
        gamePiece == GamePiece.CONE
            ? new IntakeScoreCone(intake).withTimeout(0.3)
            : new IntakeScoreCube(intake).withTimeout(0.3);

    return Commands.sequence(
        MechanismPositions.scoringPosition(elevator, stinger, gamePiece, row)
            .deadlineWith(holdWhilePositioning),
        eject,
        MechanismPositions.stow(elevator, stinger));
  }

}
