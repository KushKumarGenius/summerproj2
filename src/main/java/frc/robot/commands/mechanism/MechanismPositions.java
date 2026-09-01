package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState.GamePiece;
import frc.robot.RobotState.ScoringRow;
import frc.robot.commands.elevator.ElevatorSetHeight;
import frc.robot.commands.stinger.StingerSetExtension;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.stinger.Stinger;

/** Named, reusable mechanism positions from the 2023 Rober operator workflow. */
public final class MechanismPositions {
  private MechanismPositions() {}

  public static Command stow(Elevator elevator, Stinger stinger) {
    return Commands.sequence(
        new StingerSetExtension(stinger, Constants.Stinger.STOW_EXTENSION_INCHES),
        new ElevatorSetHeight(elevator, Constants.Elevator.STOW_HEIGHT_INCHES));
  }

  public static Command groundPickup(Elevator elevator, Stinger stinger) {
    return Commands.sequence(
        new ElevatorSetHeight(elevator, 5.5),
        new StingerSetExtension(stinger, Constants.Stinger.GROUND_PICKUP_EXTENSION_INCHES),
        new ElevatorSetHeight(elevator, Constants.Elevator.GROUND_PICKUP_HEIGHT_INCHES));
  }

  public static Command substationPickup(Elevator elevator, Stinger stinger) {
    return Commands.parallel(
        new ElevatorSetHeight(elevator, Constants.Elevator.SUBSTATION_HEIGHT_INCHES),
        new StingerSetExtension(stinger, Constants.Stinger.SUBSTATION_EXTENSION_INCHES));
  }

  public static Command scoringPosition(
      Elevator elevator, Stinger stinger, GamePiece gamePiece, ScoringRow row) {
    double height;
    double extension;
    switch (row) {
      case HYBRID:
        height = Constants.Elevator.LOW_HEIGHT_INCHES;
        extension = Constants.Stinger.LOW_EXTENSION_INCHES;
        break;
      case MID:
        if (gamePiece == GamePiece.CONE) {
          height = Constants.Elevator.MID_CONE_HEIGHT_INCHES;
          extension = Constants.Stinger.MID_CONE_EXTENSION_INCHES;
        } else {
          height = Constants.Elevator.MID_CUBE_HEIGHT_INCHES;
          extension = Constants.Stinger.MID_CUBE_EXTENSION_INCHES;
        }
        break;
      case HIGH:
        height =
            gamePiece == GamePiece.CONE
                ? Constants.Elevator.MAX_HEIGHT_INCHES
                : Constants.Elevator.HIGH_CUBE_HEIGHT_INCHES;
        extension = Constants.Stinger.HIGH_EXTENSION_INCHES;
        break;
      default:
        throw new IllegalStateException("Unhandled scoring row: " + row);
    }

    return Commands.sequence(
        new StingerSetExtension(stinger, Constants.Stinger.STOW_EXTENSION_INCHES),
        new ElevatorSetHeight(elevator, height),
        new StingerSetExtension(stinger, extension));
  }
}
