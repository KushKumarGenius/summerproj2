package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Operator-selected state shared by mechanism commands. */
public class RobotState {
  private static final RobotState INSTANCE = new RobotState();

  public enum GamePiece {
    CONE,
    CUBE
  }

  public enum ScoringRow {
    HYBRID,
    MID,
    HIGH
  }

  private GamePiece desiredGamePiece = GamePiece.CONE;
  private ScoringRow desiredScoringRow = ScoringRow.HIGH;

  public static RobotState getInstance() {
    return INSTANCE;
  }

  private RobotState() {
    SmartDashboard.putString("RobotState/GamePiece", desiredGamePiece.name());
    SmartDashboard.putString("RobotState/ScoringRow", desiredScoringRow.name());
  }

  public void setDesiredGamePiece(GamePiece gamePiece) {
    desiredGamePiece = gamePiece;
    SmartDashboard.putString("RobotState/GamePiece", gamePiece.name());
  }

  public GamePiece getDesiredGamePiece() {
    return desiredGamePiece;
  }

  public void setDesiredScoringRow(ScoringRow row) {
    desiredScoringRow = row;
    SmartDashboard.putString("RobotState/ScoringRow", row.name());
  }

  public ScoringRow getDesiredScoringRow() {
    return desiredScoringRow;
  }
}
