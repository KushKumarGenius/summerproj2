package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorSetHeight extends Command {
  private final Elevator elevator;
  private final double targetHeightInches;

  public ElevatorSetHeight(Elevator elevator, double targetHeightInches) {
    this.elevator = elevator;
    this.targetHeightInches = targetHeightInches;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setHeightInches(targetHeightInches);
  }

  @Override
  public boolean isFinished() {
    return elevator.isAtHeight(targetHeightInches);
  }
}
