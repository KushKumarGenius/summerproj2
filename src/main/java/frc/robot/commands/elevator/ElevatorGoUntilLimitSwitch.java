package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorGoUntilLimitSwitch extends Command {
  private final Elevator elevator;
  private final double percent;

  public ElevatorGoUntilLimitSwitch(Elevator elevator, double percent) {
    this.elevator = elevator;
    this.percent = Math.abs(percent);
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setPercentOutput(-percent);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.zeroHeight();
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return elevator.atLowerLimit();
  }
}
