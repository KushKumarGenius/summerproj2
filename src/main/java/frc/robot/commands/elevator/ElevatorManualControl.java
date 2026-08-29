package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

/** Manual elevator control for the operator's joystick. */
public class ElevatorManualControl extends Command {
  private final Elevator elevator;
  private final DoubleSupplier axis;

  public ElevatorManualControl(Elevator elevator, DoubleSupplier axis) {
    this.elevator = elevator;
    this.axis = axis;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    elevator.setPercentOutput(MathUtil.applyDeadband(axis.getAsDouble(), 0.1) * 0.35);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
