package frc.robot.commands.stinger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.stinger.Stinger;
import java.util.function.DoubleSupplier;

public class StingerManualControl extends Command {
  private static final double MIN_CLEARANCE_HEIGHT_INCHES = 3.0;

  private final Stinger stinger;
  private final Elevator elevator;
  private final DoubleSupplier axis;

  public StingerManualControl(Stinger stinger, Elevator elevator, DoubleSupplier axis) {
    this.stinger = stinger;
    this.elevator = elevator;
    this.axis = axis;
    addRequirements(stinger);
  }

  @Override
  public void execute() {
    double percent = MathUtil.applyDeadband(axis.getAsDouble(), 0.1) * 0.35;
    if (elevator.getHeightInches() < MIN_CLEARANCE_HEIGHT_INCHES && percent > 0.0) {
      percent = 0.0;
    }
    stinger.setPercentOutput(percent);
  }

  @Override
  public void end(boolean interrupted) {
    stinger.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
