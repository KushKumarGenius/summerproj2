package frc.robot.commands.stinger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.stinger.Stinger;

/** Moves the stinger to a target extension and leaves position hold active. */
public class StingerSetExtension extends Command {
  private final Stinger stinger;
  private final double targetExtensionInches;

  public StingerSetExtension(Stinger stinger, double targetExtensionInches) {
    this.stinger = stinger;
    this.targetExtensionInches = targetExtensionInches;
    addRequirements(stinger);
  }

  @Override
  public void initialize() {
    stinger.setExtensionInches(targetExtensionInches);
  }

  @Override
  public boolean isFinished() {
    return stinger.isAtExtension(targetExtensionInches);
  }
}
