package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;

public class LedSetColorNoEnd extends Command {
  private final LED leds;
  private final LED.Color color;

  public LedSetColorNoEnd(LED leds, LED.Color color) {
    this.leds = leds;
    this.color = color;
    addRequirements(leds);
  }

  @Override
  public void initialize() {
    leds.setColor(color);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
