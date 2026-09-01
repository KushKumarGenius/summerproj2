package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  public enum Color {
    RED(0.61),
    GREEN(0.77),
    BLUE(0.87),
    YELLOW(0.69),
    VIOLET(0.91),
    ORANGE(0.65),
    WHITE(0.93),
    BLACK(0.99),
    RED_STROBE(-0.11),
    BLUE_STROBE(-0.09),
    YELLOW_STROBE(-0.07),
    WHITE_STROBE(-0.05),
    RAINBOW(-0.99),
    NOTHING(0.0);

    private final double pattern;

    Color(double pattern) {
      this.pattern = pattern;
    }
  }

  private final LedIO io;
  private Color currentColor = Color.NOTHING;

  public LED() {
    this(new LedIOSim());
  }

  public LED(LedIO io) {
    this.io = io;
    setColor(Color.NOTHING);
  }

  public void setColor(Color color) {
    currentColor = color;
    io.setPattern(color.pattern);
    SmartDashboard.putString("LED/Color", color.name());
  }

  public Color getColor() {
    return currentColor;
  }
}
