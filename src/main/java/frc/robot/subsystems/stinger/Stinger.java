package frc.robot.subsystems.stinger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Horizontal telescoping mechanism that moves the intake to game pieces and nodes. */
public class Stinger extends SubsystemBase {
  public static final double MAX_EXTENSION_INCHES = 25.0;
  public static final double TOLERANCE_INCHES = 0.25;

  private final StingerIO io;
  private final StingerIO.StingerIOInputs inputs = new StingerIO.StingerIOInputs();
  private final PIDController controller = new PIDController(0.12, 0.0, 0.0);
  private boolean closedLoop;
  private double targetExtensionInches;
  private double manualVolts;

  /** Creates the simulation-backed stinger used on the desktop. */
  public Stinger() {
    this(new StingerIOSim());
  }

  public Stinger(StingerIO io) {
    this.io = io;
    io.resetPosition(0.0);
    controller.setTolerance(TOLERANCE_INCHES);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (closedLoop) {
      io.setVoltage(MathUtil.clamp(controller.calculate(inputs.extensionInches, targetExtensionInches), -12.0, 12.0));
    } else {
      io.setVoltage(manualVolts);
    }

    SmartDashboard.putNumber("Stinger/ExtensionInches", inputs.extensionInches);
    SmartDashboard.putNumber("Stinger/TargetExtensionInches", targetExtensionInches);
    SmartDashboard.putNumber("Stinger/VelocityInchesPerSecond", inputs.velocityInchesPerSecond);
    SmartDashboard.putBoolean("Stinger/AtRetractedLimit", inputs.atRetractedLimit);
    SmartDashboard.putBoolean("Stinger/AtExtendedLimit", inputs.atExtendedLimit);
  }

  public void setExtensionInches(double extensionInches) {
    targetExtensionInches = MathUtil.clamp(extensionInches, 0.0, MAX_EXTENSION_INCHES);
    closedLoop = true;
  }

  public void setPercentOutput(double percent) {
    closedLoop = false;
    manualVolts = MathUtil.clamp(percent, -1.0, 1.0) * 10.0;
  }

  public void stop() {
    closedLoop = false;
    manualVolts = 0.0;
    io.stop();
  }

  public void zeroExtension() {
    io.resetPosition(0.0);
    targetExtensionInches = 0.0;
  }

  public boolean isAtExtension(double extensionInches) {
    return Math.abs(inputs.extensionInches - extensionInches) <= TOLERANCE_INCHES;
  }

  public boolean isAtTarget() {
    return isAtExtension(targetExtensionInches);
  }

  public boolean atRetractedLimit() {
    return inputs.atRetractedLimit;
  }

  public boolean atExtendedLimit() {
    return inputs.atExtendedLimit;
  }

  public double getExtensionInches() {
    return inputs.extensionInches;
  }

  public double getTargetExtensionInches() {
    return targetExtensionInches;
  }
}
