package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Two-stage vertical elevator used to reach the Charged Up scoring rows. */
public class Elevator extends SubsystemBase {
  public static final double MAX_HEIGHT_INCHES = 48.7;
  public static final double TOLERANCE_INCHES = 0.2;

  private static final double MAX_MANUAL_VOLTS = 10.0;
  private static final double HOLDING_VOLTS = 0.18;

  private final ElevatorIO io;
  private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();
  private final PIDController controller = new PIDController(0.12, 0.0, 0.0);
  private boolean closedLoop;
  private double targetHeightInches;
  private double manualVolts;

  /** Creates the simulation-backed elevator used on the desktop. */
  public Elevator() {
    this(new ElevatorIOSim());
  }

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.resetPosition(0.0);
    controller.setTolerance(TOLERANCE_INCHES);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (closedLoop) {
      double feedbackVolts = controller.calculate(inputs.heightInches, targetHeightInches);
      double gravityVolts = inputs.heightInches > 0.5 ? HOLDING_VOLTS : 0.0;
      io.setVoltage(MathUtil.clamp(feedbackVolts + gravityVolts, -12.0, 12.0));
    } else {
      io.setVoltage(manualVolts);
    }

    SmartDashboard.putNumber("Elevator/HeightInches", inputs.heightInches);
    SmartDashboard.putNumber("Elevator/TargetHeightInches", targetHeightInches);
    SmartDashboard.putNumber("Elevator/VelocityInchesPerSecond", inputs.velocityInchesPerSecond);
    SmartDashboard.putBoolean("Elevator/AtLowerLimit", inputs.atLowerLimit);
    SmartDashboard.putBoolean("Elevator/AtUpperLimit", inputs.atUpperLimit);
  }

  public void setHeightInches(double heightInches) {
    targetHeightInches = MathUtil.clamp(heightInches, 0.0, MAX_HEIGHT_INCHES);
    closedLoop = true;
  }

  public void setPercentOutput(double percent) {
    closedLoop = false;
    manualVolts = MathUtil.clamp(percent, -1.0, 1.0) * MAX_MANUAL_VOLTS;
  }

  public void runVoltage(double volts) {
    closedLoop = false;
    manualVolts = MathUtil.clamp(volts, -MAX_MANUAL_VOLTS, MAX_MANUAL_VOLTS);
  }

  public void holdCurrentHeight() {
    setHeightInches(inputs.heightInches);
  }

  public void stop() {
    closedLoop = false;
    manualVolts = 0.0;
    io.stop();
  }

  public void zeroHeight() {
    io.resetPosition(0.0);
    targetHeightInches = 0.0;
  }

  public boolean isAtHeight(double heightInches) {
    return Math.abs(inputs.heightInches - heightInches) <= TOLERANCE_INCHES;
  }

  public boolean isAtTarget() {
    return isAtHeight(targetHeightInches);
  }

  public boolean atLowerLimit() {
    return inputs.atLowerLimit;
  }

  public boolean atUpperLimit() {
    return inputs.atUpperLimit;
  }

  public double getHeightInches() {
    return inputs.heightInches;
  }

  public double getTargetHeightInches() {
    return targetHeightInches;
  }
}
