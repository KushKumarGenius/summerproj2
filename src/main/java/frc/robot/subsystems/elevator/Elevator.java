package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.halleffect.HallEffect;
import frc.robot.devices.halleffect.HallEffectConfig;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public static final double MAX_HEIGHT_INCHES = ElevatorConstants.MAX_HEIGHT_INCHES;
  public static final double TOLERANCE_INCHES = ElevatorConstants.TOLERANCE_INCHES;

  private final Motor motor;
  private final HallEffect lowerLimit;
  private boolean closedLoop;
  private double heightInches;
  private double velocityInchesPerSecond;
  private double targetHeightInches;
  private double manualVolts;

  public Elevator() {
    motor =
        new Motor(
            "Elevator/Motor",
            new MotorConfig(ElevatorConstants.MOTOR_ID)
                .withFollower(
                    ElevatorConstants.FOLLOWER_ID, ElevatorConstants.FOLLOWER_ALIGNMENT)
                .withInverted(true)
                .withBrake(true)
                .withSupplyCurrentLimit(10.0)
                .withFeedforward(0.0, 0.0, 0.0, ElevatorConstants.HOLDING_VOLTS)
                .withPid(0.12, 0.0, 0.0, MotorConfig.GravityType.ELEVATOR)
                .withSimulation(
                    ElevatorConstants.SIM_MOTOR,
                    1.0,
                    ElevatorConstants.SIM_MOMENT_OF_INERTIA));
    lowerLimit =
        new HallEffect(
            "Elevator/LowerLimit",
            new HallEffectConfig(ElevatorConstants.LOWER_LIMIT_DIO_CHANNEL)
                .withInverted(ElevatorConstants.LOWER_LIMIT_INVERTED)
                .withDebounce(
                    ElevatorConstants.LOWER_LIMIT_DEBOUNCE_SECONDS,
                    ElevatorConstants.LOWER_LIMIT_DEBOUNCE_TYPE));
    motor.zeroPosition(0.0);
  }

  @Override
  public void periodic() {
    motor.readInputs();
    heightInches =
        MathUtil.clamp(
            motor.getPosition() * ElevatorConstants.HEIGHT_PER_MOTOR_ROTATION_INCHES,
            0.0,
            MAX_HEIGHT_INCHES);
    velocityInchesPerSecond =
        motor.getVelocity() * ElevatorConstants.HEIGHT_PER_MOTOR_ROTATION_INCHES;
    if (RobotBase.isSimulation()) {
      lowerLimit.setSimState(heightInches <= 0.0);
    }
    lowerLimit.readInputs();

    if (closedLoop) {
      motor.setMotionMagic(targetHeightInches / ElevatorConstants.HEIGHT_PER_MOTOR_ROTATION_INCHES);
    } else if ((atLowerLimit() && manualVolts < 0.0)
        || (atUpperLimit() && manualVolts > 0.0)) {
      motor.stop();
    } else {
      motor.setVoltage(manualVolts);
    }

    SmartDashboard.putNumber("Elevator/HeightInches", heightInches);
    SmartDashboard.putNumber("Elevator/TargetHeightInches", targetHeightInches);
    SmartDashboard.putNumber("Elevator/VelocityInchesPerSecond", velocityInchesPerSecond);
    SmartDashboard.putBoolean("Elevator/AtLowerLimit", atLowerLimit());
    SmartDashboard.putBoolean("Elevator/AtUpperLimit", atUpperLimit());
    Logger.recordOutput("Elevator/HeightInches", heightInches);
    Logger.recordOutput("Elevator/TargetHeightInches", targetHeightInches);
  }

  public void setHeightInches(double heightInches) {
    targetHeightInches = MathUtil.clamp(heightInches, 0.0, MAX_HEIGHT_INCHES);
    closedLoop = true;
  }

  public void setPercentOutput(double percent) {
    closedLoop = false;
    manualVolts = MathUtil.clamp(percent, -1.0, 1.0) * ElevatorConstants.MAX_MANUAL_VOLTS;
  }

  public void runVoltage(double volts) {
    closedLoop = false;
    manualVolts = MathUtil.clamp(volts, -ElevatorConstants.MAX_MANUAL_VOLTS, ElevatorConstants.MAX_MANUAL_VOLTS);
  }

  public void holdCurrentHeight() {
    setHeightInches(heightInches);
  }

  public void stop() {
    closedLoop = false;
    manualVolts = 0.0;
    motor.stop();
  }

  public void zeroHeight() {
    motor.zeroPosition(0.0);
    targetHeightInches = 0.0;
    heightInches = 0.0;
  }

  public boolean isAtHeight(double heightInches) {
    return Math.abs(this.heightInches - heightInches) <= TOLERANCE_INCHES;
  }

  public boolean isAtTarget() {
    return isAtHeight(targetHeightInches);
  }

  public boolean atLowerLimit() {
    return lowerLimit.get() || heightInches <= 0.0;
  }

  public boolean atUpperLimit() {
    return heightInches >= MAX_HEIGHT_INCHES;
  }

  public double getHeightInches() {
    return heightInches;
  }

  public double getTargetHeightInches() {
    return targetHeightInches;
  }
}
