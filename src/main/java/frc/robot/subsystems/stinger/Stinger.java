package frc.robot.subsystems.stinger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.halleffect.HallEffect;
import frc.robot.devices.halleffect.HallEffectConfig;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import org.littletonrobotics.junction.Logger;

public class Stinger extends SubsystemBase {
  public static final double MAX_EXTENSION_INCHES = StingerConstants.MAX_EXTENSION_INCHES;
  public static final double TOLERANCE_INCHES = StingerConstants.TOLERANCE_INCHES;

  private final Motor motor;
  private final HallEffect retractedLimit;
  private boolean closedLoop;
  private double extensionInches;
  private double velocityInchesPerSecond;
  private double targetExtensionInches;
  private double manualVolts;

  public Stinger() {
    motor =
        new Motor(
            "Stinger/Motor",
            new MotorConfig(StingerConstants.MOTOR_ID)
                .withInverted(true)
                .withBrake(false)
                .withSupplyCurrentLimit(20.0)
                .withPid(0.12, 0.0, 0.0, MotorConfig.GravityType.NONE)
                .withSimulation(
                    StingerConstants.SIM_MOTOR,
                    1.0,
                    StingerConstants.SIM_MOMENT_OF_INERTIA));
    retractedLimit =
        new HallEffect(
            "Stinger/RetractedLimit",
            new HallEffectConfig(StingerConstants.RETRACTED_LIMIT_DIO_CHANNEL)
                .withInverted(StingerConstants.RETRACTED_LIMIT_INVERTED)
                .withDebounce(
                    StingerConstants.RETRACTED_LIMIT_DEBOUNCE_SECONDS,
                    StingerConstants.RETRACTED_LIMIT_DEBOUNCE_TYPE));
    motor.zeroPosition(0.0);
  }

  @Override
  public void periodic() {
    motor.readInputs();
    extensionInches =
        MathUtil.clamp(
            motor.getPosition() * StingerConstants.INCHES_PER_MOTOR_ROTATION,
            0.0,
            MAX_EXTENSION_INCHES);
    velocityInchesPerSecond =
        motor.getVelocity() * StingerConstants.INCHES_PER_MOTOR_ROTATION;
    if (RobotBase.isSimulation()) {
      retractedLimit.setSimState(extensionInches <= 0.0);
    }
    retractedLimit.readInputs();

    if (closedLoop) {
      motor.setMotionMagic(targetExtensionInches / StingerConstants.INCHES_PER_MOTOR_ROTATION);
    } else if ((atRetractedLimit() && manualVolts < 0.0)
        || (atExtendedLimit() && manualVolts > 0.0)) {
      motor.stop();
    } else {
      motor.setVoltage(manualVolts);
    }

    SmartDashboard.putNumber("Stinger/ExtensionInches", extensionInches);
    SmartDashboard.putNumber("Stinger/TargetExtensionInches", targetExtensionInches);
    SmartDashboard.putNumber("Stinger/VelocityInchesPerSecond", velocityInchesPerSecond);
    SmartDashboard.putBoolean("Stinger/AtRetractedLimit", atRetractedLimit());
    SmartDashboard.putBoolean("Stinger/AtExtendedLimit", atExtendedLimit());
    Logger.recordOutput("Stinger/ExtensionInches", extensionInches);
    Logger.recordOutput("Stinger/TargetExtensionInches", targetExtensionInches);
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
    motor.stop();
  }

  public void zeroExtension() {
    motor.zeroPosition(0.0);
    targetExtensionInches = 0.0;
    extensionInches = 0.0;
  }

  public boolean isAtExtension(double extensionInches) {
    return Math.abs(this.extensionInches - extensionInches) <= TOLERANCE_INCHES;
  }

  public boolean isAtTarget() {
    return isAtExtension(targetExtensionInches);
  }

  public boolean atRetractedLimit() {
    return retractedLimit.get() || extensionInches <= 0.0;
  }

  public boolean atExtendedLimit() {
    return extensionInches >= MAX_EXTENSION_INCHES;
  }

  public double getExtensionInches() {
    return extensionInches;
  }

  public double getTargetExtensionInches() {
    return targetExtensionInches;
  }
}
