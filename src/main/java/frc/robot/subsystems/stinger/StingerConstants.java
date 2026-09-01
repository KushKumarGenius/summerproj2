package frc.robot.subsystems.stinger;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

public final class StingerConstants {
  private StingerConstants() {}

  public static final int MOTOR_ID = Constants.CanId.STINGER_MOTOR;
  public static final double MAX_EXTENSION_INCHES = Constants.Stinger.MAX_EXTENSION_INCHES;
  public static final double INCHES_PER_MOTOR_ROTATION = 0.144 * (1.125 * Math.PI) * 2.0;
  public static final double TOLERANCE_INCHES = 0.25;
  public static final int RETRACTED_LIMIT_DIO_CHANNEL = -1;
  public static final boolean RETRACTED_LIMIT_INVERTED = true;
  public static final double RETRACTED_LIMIT_DEBOUNCE_SECONDS = 0.1;
  public static final DebounceType RETRACTED_LIMIT_DEBOUNCE_TYPE = DebounceType.kRising;
  public static final DCMotor SIM_MOTOR = DCMotor.getFalcon500(1);
  public static final double SIM_MOMENT_OF_INERTIA = 0.001;
}
