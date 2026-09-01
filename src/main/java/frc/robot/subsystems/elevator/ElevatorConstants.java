package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

public final class ElevatorConstants {
  private ElevatorConstants() {}

  public static final int MOTOR_ID = Constants.CanId.ELEVATOR_LEAD_MOTOR;
  public static final int FOLLOWER_ID = Constants.CanId.ELEVATOR_FOLLOW_MOTOR;
  public static final MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;
  public static final double MAX_HEIGHT_INCHES = Constants.Elevator.MAX_HEIGHT_INCHES;
  public static final double HEIGHT_PER_MOTOR_ROTATION_INCHES = 45.0 / 103297.0 * 2048.0;
  public static final double TOLERANCE_INCHES = 0.2;
  public static final double MAX_MANUAL_VOLTS = 10.0;
  public static final double HOLDING_VOLTS = 0.18;
  public static final int LOWER_LIMIT_DIO_CHANNEL = -1;
  public static final boolean LOWER_LIMIT_INVERTED = true;
  public static final double LOWER_LIMIT_DEBOUNCE_SECONDS = 0.1;
  public static final DebounceType LOWER_LIMIT_DEBOUNCE_TYPE = DebounceType.kRising;
  public static final DCMotor SIM_MOTOR = DCMotor.getFalcon500(2);
  public static final double SIM_MOMENT_OF_INERTIA = 0.1;
}
