package frc.robot.subsystems.intake;

import frc.robot.Constants;

public final class IntakeConstants {
  private IntakeConstants() {}

  public static final int MOTOR_ID = Constants.CanId.INTAKE_MOTOR;
  public static final double MAX_VOLTAGE = 11.0;
  public static final double CONE_DIRECTION = 1.0;
  public static final double CUBE_DIRECTION = -1.0;
  public static final double HOLD_VOLTAGE = 0.7;
  public static final double STALL_VELOCITY_RPM = 250.0;
  public static final double STALL_CURRENT_AMPS = 18.0;
}
