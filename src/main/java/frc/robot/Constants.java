package frc.robot;

/** Robot-wide configuration for Team 2930's 2023 Rober rewrite. */
public final class Constants {
  public static final double LOOP_PERIOD_SECONDS = 0.02;

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double AXIS_DEADBAND = 0.1;

    private OperatorConstants() {}
  }

  public static final class CanId {
    public static final int STINGER_MOTOR = 5;
    public static final int INTAKE_MOTOR = 6;
    public static final int ELEVATOR_LEAD_MOTOR = 9;
    public static final int ELEVATOR_FOLLOW_MOTOR = 10;
    public static final int PIGEON = 15;

    private CanId() {}
  }

  public static final class Elevator {
    public static final double MAX_HEIGHT_INCHES = 48.7;
    public static final double STOW_HEIGHT_INCHES = 7.5;
    public static final double GROUND_PICKUP_HEIGHT_INCHES = 3.0;
    public static final double SUBSTATION_HEIGHT_INCHES = 45.7;
    public static final double HIGH_CUBE_HEIGHT_INCHES = 46.5;
    public static final double MID_CUBE_HEIGHT_INCHES = 30.0;
    public static final double MID_CONE_HEIGHT_INCHES = 35.0;
    public static final double LOW_HEIGHT_INCHES = 11.0;

    private Elevator() {}
  }

  public static final class Stinger {
    public static final double MAX_EXTENSION_INCHES = 25.0;
    public static final double STOW_EXTENSION_INCHES = 0.0;
    public static final double GROUND_PICKUP_EXTENSION_INCHES = 11.0;
    public static final double SUBSTATION_EXTENSION_INCHES = 1.5;
    public static final double HIGH_EXTENSION_INCHES = 25.0;
    public static final double MID_CONE_EXTENSION_INCHES = 13.0;
    public static final double MID_CUBE_EXTENSION_INCHES = 16.3;
    public static final double LOW_EXTENSION_INCHES = 7.0;

    private Stinger() {}
  }

  public static final class LED {
    public static final int BLINKIN_PWM_PORT = 0;

    private LED() {}
  }

  private Constants() {}
}
