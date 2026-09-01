package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DrivetrainConstants {
  private DrivetrainConstants() {}

  public record ModuleConfig(
      int driveCanId,
      int steerCanId,
      int encoderCanId,
      double encoderOffsetDegrees,
      boolean driveInverted,
      boolean steerInverted,
      boolean encoderInverted) {}

  public static final double TRACKWIDTH_METERS = Units.inchesToMeters(23.0);
  public static final double WHEELBASE_METERS = Units.inchesToMeters(25.0);
  public static final double WHEEL_CIRCUMFERENCE_METERS = 0.308;
  public static final double DRIVE_GEAR_RATIO =
      1.0 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
  public static final double STEER_GEAR_RATIO = 1.0 / ((14.0 / 50.0) * (10.0 / 60.0));

  public static final double MAX_SPEED_METERS_PER_SECOND =
      6380.0 / 60.0 / DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE_METERS;
  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND =
      MAX_SPEED_METERS_PER_SECOND
          / Math.hypot(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0);

  public static final double THROTTLED_MAX_SPEED_METERS_PER_SECOND = 3.5;
  public static final double MAX_FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED = 55.0;
  public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED =
      22.0 * MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
  public static final double MAX_VOLTAGE = 12.0;

  public static final int PIGEON_CAN_ID = 15;
  public static final String CAN_BUS_NAME = "";

  public static final ModuleConfig FRONT_LEFT =
      new ModuleConfig(1, 11, 21, 221.4, true, true, false);
  public static final ModuleConfig FRONT_RIGHT =
      new ModuleConfig(2, 12, 22, 190.4, true, true, false);
  public static final ModuleConfig BACK_LEFT =
      new ModuleConfig(4, 14, 24, 179.2, true, false, false);
  public static final ModuleConfig BACK_RIGHT =
      new ModuleConfig(3, 13, 23, 311.9, false, true, false);

  public static final ModuleConfig[] MODULE_CONFIGS =
      new ModuleConfig[] {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
          new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0));
}
