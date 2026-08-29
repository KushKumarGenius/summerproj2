package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Geometry and motion limits for Rober's four-module swerve drive. */
public final class DrivetrainConstants {
  private DrivetrainConstants() {}

  public static final double TRACKWIDTH_METERS = Units.inchesToMeters(23.0);
  public static final double WHEELBASE_METERS = Units.inchesToMeters(25.0);
  public static final double MAX_SPEED_METERS_PER_SECOND = 4.5;
  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND =
      MAX_SPEED_METERS_PER_SECOND
          / Math.hypot(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0);

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
          new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0));
}
