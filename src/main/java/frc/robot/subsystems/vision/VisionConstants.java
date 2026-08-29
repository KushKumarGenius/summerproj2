package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Camera names and robot-to-camera measurements from Bot 2930's 2023 robot. */
public final class VisionConstants {
  public static final String FRONT_LEFT_CAMERA = "LeftCamera";
  public static final String FRONT_RIGHT_CAMERA = "RightCamera";
  public static final String BACK_CAMERA = "Arducam_OV9281_Camera_4";

  public static final Transform3d FRONT_LEFT_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-0.51), Units.inchesToMeters(10.2), Units.inchesToMeters(22.8)),
          new Rotation3d(0.0, 0.0, Math.toRadians(30.0)));
  public static final Transform3d FRONT_RIGHT_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-0.51), Units.inchesToMeters(-10.2), Units.inchesToMeters(22.8)),
          new Rotation3d(0.0, 0.0, Math.toRadians(-30.0)));
  public static final Transform3d BACK_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-2.7), 0.0, Units.inchesToMeters(33.42)),
          new Rotation3d(0.0, Math.toRadians(10.0), Math.toRadians(180.0)));

  public static final double MAXIMUM_AMBIGUITY = 0.08;
  public static final double MAX_VALID_DISTANCE_METERS = 3.0;

  private VisionConstants() {}
}
