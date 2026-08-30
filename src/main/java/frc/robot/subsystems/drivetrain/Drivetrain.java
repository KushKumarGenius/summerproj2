package frc.robot.subsystems.drivetrain;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.KINEMATICS;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.MAX_FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.MAX_VOLTAGE;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.THROTTLED_MAX_SPEED_METERS_PER_SECOND;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.TRACKWIDTH_METERS;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.WHEELBASE_METERS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Raymond-style IO-driven swerve drivetrain adapted for Team 2930's 2023 hardware. */
public class Drivetrain extends SubsystemBase {
  private enum DriveMode {
    NORMAL,
    X_STANCE,
    CHARACTERIZATION
  }

  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private static final double SKEW_COMPENSATION_SECONDS = -0.0095;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModule[] modules;
  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  private final SwerveModulePosition[] previousModulePositions = new SwerveModulePosition[4];
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field = new Field2d();

  private DriveMode driveMode = DriveMode.NORMAL;
  private boolean fieldRelative = true;
  private Rotation2d fallbackRotation = new Rotation2d();
  private ChassisSpeeds requestedSpeeds = new ChassisSpeeds();
  private ChassisSpeeds outputSpeeds = new ChassisSpeeds();
  private Translation2d centerOfRotation = new Translation2d();
  private double characterizationVolts;

  /** Builds the real or simulated IO stack for the current runtime. */
  public Drivetrain() {
    this(createGyroIO(), createModuleIOs());
  }

  public Drivetrain(GyroIO gyroIO, SwerveModuleIO[] moduleIO) {
    if (moduleIO.length != 4) {
      throw new IllegalArgumentException("A swerve drivetrain needs exactly four modules");
    }

    this.gyroIO = gyroIO;
    modules = new SwerveModule[4];
    for (int i = 0; i < modules.length; i++) {
      modulePositions[i] = new SwerveModulePosition();
      previousModulePositions[i] = new SwerveModulePosition();
      modules[i] = new SwerveModule(moduleIO[i], i, MAX_SPEED_METERS_PER_SECOND);
    }

    poseEstimator =
        new SwerveDrivePoseEstimator(
            KINEMATICS,
            fallbackRotation,
            modulePositions,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, 0.05),
            VecBuilder.fill(0.5, 0.5, 1.0));
    SmartDashboard.putData("Drivetrain/Field", field);
  }

  private static GyroIO createGyroIO() {
    return RobotBase.isReal() ? new GyroIOPigeon2() : new GyroIOSim();
  }

  private static SwerveModuleIO[] createModuleIOs() {
    SwerveModuleIO[] moduleIO = new SwerveModuleIO[4];
    for (int i = 0; i < moduleIO.length; i++) {
      moduleIO[i] =
          RobotBase.isReal()
              ? new SwerveModuleIOReal(DrivetrainConstants.MODULE_CONFIGS[i])
              : new SwerveModuleIOSim(i);
    }
    return moduleIO;
  }

  @Override
  public void periodic() {
    inputPeriodic();
    handle();
    outputPeriodic();
  }

  /** Reads all IO once per loop and updates timestamped pose estimation. */
  private void inputPeriodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (int i = 0; i < modules.length; i++) {
      modules[i].inputPeriodic();
      modulePositions[i] = modules[i].getPosition();
    }

    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < modulePositions.length; i++) {
      moduleDeltas[i] =
          new SwerveModulePosition(
              modulePositions[i].distanceMeters - previousModulePositions[i].distanceMeters,
              modulePositions[i].angle);
      previousModulePositions[i] = modulePositions[i];
    }

    if (gyroInputs.connected) {
      fallbackRotation = gyroInputs.yaw_Rot2d;
    } else {
      Twist2d twist = KINEMATICS.toTwist2d(moduleDeltas);
      fallbackRotation = fallbackRotation.plus(new Rotation2d(twist.dtheta));
    }

    Rotation2d rotation = gyroInputs.connected ? gyroInputs.yaw_Rot2d : fallbackRotation;
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), rotation, modulePositions);
    field.setRobotPose(getPose());

    SmartDashboard.putBoolean("Drivetrain/GyroConnected", gyroInputs.connected);
    SmartDashboard.putNumber("Drivetrain/PoseX", getPose().getX());
    SmartDashboard.putNumber("Drivetrain/PoseY", getPose().getY());
    SmartDashboard.putNumber("Drivetrain/HeadingDeg", getRotation().getDegrees());
  }

  /** Applies Raymond's acceleration limiting and anti-skew processing. */
  private void handle() {
    if (driveMode != DriveMode.NORMAL) {
      outputSpeeds = new ChassisSpeeds();
      return;
    }

    double maxLinearDelta = MAX_FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED * LOOP_PERIOD_SECONDS;
    double maxAngularDelta =
        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED * LOOP_PERIOD_SECONDS;
    outputSpeeds =
        new ChassisSpeeds(
            limitDelta(
                outputSpeeds.vxMetersPerSecond,
                requestedSpeeds.vxMetersPerSecond,
                maxLinearDelta),
            limitDelta(
                outputSpeeds.vyMetersPerSecond,
                requestedSpeeds.vyMetersPerSecond,
                maxLinearDelta),
            limitDelta(
                outputSpeeds.omegaRadiansPerSecond,
                requestedSpeeds.omegaRadiansPerSecond,
                maxAngularDelta));

    // Pre-rotate the velocity command to compensate for azimuth lag during combined translation
    // and rotation. This is the same plant-specific correction used by Raymond's drive code.
    double perpendicularX = -outputSpeeds.vyMetersPerSecond;
    double perpendicularY = outputSpeeds.vxMetersPerSecond;
    outputSpeeds.vxMetersPerSecond +=
        perpendicularX * outputSpeeds.omegaRadiansPerSecond * SKEW_COMPENSATION_SECONDS;
    outputSpeeds.vyMetersPerSecond +=
        perpendicularY * outputSpeeds.omegaRadiansPerSecond * SKEW_COMPENSATION_SECONDS;

    Logger.recordOutput("Drive/Speeds/Requested", requestedSpeeds);
    Logger.recordOutput("Drive/Speeds/Output", outputSpeeds);
  }

  private static double limitDelta(double current, double target, double maxDelta) {
    return current + MathUtil.clamp(target - current, -maxDelta, maxDelta);
  }

  /** Optimizes, desaturates, logs, and sends module setpoints to the selected IO implementation. */
  private void outputPeriodic() {
    if (DriverStation.isDisabled()) {
      for (SwerveModule module : modules) {
        module.stop();
      }
      Logger.recordOutput("Drive/Modules/Setpoints", new SwerveModuleState[] {});
      return;
    }

    if (driveMode == DriveMode.X_STANCE) {
      applyXStance();
      return;
    }

    if (driveMode == DriveMode.CHARACTERIZATION) {
      for (SwerveModule module : modules) {
        module.runCharacterization(characterizationVolts);
      }
      return;
    }

    SwerveModuleState[] setpointStates = KINEMATICS.toSwerveModuleStates(outputSpeeds, centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_SPEED_METERS_PER_SECOND);
    SwerveModuleState[] optimizedStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
      modules[i].outputPeriodic(SwerveModule.Mode.HIGH_CONTROL);
    }

    Logger.recordOutput("Drive/Modules/Setpoints", setpointStates);
    Logger.recordOutput("Drive/Modules/SetpointsOptimized", optimizedStates);
    Logger.recordOutput("Drive/Pose", getPose());
  }

  public void drive(double xMetersPerSecond, double yMetersPerSecond, double omegaRadiansPerSecond) {
    if (driveMode != DriveMode.NORMAL) {
      return;
    }
    requestedSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond, getRotation())
            : new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond);
  }

  /** Accepts robot-relative speeds for autonomous and path-following callers. */
  public void drive(ChassisSpeeds robotRelativeSpeeds) {
    if (driveMode == DriveMode.NORMAL) {
      requestedSpeeds = robotRelativeSpeeds;
    }
  }

  public void stop() {
    requestedSpeeds = new ChassisSpeeds();
    outputSpeeds = new ChassisSpeeds();
    driveMode = DriveMode.NORMAL;
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  public void stopWithX() {
    requestedSpeeds = new ChassisSpeeds();
    driveMode = DriveMode.X_STANCE;
    applyXStance();
  }

  public void zeroGyroscope() {
    fallbackRotation = new Rotation2d();
    gyroIO.zero(0.0);
    poseEstimator.resetRotation(fallbackRotation);
  }

  public void resetPose(Pose2d pose) {
    fallbackRotation = pose.getRotation();
    gyroIO.zero(pose.getRotation().getDegrees());
    poseEstimator.resetPosition(fallbackRotation, modulePositions, pose);
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return gyroInputs.connected ? gyroInputs.yaw_Rot2d : fallbackRotation;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Requested")
  public ChassisSpeeds getRequestedSpeeds() {
    return requestedSpeeds;
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    if (timestampSeconds > 0.0) {
      poseEstimator.addVisionMeasurement(
          pose, timestampSeconds, VecBuilder.fill(0.35, 0.35, 0.7));
    }
  }

  public void enableFieldRelative() {
    fieldRelative = true;
  }

  public void disableFieldRelative() {
    fieldRelative = false;
  }

  public boolean isFieldRelative() {
    return fieldRelative;
  }

  public void enableXStance() {
    driveMode = DriveMode.X_STANCE;
    applyXStance();
  }

  public void disableXStance() {
    driveMode = DriveMode.NORMAL;
  }

  public boolean isXStance() {
    return driveMode == DriveMode.X_STANCE;
  }

  private void applyXStance() {
    double angle = Math.atan2(WHEELBASE_METERS, TRACKWIDTH_METERS);
    Rotation2d[] stanceAngles =
        new Rotation2d[] {
          new Rotation2d(angle),
          new Rotation2d(-angle),
          new Rotation2d(-angle),
          new Rotation2d(angle)
        };
    for (int i = 0; i < modules.length; i++) {
      modules[i].runSetpoint(new SwerveModuleState(0.0, stanceAngles[i]));
      modules[i].outputPeriodic(SwerveModule.Mode.HIGH_CONTROL);
    }
  }

  public void setCenterOfRotation(Translation2d centerOfRotation) {
    this.centerOfRotation = centerOfRotation;
  }

  public void resetCenterOfRotation() {
    centerOfRotation = new Translation2d();
  }

  public void runCharacterizationVolts(double volts) {
    driveMode = DriveMode.CHARACTERIZATION;
    characterizationVolts = MathUtil.clamp(volts, -MAX_VOLTAGE, MAX_VOLTAGE);
  }

  public double getCharacterizationVelocity() {
    double sum = 0.0;
    for (SwerveModuleState state : getModuleStates()) {
      sum += state.speedMetersPerSecond;
    }
    return sum / modules.length;
  }

  public double getMaxAngularSpeed() {
    return MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
  }

  public double getMaxSpeedMetersPerSecond() {
    return fieldRelative ? MAX_SPEED_METERS_PER_SECOND : THROTTLED_MAX_SPEED_METERS_PER_SECOND;
  }
}
