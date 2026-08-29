package frc.robot.subsystems.drivetrain;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.KINEMATICS;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.TRACKWIDTH_METERS;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.WHEELBASE_METERS;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Four-module swerve drive with odometry, field-relative driving, and vision fusion. */
public class Drivetrain extends SubsystemBase {
  private enum DriveMode {
    NORMAL,
    X_STANCE,
    CHARACTERIZATION
  }

  private final GyroIO gyroIO;
  private final GyroIO.GyroIOInputs gyroInputs = new GyroIO.GyroIOInputs();
  private final SwerveModule[] modules;
  private final SwerveModulePosition[] modulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveModulePosition[] previousModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field = new Field2d();

  private DriveMode driveMode = DriveMode.NORMAL;
  private boolean fieldRelative = true;
  private Rotation2d fallbackRotation = new Rotation2d();
  private ChassisSpeeds requestedSpeeds = new ChassisSpeeds();
  private Translation2d centerOfRotation = new Translation2d();
  private double characterizationVolts;

  /** Creates a simulation-backed drivetrain. */
  public Drivetrain() {
    this(
        new GyroIO() {},
        new SwerveModuleIO[] {
          new SwerveModuleIOSim(0),
          new SwerveModuleIOSim(1),
          new SwerveModuleIOSim(2),
          new SwerveModuleIOSim(3)
        });
  }

  public Drivetrain(GyroIO gyroIO, SwerveModuleIO[] moduleIO) {
    if (moduleIO.length != 4) {
      throw new IllegalArgumentException("A swerve drivetrain needs exactly four modules");
    }

    this.gyroIO = gyroIO;
    modules = new SwerveModule[4];
    for (int i = 0; i < modules.length; i++) {
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

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    for (SwerveModule module : modules) {
      module.periodic();
      modulePositions[module.getModuleNumber()] = module.getPosition();
    }

    if (!gyroInputs.connected) {
      SwerveModulePosition[] deltas = new SwerveModulePosition[4];
      for (int i = 0; i < modulePositions.length; i++) {
        SwerveModulePosition current = modulePositions[i];
        SwerveModulePosition previous = previousModulePositions[i];
        deltas[i] =
            new SwerveModulePosition(current.distanceMeters - previous.distanceMeters, current.angle);
        previousModulePositions[i] = current;
      }
      Twist2d twist = KINEMATICS.toTwist2d(deltas);
      fallbackRotation = fallbackRotation.plus(new Rotation2d(twist.dtheta));
    } else {
      fallbackRotation = new Rotation2d(gyroInputs.yawRadians);
    }

    Rotation2d rotation = gyroInputs.connected ? new Rotation2d(gyroInputs.yawRadians) : fallbackRotation;
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), rotation, modulePositions);
    field.setRobotPose(poseEstimator.getEstimatedPosition());

    SmartDashboard.putString("Drivetrain/Mode", driveMode.name());
    SmartDashboard.putBoolean("Drivetrain/FieldRelative", fieldRelative);
    SmartDashboard.putNumber("Drivetrain/PoseX", getPose().getX());
    SmartDashboard.putNumber("Drivetrain/PoseY", getPose().getY());
    SmartDashboard.putNumber("Drivetrain/HeadingDeg", getRotation().getDegrees());
    SmartDashboard.putNumber("Drivetrain/RequestedVx", requestedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Drivetrain/RequestedVy", requestedSpeeds.vyMetersPerSecond);
  }

  public void drive(double xMetersPerSecond, double yMetersPerSecond, double omegaRadiansPerSecond) {
    if (driveMode != DriveMode.NORMAL) {
      if (driveMode == DriveMode.X_STANCE) {
        setXStance();
      } else {
        for (SwerveModule module : modules) {
          module.setCharacterizationVoltage(characterizationVolts);
        }
      }
      return;
    }

    requestedSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond, getRotation())
            : new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond);
    drive(requestedSpeeds);
  }

  public void drive(ChassisSpeeds robotRelativeSpeeds) {
    requestedSpeeds = robotRelativeSpeeds;
    SwerveModuleState[] states =
        KINEMATICS.toSwerveModuleStates(robotRelativeSpeeds, centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED_METERS_PER_SECOND);
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(states[i], true, false);
    }
  }

  public void stop() {
    requestedSpeeds = new ChassisSpeeds();
    drive(new ChassisSpeeds());
  }

  public void zeroGyroscope() {
    fallbackRotation = new Rotation2d();
    if (!gyroInputs.connected) {
      poseEstimator.resetRotation(fallbackRotation);
    }
  }

  public void resetPose(Pose2d pose) {
    fallbackRotation = pose.getRotation();
    poseEstimator.resetPosition(fallbackRotation, modulePositions, pose);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return gyroInputs.connected ? new Rotation2d(gyroInputs.yawRadians) : fallbackRotation;
  }

  public ChassisSpeeds getRequestedSpeeds() {
    return requestedSpeeds;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
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
    setXStance();
  }

  public void disableXStance() {
    driveMode = DriveMode.NORMAL;
  }

  public boolean isXStance() {
    return driveMode == DriveMode.X_STANCE;
  }

  public void setXStance() {
    double angle = Math.atan2(WHEELBASE_METERS, TRACKWIDTH_METERS);
    Rotation2d[] stanceAngles =
        new Rotation2d[] {
          new Rotation2d(angle),
          new Rotation2d(-angle),
          new Rotation2d(-angle),
          new Rotation2d(angle)
        };
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(new SwerveModuleState(0.0, stanceAngles[i]), true, true);
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
    characterizationVolts = volts;
  }

  public double getCharacterizationVelocity() {
    double sum = 0.0;
    for (SwerveModuleState state : getModuleStates()) {
      sum += state.speedMetersPerSecond;
    }
    return sum / modules.length;
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    if (timestampSeconds > 0.0) {
      poseEstimator.addVisionMeasurement(
          pose, timestampSeconds, VecBuilder.fill(0.35, 0.35, 0.7));
    }
  }

  public double getMaxAngularSpeed() {
    return MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
  }
}
