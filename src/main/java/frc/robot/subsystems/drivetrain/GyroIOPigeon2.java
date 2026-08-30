package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;

/** Phoenix 6 implementation of the robot's Pigeon 2 gyro. */
public class GyroIOPigeon2 implements GyroIO {
  private static final double UPDATE_FREQUENCY_HZ = 50.0;

  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<AngularVelocity> yawVelocity;

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(DrivetrainConstants.PIGEON_CAN_ID, DrivetrainConstants.CAN_BUS_NAME);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    BaseStatusSignal.setUpdateFrequencyForAll(UPDATE_FREQUENCY_HZ, yaw, pitch, roll, yawVelocity);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(yaw, pitch, roll, yawVelocity);
    inputs.connected = status.isOK();
    inputs.yaw_Rot2d = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.pitch_Rot2d = Rotation2d.fromDegrees(pitch.getValueAsDouble());
    inputs.roll_Rot2d = Rotation2d.fromDegrees(roll.getValueAsDouble());
    inputs.yawVelocity_radps = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.odometryTimestamps_s = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryYawPositions_Rot2d = new Rotation2d[] {inputs.yaw_Rot2d};
  }

  @Override
  public void zero(double angleDegrees) {
    pigeon.setYaw(angleDegrees);
  }
}
