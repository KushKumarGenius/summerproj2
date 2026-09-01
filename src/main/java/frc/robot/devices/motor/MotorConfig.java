package frc.robot.devices.motor;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.system.plant.DCMotor;

public class MotorConfig {
  public enum GravityType {
    NONE,
    ELEVATOR,
    ARM
  }

  public final int canId;
  public String canBus = "";
  public Integer followerId;
  public MotorAlignmentValue followerAlignment = MotorAlignmentValue.Opposed;
  public boolean inverted;
  public boolean brake = true;
  public double supplyCurrentLimit = 40.0;
  public boolean foc = true;
  public double sensorToMechanismRatio = 1.0;
  public double kS;
  public double kV;
  public double kA;
  public double kG;
  public double kP;
  public double kI;
  public double kD;
  public GravityType gravity = GravityType.NONE;
  public double motionMagicCruiseVelocity;
  public double motionMagicAcceleration;
  public double motionMagicJerk;
  public DCMotor simMotor = DCMotor.getFalcon500(1);
  public double simGearing = 1.0;
  public double simMomentOfInertia = 0.001;

  public MotorConfig(int canId) {
    this.canId = canId;
  }

  public MotorConfig withCanBus(String canBus) {
    this.canBus = canBus;
    return this;
  }

  public MotorConfig withFollower(int followerId, MotorAlignmentValue alignment) {
    this.followerId = followerId;
    this.followerAlignment = alignment;
    return this;
  }

  public MotorConfig withInverted(boolean inverted) {
    this.inverted = inverted;
    return this;
  }

  public MotorConfig withBrake(boolean brake) {
    this.brake = brake;
    return this;
  }

  public MotorConfig withSupplyCurrentLimit(double amps) {
    supplyCurrentLimit = amps;
    return this;
  }

  public MotorConfig withFoc(boolean foc) {
    this.foc = foc;
    return this;
  }

  public MotorConfig withSensorToMechanismRatio(double ratio) {
    sensorToMechanismRatio = ratio;
    return this;
  }

  public MotorConfig withFeedforward(double kS, double kV, double kA, double kG) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kG = kG;
    return this;
  }

  public MotorConfig withPid(double kP, double kI, double kD, GravityType gravity) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.gravity = gravity;
    return this;
  }

  public MotorConfig withMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
    motionMagicCruiseVelocity = cruiseVelocity;
    motionMagicAcceleration = acceleration;
    motionMagicJerk = jerk;
    return this;
  }

  public MotorConfig withSimulation(DCMotor motor, double gearing, double momentOfInertia) {
    simMotor = motor;
    simGearing = gearing;
    simMomentOfInertia = momentOfInertia;
    return this;
  }
}
