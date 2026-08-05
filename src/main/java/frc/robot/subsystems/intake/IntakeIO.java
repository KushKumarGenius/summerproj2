package frc.robot.subsystems.intake;

public interface IntakeIO {
  // Information sent from the motor/sensor layer to the subsystem.
  // The subsystem does not need to know whether this came from hardware or simulation.
  class IntakeIOInputs {
    // Voltage currently being sent to the intake motor.
    public double appliedVolts = 0.0;

    // Intake roller speed, useful for simulation, logging, and debugging.
    public double velocityRPM = 0.0;

    // This will eventually come from a real game-piece sensor.
    // The simulator currently leaves it false.
    public boolean hasPiece = false;
  }

  // Refresh sensor and motor information before Intake.periodic() uses it.
  default void updateInputs(IntakeIOInputs inputs) {}

  // Tell the hardware or simulator what motor voltage to apply.
  default void setVoltage(double volts) {}

  // Every IO implementation can use the same stopping behavior.
  default void stop() {
    setVoltage(0.0);
  }
}