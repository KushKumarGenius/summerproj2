package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.stinger.Stinger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class SubsystemTest {
  @AfterEach
  void cleanScheduler() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Test
  void intakePreservesGamePieceDirectionAndStops() {
    RecordingIntakeIO io = new RecordingIntakeIO();
    Intake intake = new Intake(io);

    intake.intakeCone(0.5);
    intake.periodic();
    assertEquals(5.5, io.lastVoltage, 1e-9);
    assertEquals(Intake.State.INTAKING, intake.getState());

    intake.stop();
    assertEquals(0.0, io.lastVoltage, 1e-9);
    assertEquals(Intake.State.IDLE, intake.getState());
  }

  @Test
  void elevatorAndStingerClampTargetsToSafeRanges() {
    Elevator elevator = new Elevator();
    Stinger stinger = new Stinger();

    elevator.setHeightInches(100.0);
    stinger.setExtensionInches(100.0);

    assertEquals(Elevator.MAX_HEIGHT_INCHES, elevator.getTargetHeightInches(), 1e-9);
    assertEquals(Stinger.MAX_EXTENSION_INCHES, stinger.getTargetExtensionInches(), 1e-9);
  }

  @Test
  void drivetrainProducesFourModuleStates() {
    Drivetrain drivetrain = new Drivetrain();
    drivetrain.periodic();
    drivetrain.drive(1.0, 0.0, 0.0);
    drivetrain.periodic();

    assertEquals(4, drivetrain.getModuleStates().length);
    assertTrue(drivetrain.getRequestedSpeeds().vxMetersPerSecond != 0.0);
  }

  private static class RecordingIntakeIO implements IntakeIO {
    private double lastVoltage;

    @Override
    public void setVoltage(double volts) {
      lastVoltage = volts;
    }
  }
}
