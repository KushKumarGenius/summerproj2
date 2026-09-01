package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState.GamePiece;
import frc.robot.RobotState.ScoringRow;
import frc.robot.autonomous.SimpleAutos;
import frc.robot.commands.drive.TeleopSwerve;
import frc.robot.commands.elevator.ElevatorManualControl;
import frc.robot.commands.intake.IntakeGrabCone;
import frc.robot.commands.intake.IntakeGrabCube;
import frc.robot.commands.intake.IntakeScoreCone;
import frc.robot.commands.intake.IntakeScoreCube;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.commands.mechanism.MechanismPositions;
import frc.robot.commands.stinger.StingerManualControl;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LedIOReal;
import frc.robot.subsystems.stinger.Stinger;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import java.util.Set;

public class RobotContainer {
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final Stinger stinger = new Stinger();
  private final Intake intake = new Intake();
  private final LED leds =
      RobotBase.isReal()
          ? new LED(new LedIOReal(Constants.LED.BLINKIN_PWM_PORT))
          : new LED();
  private final Vision vision;
  private final RobotState robotState = RobotState.getInstance();
  private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {
    vision =
        RobotBase.isReal()
            ? new Vision(
                drivetrain,
                new VisionIOReal(
                    VisionConstants.FRONT_LEFT_CAMERA,
                    VisionConstants.FRONT_LEFT_ROBOT_TO_CAMERA,
                    drivetrain::getPose),
                new VisionIOReal(
                    VisionConstants.FRONT_RIGHT_CAMERA,
                    VisionConstants.FRONT_RIGHT_ROBOT_TO_CAMERA,
                    drivetrain::getPose),
                new VisionIOReal(
                    VisionConstants.BACK_CAMERA,
                    VisionConstants.BACK_ROBOT_TO_CAMERA,
                    drivetrain::getPose))
            : new Vision(
                drivetrain,
                new VisionIOSim(drivetrain::getPose),
                new VisionIOSim(drivetrain::getPose),
                new VisionIOSim(drivetrain::getPose));

    DriverStation.silenceJoystickConnectionWarning(true);
    configureDefaultCommands();
    configureBindings();
    configureAutonomousChooser();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX));
  }

  private void configureBindings() {
    driverController.back().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));
    driverController
        .start()
        .onTrue(Commands.runOnce(drivetrain::enableXStance, drivetrain))
        .onFalse(Commands.runOnce(drivetrain::disableXStance, drivetrain));

    operatorController
        .back()
        .onTrue(Commands.runOnce(() -> robotState.setDesiredGamePiece(GamePiece.CUBE)));
    operatorController
        .start()
        .onTrue(Commands.runOnce(() -> robotState.setDesiredGamePiece(GamePiece.CONE)));
    operatorController
        .povUp()
        .onTrue(Commands.runOnce(() -> robotState.setDesiredScoringRow(ScoringRow.HIGH)));
    operatorController
        .povLeft()
        .onTrue(Commands.runOnce(() -> robotState.setDesiredScoringRow(ScoringRow.MID)));
    operatorController
        .povDown()
        .onTrue(Commands.runOnce(() -> robotState.setDesiredScoringRow(ScoringRow.HYBRID)));

    operatorController
        .rightBumper()
        .whileTrue(
            new ConditionalCommand(
                new IntakeGrabCone(intake),
                new IntakeGrabCube(intake),
                () -> robotState.getDesiredGamePiece() == GamePiece.CONE));
    operatorController
        .leftBumper()
        .whileTrue(
            new ConditionalCommand(
                new IntakeScoreCone(intake),
                new IntakeScoreCube(intake),
                () -> robotState.getDesiredGamePiece() == GamePiece.CONE));

    operatorController
        .x()
        .onTrue(
            MechanismPositions.groundPickup(elevator, stinger)
                .deadlineWith(
                    new ConditionalCommand(
                        new IntakeGrabCone(intake),
                        new IntakeGrabCube(intake),
                        () -> robotState.getDesiredGamePiece() == GamePiece.CONE)
                        .withTimeout(1.0)));
    operatorController.y().onTrue(MechanismPositions.substationPickup(elevator, stinger));
    operatorController.b().onTrue(MechanismPositions.stow(elevator, stinger));

    operatorController
        .leftTrigger(0.5)
        .whileTrue(
            new ElevatorManualControl(elevator, () -> -operatorController.getLeftY())
                .alongWith(
                    new StingerManualControl(
                        stinger, elevator, () -> operatorController.getRightX())));

    driverController
        .rightTrigger(0.8)
        .onTrue(
            Commands.defer(
                () -> MechanismActions.scoreCurrentGamePiece(elevator, stinger, intake),
                Set.of(elevator, stinger, intake)));
  }

  private void configureAutonomousChooser() {
    autonomousChooser.setDefaultOption("Do Nothing", SimpleAutos.doNothing());
    autonomousChooser.addOption(
        "Score Cone", SimpleAutos.scoreCone(drivetrain, elevator, stinger, intake));
    autonomousChooser.addOption(
        "Score Cube", SimpleAutos.scoreCube(drivetrain, elevator, stinger, intake));
    autonomousChooser.addOption(
        "Score Cone and Taxi", SimpleAutos.scoreAndTaxi(drivetrain, elevator, stinger, intake));
    SmartDashboard.putData("Autonomous", autonomousChooser);
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public Vision getVision() {
    return vision;
  }

  public void stopAll() {
    drivetrain.stop();
    elevator.stop();
    stinger.stop();
    intake.stop();
    vision.setEnabled(false);
  }
}
