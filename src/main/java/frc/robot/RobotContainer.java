// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.arm.ArmFloorCommand;
import frc.robot.commands.arm.ArmHighCommand;
import frc.robot.commands.arm.ArmIntakeStageCommand;
import frc.robot.commands.arm.ArmLowCommand;
import frc.robot.commands.arm.ArmMidCommand;
import frc.robot.commands.arm.ArmShelfCommand;
import frc.robot.commands.arm.ArmToIntakeCommand;
import frc.robot.commands.arm.StowArmCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.DriveToPlaceNotPathCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.drive.xLockCommand;
import frc.robot.commands.elevator.ElevatorSpeedCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.commands.hand.GrabConeCommand;
import frc.robot.commands.hand.GrabCubeCommand;
import frc.robot.commands.hand.HandLeftSpeedCommand;
import frc.robot.commands.hand.HandToPositionCommand;
import frc.robot.commands.hand.ZeroHandCommand;
import frc.robot.commands.intake.IntakeExtendCommand;
import frc.robot.commands.intake.IntakeOpenLoopCommand;
import frc.robot.commands.intake.ToggleIntakeExtendedCommand;
import frc.robot.commands.robotState.SetTargetColCommand;
import frc.robot.commands.shoulder.ShoulderSpeedCommand;
import frc.robot.commands.shoulder.ZeroShoulderCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;
import frc.robot.subsystems.ShoulderSubsystem;
import java.util.Map;
import org.strykeforce.healthcheck.HealthCheckCommand;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {
  private final ShoulderSubsystem shoulderSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private final DriveSubsystem driveSubsystem;
  // private final VisionSubsystem visionSubsystem;
  private final ElbowSubsystem elbowSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ArmSubsystem armSubsystem;

  private final XboxController xboxController = new XboxController(1);
  private final Joystick driveJoystick = new Joystick(0);
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  private static final double kJoystickDeadband = Constants.kJoystickDeadband;

  // Dashboard Widgets
  private SuppliedValueWidget<Boolean> allianceColor;
  private Alliance alliance = Alliance.Invalid;

  // Paths
  private DriveAutonCommand testPath;

  private HandSubsystem handSubsystem;

  public RobotContainer() {
    handSubsystem = new HandSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    driveSubsystem = new DriveSubsystem();
    // visionSubsystem = new VisionSubsystem(driveSubsystem);
    robotStateSubsystem =
        new RobotStateSubsystem(
            TargetLevel.NONE,
            TargetCol.NONE,
            GamePiece.NONE,
            true); // TODO: choose correct settings

    driveSubsystem.setRobotStateSubsystem(robotStateSubsystem);

    elevatorSubsystem = new ElevatorSubsystem();
    elbowSubsystem = new ElbowSubsystem();
    shoulderSubsystem = new ShoulderSubsystem();
    armSubsystem = new ArmSubsystem(shoulderSubsystem, elevatorSubsystem, elbowSubsystem);

    // driveSubsystem.setVisionSubsystem(visionSubsystem);
    // visionSubsystem.setFillBuffers(true);

    // FIX ME
    robotStateSubsystem.setAllianceColor(Alliance.Blue);

    configureTelemetry();
    configurePaths();
    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configureMatchDashboard();
    configurePitDashboard();
    new Trigger(RobotController::getUserButton)
        .onTrue(new HealthCheckCommand(driveSubsystem, intakeSubsystem));
  }

  private void configureTelemetry() {
    driveSubsystem.registerWith(telemetryService);
    // visionSubsystem.registerWith(telemetryService);
    robotStateSubsystem.registerWith(telemetryService);
    elevatorSubsystem.registerWith(telemetryService);
    elbowSubsystem.registerWith(telemetryService);
    shoulderSubsystem.registerWith(telemetryService);
    armSubsystem.registerWith(telemetryService);
    handSubsystem.registerWith(telemetryService);
    intakeSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }

  private void configurePaths() {
    testPath = new DriveAutonCommand(driveSubsystem, "mirrorTestPath", true, true);
  }

  private void configureDriverButtonBindings() {
    driveSubsystem.setDefaultCommand(
        new DriveTeleopCommand(driveJoystick, driveSubsystem, robotStateSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.RESET.id)
        .onTrue(new ZeroGyroCommand(driveSubsystem));

    // new JoystickButton(driveJoystick, Trim.RIGHT_X_POS.id)
    // .onTrue(new DriveToPlaceCommand(driveSubsystem, robotStateSubsystem));
    new JoystickButton(driveJoystick, Trim.RIGHT_X_POS.id)
        .onTrue(new DriveToPlaceNotPathCommand(driveSubsystem, robotStateSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.X.id)
        .onTrue(new xLockCommand(driveSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.HAMBURGER.id)
        .onTrue(new ResetOdometryCommand(driveSubsystem, robotStateSubsystem));

    new JoystickButton(driveJoystick, InterlinkButton.UP.id)
        .onTrue(new SetTargetColCommand(robotStateSubsystem, TargetCol.LEFT));
    new JoystickButton(driveJoystick, InterlinkButton.DOWN.id)
        .onTrue(new SetTargetColCommand(robotStateSubsystem, TargetCol.RIGHT));
    new JoystickButton(driveJoystick, Trim.RIGHT_X_NEG.id)
        .onTrue(new SetTargetColCommand(robotStateSubsystem, TargetCol.MID));

    // new JoystickButton(driveJoystick, InterlinkButton.HAMBURGER.id)
    //     .onTrue(new DriveAutonCommand(driveSubsystem, "straightPathX", true, true));
    // Requires swerve migration to new Pose2D
    // new JoystickButton(joystick, InterlinkButton.HAMBURGER.id).whenPressed(() ->
    // {driveSubsystem.resetOdometry(new Pose2d());},driveSubsystem);
    // new JoystickButton(driveJoystick, InterlinkButton.HAMBURGER.id).onTrue(testPath);

    // Hand
    new JoystickButton(driveJoystick, Shoulder.LEFT_DOWN.id)
        .onTrue(
            new HandToPositionCommand(handSubsystem, Constants.HandConstants.kCubeGrabbingPosition))
        .onFalse(new HandToPositionCommand(handSubsystem, Constants.HandConstants.kMaxRev));
    new JoystickButton(driveJoystick, Shoulder.LEFT_UP.id)
        .onTrue(
            new HandToPositionCommand(handSubsystem, Constants.HandConstants.kConeGrabbingPosition))
        .onFalse(new HandToPositionCommand(handSubsystem, Constants.HandConstants.kMaxRev));

    // Shoulder
    new JoystickButton(driveJoystick, Trim.LEFT_X_NEG.id)
        .onFalse(new ShoulderSpeedCommand(shoulderSubsystem, 0))
        .onTrue(new ShoulderSpeedCommand(shoulderSubsystem, -0.35));
    new JoystickButton(driveJoystick, Trim.LEFT_X_POS.id)
        .onFalse(new ShoulderSpeedCommand(shoulderSubsystem, 0))
        .onTrue(new ShoulderSpeedCommand(shoulderSubsystem, 0.3));

    // Elevator testing
    new JoystickButton(driveJoystick, Trim.RIGHT_X_NEG.id)
        .onTrue(new ElevatorSpeedCommand(elevatorSubsystem, -0.2))
        .onFalse(new ElevatorSpeedCommand(elevatorSubsystem, 0));
    new JoystickButton(driveJoystick, Trim.RIGHT_X_POS.id)
        .onTrue(new ElevatorSpeedCommand(elevatorSubsystem, 0.2))
        .onFalse(new ElevatorSpeedCommand(elevatorSubsystem, 0));
    new JoystickButton(driveJoystick, InterlinkButton.UP.id)
        .onTrue(new ZeroElevatorCommand(elevatorSubsystem));

    // // Elbow testing
    // new JoystickButton(driveJoystick, Trim.LEFT_Y_NEG.id)
    //     .onFalse(new ElbowOpenLoopCommand(elbowSubsystem, 0))
    //     .onTrue(new ElbowOpenLoopCommand(elbowSubsystem, -0.1));
    // new JoystickButton(driveJoystick, Trim.LEFT_Y_POS.id)
    //     .onFalse(new ElbowOpenLoopCommand(elbowSubsystem, 0))
    //     .onTrue(new ElbowOpenLoopCommand(elbowSubsystem, 0.1));

    // intake buttons
    // new JoystickButton(xboxController, 3).onTrue(new
    // ToggleIntakeExtendedCommand(intakeSubsystem));
    new JoystickButton(driveJoystick, Shoulder.RIGHT_DOWN.id)
        .onTrue(new ToggleIntakeExtendedCommand(intakeSubsystem))
        .onFalse(new ToggleIntakeExtendedCommand(intakeSubsystem));

    // Toggle auto staging
    // new JoystickButton(driveJoystick, Trim.LEFT_X_POS.id)
    //     .onTrue(new SetAutoStagingCommand(robotStateSubsystem, false));
    // new JoystickButton(driveJoystick, Trim.LEFT_X_NEG.id)
    //     .onTrue(new SetAutoStagingCommand(robotStateSubsystem, false));
    // new JoystickButton(driveJoystick, Trim.RIGHT_X_POS.id)
    //     .onTrue(new SetAutoStagingCommand(robotStateSubsystem, true));
    // new JoystickButton(driveJoystick, Trim.RIGHT_X_NEG.id)
    //     .onTrue(new SetAutoStagingCommand(robotStateSubsystem, true));
  }

  public void configureOperatorButtonBindings() {
    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .onTrue(new StowArmCommand(armSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(new ArmToIntakeCommand(armSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(new ArmIntakeStageCommand(armSubsystem));

    new JoystickButton(xboxController, XboxController.Button.kX.value)
        .onTrue(new ArmHighCommand(armSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
        .onTrue(new ArmLowCommand(armSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
        .onTrue(new ArmMidCommand(armSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kStart.value)
        .onTrue(new ArmFloorCommand(armSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
        .onTrue(new ArmShelfCommand(armSubsystem));

    // new JoystickButton(xboxController, XboxController.Button.kStart.value)
    //     .onTrue(new ElbowToPositionCommand(elbowSubsystem, Constants.ElbowConstants.kStowElbow));
    // Set auto staging target
    // Left column
    // new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
    //     .onTrue(
    //         new SetLevelAndColCommandGroup(robotStateSubsystem, TargetLevel.MID,
    // TargetCol.LEFT));
    // new JoystickButton(xboxController, XboxController.Axis.kLeftTrigger.value)
    //     .onTrue(
    //         new SetLevelAndColCommandGroup(robotStateSubsystem, TargetLevel.HIGH,
    // TargetCol.LEFT));
    // Right column
    // new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
    //     .onTrue(
    //         new SetLevelAndColCommandGroup(robotStateSubsystem, TargetLevel.MID,
    // TargetCol.RIGHT));
    // new JoystickButton(xboxController, XboxController.Axis.kRightTrigger.value)
    //     .onTrue(
    //         new SetLevelAndColCommandGroup(robotStateSubsystem, TargetLevel.HIGH,
    // TargetCol.RIGHT));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void configureMatchDashboard() {
    allianceColor =
        Shuffleboard.getTab("Match")
            .addBoolean("AllianceColor", () -> alliance != Alliance.Invalid)
            .withProperties(Map.of("colorWhenFalse", "black"))
            .withSize(2, 2)
            .withPosition(0, 0);

    Shuffleboard.getTab("Match")
        .addBoolean("IsRight", () -> robotStateSubsystem.getTargetCol() == TargetCol.RIGHT)
        .withSize(1, 1)
        .withPosition(3, 0);
    Shuffleboard.getTab("Match")
        .addBoolean("IsLeft", () -> robotStateSubsystem.getTargetCol() == TargetCol.LEFT)
        .withSize(1, 1)
        .withPosition(3, 1);
  }

  private void configurePitDashboard() {
    ShuffleboardTab pitTab = Shuffleboard.getTab("Pit");

    ShuffleboardLayout intakeCommands =
        pitTab.getLayout("Intake", BuiltInLayouts.kGrid).withPosition(3, 0).withSize(1, 4);
    intakeCommands
        .add("Intake Stop", new IntakeOpenLoopCommand(intakeSubsystem, 0))
        .withPosition(0, 5);
    intakeCommands
        .add("Intake Fwd", new IntakeOpenLoopCommand(intakeSubsystem, IntakeConstants.kIntakeSpeed))
        .withPosition(0, 1);
    intakeCommands
        .add(
            "Intake Rev",
            new IntakeOpenLoopCommand(intakeSubsystem, IntakeConstants.kIntakeEjectSpeed))
        .withPosition(0, 2);
    intakeCommands
        .add("Intake Ext", new IntakeExtendCommand(intakeSubsystem, true))
        .withPosition(0, 3);
    intakeCommands
        .add("Intake Ret", new IntakeExtendCommand(intakeSubsystem, false))
        .withPosition(0, 4);

    // Shoulder Commands
    ShuffleboardLayout shoulderCommands =
        pitTab.getLayout("Shoulder", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(1, 3);
    shoulderCommands
        .add("Shoulder Stop", new ShoulderSpeedCommand(shoulderSubsystem, 0))
        .withPosition(0, 0);
    shoulderCommands
        .add("Shoulder Zero", new ZeroShoulderCommand(shoulderSubsystem))
        .withPosition(0, 1);
    shoulderCommands
        .add("Shoulder Forward", new ShoulderSpeedCommand(shoulderSubsystem, 0.1))
        .withPosition(0, 2);
    shoulderCommands
        .add("Shoulder Reverse", new ShoulderSpeedCommand(shoulderSubsystem, -0.1))
        .withPosition(0, 3);

    // Elevator Commands
    ShuffleboardLayout elevatorCommands =
        pitTab.getLayout("Elevator", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(1, 3);
    elevatorCommands
        .add("Elevator Stop", new ElevatorSpeedCommand(elevatorSubsystem, 0))
        .withPosition(0, 0);
    elevatorCommands
        .add("Elevator Zero", new ZeroElevatorCommand(elevatorSubsystem))
        .withPosition(0, 1);
    elevatorCommands
        .add("Elevator Up", new ElevatorSpeedCommand(elevatorSubsystem, 0.1))
        .withPosition(0, 2);
    elevatorCommands
        .add("Elevator Down", new ElevatorSpeedCommand(elevatorSubsystem, -0.1))
        .withPosition(0, 3);

    // Hand Commands
    ShuffleboardLayout handCommands =
        pitTab.getLayout("Hand", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(1, 9);
    handCommands
        .add("Hand Left Stop", new HandLeftSpeedCommand(handSubsystem, 0))
        .withPosition(0, 0);
    handCommands.add("Hand Zero", new ZeroHandCommand(handSubsystem)).withPosition(0, 2);
    handCommands
        .add("Hand Left In", new HandLeftSpeedCommand(handSubsystem, 0.1))
        .withPosition(0, 3);
    handCommands
        .add("Hand Left Out", new HandLeftSpeedCommand(handSubsystem, -0.1))
        .withPosition(0, 5);
    handCommands.add("Grab Cube", new GrabCubeCommand(handSubsystem)).withPosition(0, 7);
    handCommands.add("Grab Cone", new GrabConeCommand(handSubsystem)).withPosition(0, 8);
  }

  public void setAllianceColor(Alliance alliance) {
    this.alliance = alliance;
    allianceColor.withProperties(
        Map.of(
            "colorWhenTrue", alliance == Alliance.Red ? "red" : "blue", "colorWhenFalse", "black"));
    robotStateSubsystem.setAllianceColor(alliance);
  }

  // Interlink Controller Mapping
  public enum Axis {
    RIGHT_X(1),
    RIGHT_Y(0),
    LEFT_X(2),
    LEFT_Y(5),
    TUNER(6),
    LEFT_BACK(4),
    RIGHT_BACK(3);

    public final int id;

    Axis(int id) {
      this.id = id;
    }
  }

  public enum Shoulder {
    RIGHT_DOWN(2),
    LEFT_DOWN(4),
    LEFT_UP(5);

    private final int id;

    Shoulder(int id) {
      this.id = id;
    }
  }

  public enum Toggle {
    LEFT_TOGGLE(1);

    private final int id;

    Toggle(int id) {
      this.id = id;
    }
  }

  public enum InterlinkButton {
    RESET(3),
    HAMBURGER(14),
    X(15),
    UP(16),
    DOWN(17);

    private final int id;

    InterlinkButton(int id) {
      this.id = id;
    }
  }

  public enum Trim {
    LEFT_Y_POS(7),
    LEFT_Y_NEG(6),
    LEFT_X_POS(8),
    LEFT_X_NEG(9),
    RIGHT_X_POS(10),
    RIGHT_X_NEG(11),
    RIGHT_Y_POS(12),
    RIGHT_Y_NEG(13);

    private final int id;

    Trim(int id) {
      this.id = id;
    }
  }

  public double getLeftX() {
    double val = driveJoystick.getRawAxis(Axis.LEFT_X.id);
    if (Math.abs(val) < kJoystickDeadband) return 0.0;
    return val;
  }

  public double getLeftY() {
    double val = driveJoystick.getRawAxis(Axis.LEFT_Y.id);
    if (Math.abs(val) < kJoystickDeadband) return 0.0;
    return val;
  }

  public double getRightY() {
    double val = driveJoystick.getRawAxis(Axis.RIGHT_Y.id);
    if (Math.abs(val) < kJoystickDeadband) return 0.0;
    return val;
  }
}
