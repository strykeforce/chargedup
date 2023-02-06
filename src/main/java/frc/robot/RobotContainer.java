// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.drive.xLockCommand;
import frc.robot.commands.elbow.ElbowOpenLoopCommand;
import frc.robot.commands.elevator.ElevatorSpeedCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.commands.intake.IntakeExtendCommand;
import frc.robot.commands.intake.IntakeOpenLoopCommand;
import frc.robot.commands.intake.ToggleIntakeExtendedCommand;
import frc.robot.commands.shoulder.ShoulderSpeedCommand;
import frc.robot.commands.shoulder.ZeroShoulderCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.Map;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {
  private ShoulderSubsystem shoulderSubsystem;

  private RobotStateSubsystem robotStateSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final ElbowSubsystem elbowSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private final XboxController xboxController = new XboxController(1);
  private final Joystick driveJoystick = new Joystick(0);
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  private static final double kJoystickDeadband = Constants.kJoystickDeadband;

  // Dashboard Widgets
  private SuppliedValueWidget<Boolean> allianceColor;
  private Alliance alliance = Alliance.Invalid;

  // Paths
  private DriveAutonCommand testPath;

  public RobotContainer() {
    intakeSubsystem = new IntakeSubsystem();
    driveSubsystem = new DriveSubsystem();
    visionSubsystem = new VisionSubsystem(driveSubsystem);
    robotStateSubsystem = new RobotStateSubsystem(TargetLevel.NONE, TargetCol.NONE, GamePiece.NONE);
    driveSubsystem.setRobotStateSubsystem(robotStateSubsystem);
    elevatorSubsystem = new ElevatorSubsystem();
    elbowSubsystem = new ElbowSubsystem();
    shoulderSubsystem = new ShoulderSubsystem();

    driveSubsystem.setVisionSubsystem(visionSubsystem);
    driveSubsystem.registerWith(telemetryService);
    robotStateSubsystem.registerWith(telemetryService);
    elevatorSubsystem.registerWith(telemetryService);
    elbowSubsystem.registerWith(telemetryService);
    shoulderSubsystem.registerWith(telemetryService);
    visionSubsystem.setFillBuffers(true);

    configureTelemetry();
    configurePaths();
    configureDriverButtonBindings();
    configureMatchDashboard();
    configurePitDashboard();
  }

  private void configureTelemetry() {
    driveSubsystem.registerWith(telemetryService);
    visionSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }

  private void configurePaths() {
    testPath = new DriveAutonCommand(driveSubsystem, "mirrorTestPath", true, true);

    new JoystickButton(driveJoystick, Trim.LEFT_X_NEG.id)
        .onFalse(new ShoulderSpeedCommand(shoulderSubsystem, 0))
        .onTrue(new ShoulderSpeedCommand(shoulderSubsystem, -0.45));
    new JoystickButton(driveJoystick, Trim.LEFT_X_POS.id)
        .onFalse(new ShoulderSpeedCommand(shoulderSubsystem, 0))
        .onTrue(new ShoulderSpeedCommand(shoulderSubsystem, 0.4));
  }

  private void configureDriverButtonBindings() {
    driveSubsystem.setDefaultCommand(
        new DriveTeleopCommand(driveJoystick, driveSubsystem, robotStateSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.RESET.id)
        .onTrue(new ZeroGyroCommand(driveSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.X.id)
        .onTrue(new xLockCommand(driveSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.HAMBURGER.id)
        .onTrue(new ResetOdometryCommand(driveSubsystem));
    // Requires swerve migration to new Pose2D
    // new JoystickButton(joystick, InterlinkButton.HAMBURGER.id).whenPressed(() ->
    // {driveSubsystem.resetOdometry(new Pose2d());},driveSubsystem);
    new JoystickButton(driveJoystick, InterlinkButton.HAMBURGER.id).onTrue(testPath);

    // Elevator testing
    new JoystickButton(driveJoystick, Trim.RIGHT_X_NEG.id)
        .onTrue(new ElevatorSpeedCommand(elevatorSubsystem, -0.2))
        .onFalse(new ElevatorSpeedCommand(elevatorSubsystem, 0));
    new JoystickButton(driveJoystick, Trim.RIGHT_X_POS.id)
        .onTrue(new ElevatorSpeedCommand(elevatorSubsystem, 0.2))
        .onFalse(new ElevatorSpeedCommand(elevatorSubsystem, 0));
    new JoystickButton(driveJoystick, InterlinkButton.DOWN.id)
        .onTrue(new ZeroElevatorCommand(elevatorSubsystem));

    // Elbow testing
    new JoystickButton(driveJoystick, Trim.LEFT_Y_NEG.id)
        .onFalse(new ElbowOpenLoopCommand(elbowSubsystem, 0))
        .onTrue(new ElbowOpenLoopCommand(elbowSubsystem, -0.1));
    new JoystickButton(driveJoystick, Trim.LEFT_Y_POS.id)
        .onFalse(new ElbowOpenLoopCommand(elbowSubsystem, 0))
        .onTrue(new ElbowOpenLoopCommand(elbowSubsystem, 0.1));

    // intake buttons
    // new JoystickButton(xboxController, 3).onTrue(new
    // ToggleIntakeExtendedCommand(intakeSubsystem));
    new JoystickButton(driveJoystick, Shoulder.RIGHT_DOWN.id)
        .onTrue(new ToggleIntakeExtendedCommand(intakeSubsystem))
        .onFalse(new ToggleIntakeExtendedCommand(intakeSubsystem));
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
  }

  public void setAllianceColor(Alliance alliance) {
    this.alliance = alliance;
    allianceColor.withProperties(
        Map.of(
            "colorWhenTrue", alliance == Alliance.Red ? "red" : "blue", "colorWhenFalse", "black"));
    robotStateSubsystem.setAllianceColor(alliance);
    testPath.generateTrajectory();
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
