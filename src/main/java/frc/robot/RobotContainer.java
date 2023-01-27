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
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.drive.xLockCommand;
import frc.robot.commands.elbow.ElbowOpenLoopCommand;
import frc.robot.commands.elevator.ElevatorSpeedCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;
import frc.robot.subsytems.ElbowSubsystem;
import java.util.Map;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {
  private RobotStateSubsystem robotStateSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final ElbowSubsystem elbowSubsystem;

  private final XboxController xboxController = new XboxController(1);
  private final Joystick driveJoystick = new Joystick(0);

  private static final double kJoystickDeadband = Constants.kJoystickDeadband;
  private final TelemetryService telemetryService;

  // Dashboard Widgets
  private SuppliedValueWidget<Boolean> allianceColor;
  private Alliance alliance = Alliance.Invalid;

  // Paths
  private DriveAutonCommand testPath;

  private final ElevatorSubsystem elevatorSubsystem;

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();
    robotStateSubsystem = new RobotStateSubsystem(TargetLevel.NONE, TargetCol.NONE, GamePiece.NONE);
    driveSubsystem.setRobotStateSubsystem(robotStateSubsystem);
    elevatorSubsystem = new ElevatorSubsystem();
    elbowSubsystem = new ElbowSubsystem();

    telemetryService = new TelemetryService(TelemetryController::new);

    driveSubsystem.registerWith(telemetryService);
    robotStateSubsystem.registerWith(telemetryService);
    driveSubsystem.registerWith(telemetryService);
    elevatorSubsystem.registerWith(telemetryService);
    elbowSubsystem.registerWith(telemetryService);

    telemetryService.start();

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configurePaths();
    configureMatchDashboard();
    configurePitDashboard();
  }

  private void configurePaths() {
    testPath = new DriveAutonCommand(driveSubsystem, "mirrorTestPath", true, true);
  }

  private void configureDriverButtonBindings() {
    driveSubsystem.setDefaultCommand(
        new DriveTeleopCommand(driveJoystick, driveSubsystem, robotStateSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.RESET.id)
        .onTrue(new ZeroGyroCommand(driveSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.X.id)
        .onTrue(new xLockCommand(driveSubsystem));
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
  }

  private void configureOperatorButtonBindings() {}

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
