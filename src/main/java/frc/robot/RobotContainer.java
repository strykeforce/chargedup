// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.drive.xLockCommand;
import frc.robot.commands.robot_state.SetAutoStagingCommand;
import frc.robot.commands.robot_state.SetLevelAndColCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;

public class RobotContainer {
  private final RobotStateSubsystem robotStateSubsystem;
  private final DriveSubsystem driveSubsystem;

  private final XboxController xbox = new XboxController(1);
  private final Joystick driveJoystick = new Joystick(0);

  private static final double kJoystickDeadband = Constants.kJoystickDeadband;

  public RobotContainer() {
    robotStateSubsystem =
        new RobotStateSubsystem(
            TargetLevel.NONE,
            TargetCol.NONE,
            GamePiece.NONE,
            true); // TODO: choose correct settings
    driveSubsystem = new DriveSubsystem();

    configurePitDashboard();
    configureBindings();
  }

  private void configureBindings() {}

  private void configureDriverButtonBindings() {
    driveSubsystem.setDefaultCommand(new DriveTeleopCommand(driveJoystick, driveSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.RESET.id)
        .onTrue(new ZeroGyroCommand(driveSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.X.id)
        .onTrue(new xLockCommand(driveSubsystem));

    // Toggle auto staging
    new JoystickButton(driveJoystick, Trim.LEFT_X_POS.id)
        .onTrue(new SetAutoStagingCommand(robotStateSubsystem, false));
    new JoystickButton(driveJoystick, Trim.LEFT_X_NEG.id)
        .onTrue(new SetAutoStagingCommand(robotStateSubsystem, false));
    new JoystickButton(driveJoystick, Trim.RIGHT_X_POS.id)
        .onTrue(new SetAutoStagingCommand(robotStateSubsystem, true));
    new JoystickButton(driveJoystick, Trim.RIGHT_X_NEG.id)
        .onTrue(new SetAutoStagingCommand(robotStateSubsystem, true));
  }

  public void configureOperatorButtonBindings() {
    // Set auto staging target
    // Left column
    new JoystickButton(xbox, XboxController.Button.kLeftBumper.value)
        .onTrue(
            new SetLevelAndColCommandGroup(robotStateSubsystem, TargetLevel.MID, TargetCol.LEFT));
    new JoystickButton(xbox, XboxController.Axis.kLeftTrigger.value)
        .onTrue(
            new SetLevelAndColCommandGroup(robotStateSubsystem, TargetLevel.HIGH, TargetCol.LEFT));
    // Right column
    new JoystickButton(xbox, XboxController.Button.kRightBumper.value)
        .onTrue(
            new SetLevelAndColCommandGroup(robotStateSubsystem, TargetLevel.MID, TargetCol.RIGHT));
    new JoystickButton(xbox, XboxController.Axis.kRightTrigger.value)
        .onTrue(
            new SetLevelAndColCommandGroup(robotStateSubsystem, TargetLevel.HIGH, TargetCol.RIGHT));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void configurePitDashboard() {
    ShuffleboardTab pitTab = Shuffleboard.getTab("Pit");
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
