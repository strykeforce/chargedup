// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.intake.IntakeExtendCommand;
import frc.robot.commands.intake.IntakeOpenLoopCommand;
import frc.robot.commands.intake.ToggleIntakeExtendedCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private final IntakeSubsystem intakeSubsystem;

  private final XboxController xboxController = new XboxController(1);

  public RobotContainer() {
    intakeSubsystem = new IntakeSubsystem();

    configureBindings();
    configurePitDashboard();
  }

  private void configureBindings() {
    // intake buttons
    new JoystickButton(xboxController, 0)
        .whileTrue(new ToggleIntakeExtendedCommand(intakeSubsystem)); // FIXME: correct button id
  }

  private void configurePitDashboard() {
    // intake buttons
    ShuffleboardTab pitTab = Shuffleboard.getTab("Pit");

    ShuffleboardLayout intakeCommands =
        pitTab.getLayout("Intake", BuiltInLayouts.kGrid).withPosition(3, 0).withSize(1, 2);
    intakeCommands
        .add("FWD", new IntakeOpenLoopCommand(intakeSubsystem, IntakeConstants.kIntakeSpeed))
        .withSize(1, 1)
        .withPosition(0, 0);
    intakeCommands
        .add("REV", new IntakeOpenLoopCommand(intakeSubsystem, IntakeConstants.kIntakeReverseSpeed))
        .withSize(1, 1)
        .withPosition(0, 1);
    intakeCommands
        .add("EXTEND", new IntakeExtendCommand(intakeSubsystem, true))
        .withSize(1, 1)
        .withPosition(0, 2);
    intakeCommands
        .add("RETRACT", new IntakeExtendCommand(intakeSubsystem, false))
        .withSize(1, 1)
        .withPosition(0, 3);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
