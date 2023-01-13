// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private final IntakeSubsystem intakeSubsystem;

  private final XboxController xboxController = new XboxController(1);

  public RobotContainer() {
    intakeSubsystem = new IntakeSubsystem();

    configureBindings();
    configureMatchDashboard();
  }

  private void configureBindings() {
    // intake buttons
    new JoystickButton(xboxController, 0).whileTrue(new ToggleIntakeCommand(intakeSubsystem)); // FIXME: correct button id
  }

  private void configureMatchDashboard() {
    Shuffleboard.getTab("Debug").add("ToggleIntakeExtended", new ToggleIntakeCommand(intakeSubsystem)).withSize(1, 1).withPosition(1, 1);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
