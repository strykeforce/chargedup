// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShoulderSpeedCommand;
import frc.robot.commands.ZeroShoulderCommand;
import frc.robot.subsystems.ShoulderSubsystem;

public class RobotContainer {
  private ShoulderSubsystem shoulderSubsystem;

  public RobotContainer() {
    shoulderSubsystem = new ShoulderSubsystem();

    configurePitDashboard();
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void configurePitDashboard() {
    ShuffleboardTab pitTab = Shuffleboard.getTab("Pit");

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
  }
}
