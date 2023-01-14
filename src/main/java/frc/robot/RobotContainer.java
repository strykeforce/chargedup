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
import frc.robot.commands.GrabConeCommand;
import frc.robot.commands.GrabCubeCommand;
import frc.robot.commands.HandSpeedCommand;
import frc.robot.commands.ZeroHandCommand;
import frc.robot.subsystems.HandSubsystem;

public class RobotContainer {
  private HandSubsystem handSubsystem;

  public RobotContainer() {
    handSubsystem = new HandSubsystem();

    configurePitDashboard();
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void configurePitDashboard() {
    ShuffleboardTab pitTab = Shuffleboard.getTab("Pit");

    // Hand Commands
    ShuffleboardLayout elevatorCommands =
        pitTab.getLayout("Hand", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(1, 6);
    elevatorCommands.add("Hand Stop", new HandSpeedCommand(handSubsystem, 0)).withPosition(0, 0);
    elevatorCommands.add("Hand Zero", new ZeroHandCommand(handSubsystem)).withPosition(0, 1);
    elevatorCommands.add("Hand Close", new HandSpeedCommand(handSubsystem, 0.1)).withPosition(0, 2);
    elevatorCommands.add("Hand Open", new HandSpeedCommand(handSubsystem, -0.1)).withPosition(0, 3);
    elevatorCommands.add("Grab Cube", new GrabCubeCommand(handSubsystem)).withPosition(0, 4);
    elevatorCommands.add("Grab Cone", new GrabConeCommand(handSubsystem)).withPosition(0, 5);
  }
}
