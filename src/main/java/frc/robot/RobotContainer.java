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
import frc.robot.commands.hand.GrabConeCommand;
import frc.robot.commands.hand.GrabCubeCommand;
import frc.robot.commands.hand.HandLeftSpeedCommand;
import frc.robot.commands.hand.HandRightSpeedCommand;
import frc.robot.commands.hand.ZeroHandCommand;
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
    ShuffleboardLayout handCommands =
        pitTab.getLayout("Hand", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(1, 9);
    handCommands
        .add("Hand Left Stop", new HandLeftSpeedCommand(handSubsystem, 0))
        .withPosition(0, 0);
    handCommands
        .add("Hand Right Stop", new HandRightSpeedCommand(handSubsystem, 0))
        .withPosition(0, 1);
    handCommands.add("Hand Zero", new ZeroHandCommand(handSubsystem)).withPosition(0, 2);
    handCommands
        .add("Hand Left In", new HandLeftSpeedCommand(handSubsystem, 0.1))
        .withPosition(0, 3);
    handCommands
        .add("Hand Right In", new HandRightSpeedCommand(handSubsystem, 0.1))
        .withPosition(0, 4);
    handCommands
        .add("Hand Left Out", new HandLeftSpeedCommand(handSubsystem, -0.1))
        .withPosition(0, 5);
    handCommands
        .add("Hand Right Out", new HandRightSpeedCommand(handSubsystem, -0.1))
        .withPosition(0, 6);
    handCommands.add("Grab Cube", new GrabCubeCommand(handSubsystem)).withPosition(0, 7);
    handCommands.add("Grab Cone", new GrabConeCommand(handSubsystem)).withPosition(0, 8);
  }
}
