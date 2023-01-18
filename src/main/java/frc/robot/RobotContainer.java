// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;

public class RobotContainer {
  private RobotStateSubsystem robotStateSubsystem;

  public RobotContainer() {
    robotStateSubsystem = new RobotStateSubsystem(TargetLevel.NONE, TargetCol.NONE, GamePiece.NONE);

    configurePitDashboard();
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void configurePitDashboard() {
    ShuffleboardTab pitTab = Shuffleboard.getTab("Pit");
  }
}
