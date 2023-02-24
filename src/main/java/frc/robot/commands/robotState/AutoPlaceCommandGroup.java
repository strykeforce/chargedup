package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;

public class AutoPlaceCommandGroup extends SequentialCommandGroup {
  public AutoPlaceCommandGroup(
      DriveSubsystem driveSubsystem, RobotStateSubsystem robotStateSubsystem) {
    driveSubsystem.setAutoDriving(true);
    addCommands(new WaitCommand(0.25), new AutoPlaceCommand(driveSubsystem, robotStateSubsystem));
    addRequirements(robotStateSubsystem, driveSubsystem);
  }
}
