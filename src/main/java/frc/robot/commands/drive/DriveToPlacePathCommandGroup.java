package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;

public class DriveToPlacePathCommandGroup extends SequentialCommandGroup {
  public DriveToPlacePathCommandGroup(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      boolean isShelf,
      TargetCol targetCol,
      boolean isBlue) {
    driveSubsystem.setAutoDriving(true);
    addCommands(
        new WaitCommand(0.25),
        new DriveToPlaceCommand(driveSubsystem, robotStateSubsystem, isShelf, targetCol, isBlue));
    addRequirements(robotStateSubsystem, driveSubsystem);
  }
}
