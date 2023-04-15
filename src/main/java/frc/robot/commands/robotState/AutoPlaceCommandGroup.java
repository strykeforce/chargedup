package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.setAutoDrivingCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;

public class AutoPlaceCommandGroup extends SequentialCommandGroup {
  public AutoPlaceCommandGroup(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem) {
    addCommands(
        new setAutoDrivingCommand(driveSubsystem, true),
        new WaitCommand(0.3),
        new AutoPlaceCommand(
            driveSubsystem, robotStateSubsystem, armSubsystem, handSubsystem, false));
    addRequirements(robotStateSubsystem, driveSubsystem);
  }
}
