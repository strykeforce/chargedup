package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.vision.SetVisionUpdateCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;

public class AutoShowOffCommand extends SequentialCommandGroup implements AutoCommandInterface {
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Invalid;
  private DriveAutonCommand defaultPath;
  private RobotStateSubsystem robotStateSubsystem;

  public AutoShowOffCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      HandSubsystem handSubsystem,
      ArmSubsystem armSubsystem) {
    defaultPath = new DriveAutonCommand(driveSubsystem, "testPath", true, true);
    this.robotStateSubsystem = robotStateSubsystem;

    addCommands(
        new ParallelCommandGroup(new SetVisionUpdateCommand(driveSubsystem, false)),
        defaultPath,
        new SetVisionUpdateCommand(driveSubsystem, true));
  }

  public void generateTrajectory() {
    defaultPath.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
