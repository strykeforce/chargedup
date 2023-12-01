package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LockZeroCommand;
import frc.robot.commands.elbow.ElbowToPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import org.strykeforce.healthcheck.HealthCheckCommand;
import org.strykeforce.healthcheck.internal.ResetCaseNum;

public class ShuffleBoardHealthCheckCommandGroup extends SequentialCommandGroup {
  public ShuffleBoardHealthCheckCommandGroup(
      ElbowSubsystem elbowSubsystem,
      ShoulderSubsystem shoulderSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      HandSubsystem handSubsystem,
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem) {
    addCommands(
      new toggleHealthBoolean(armSubsystem),
      
      new ResetCaseNum().resetCaseId()
        new HealthCheckCommand(
            driveSubsystem
            // handSubsystem,
            // intakeSubsystem,
            // shoulderSubsystem,
            // elbowSubsystem,
            // elevatorSubsystem
            ),
        new LockZeroCommand(driveSubsystem),
        new ElbowToPositionCommand(elbowSubsystem, 0.0),
        new toggleHealthBoolean(armSubsystem));
  }
}
