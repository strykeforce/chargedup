package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LockZeroCommand;
import frc.robot.commands.elbow.ElbowToPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import org.strykeforce.healthcheck.HealthCheckCommand;

public class ShuffleBoardHealthCheckElbowCommandGroup extends SequentialCommandGroup {
  public ShuffleBoardHealthCheckElbowCommandGroup(
      ElbowSubsystem elbowSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      DriveSubsystem driveSubsystem,
      ArmSubsystem armSubsystem) {
    addCommands(
        new toggleHealthBoolean(armSubsystem),
        new HealthCheckCommand(
            elbowSubsystem, elevatorSubsystem // ,
            // handSubsystem,
            // shoulderSubsystem,
            // intakeSubsystem,
            // elbowSubsystem,
            // elevatorSubsystem
            ),
        new LockZeroCommand(driveSubsystem),
        new ElbowToPositionCommand(elbowSubsystem, 0.0),
        new toggleHealthBoolean(armSubsystem));
  }
}
