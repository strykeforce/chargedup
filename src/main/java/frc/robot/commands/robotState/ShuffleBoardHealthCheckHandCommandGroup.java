package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LockZeroCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HandSubsystem;
import org.strykeforce.healthcheck.HealthCheckCommand;

public class ShuffleBoardHealthCheckHandCommandGroup extends SequentialCommandGroup {
  public ShuffleBoardHealthCheckHandCommandGroup(
      HandSubsystem handSubsystem, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
    addCommands(
        new toggleHealthBoolean(armSubsystem),
        new HealthCheckCommand(
            handSubsystem // ,
            // handSubsystem,
            // shoulderSubsystem,
            // intakeSubsystem,
            // elbowSubsystem,
            // elevatorSubsystem
            ),
        new LockZeroCommand(driveSubsystem),
        new toggleHealthBoolean(armSubsystem));
  }
}
