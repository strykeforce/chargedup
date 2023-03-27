package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LockZeroCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import org.strykeforce.healthcheck.HealthCheckCommand;

public class ShuffleBoardHealthCheckIntakeCommandGroup extends SequentialCommandGroup {
  public ShuffleBoardHealthCheckIntakeCommandGroup(
      IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
    addCommands(
        new toggleHealthBoolean(armSubsystem),
        new HealthCheckCommand(
            intakeSubsystem // ,
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
