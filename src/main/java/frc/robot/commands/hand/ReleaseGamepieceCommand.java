package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;

public class ReleaseGamepieceCommand extends CommandBase {
  private HandSubsystem handSubsystem;
  private RobotStateSubsystem robotStateSubsystem;

  public ReleaseGamepieceCommand(
      HandSubsystem handSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem) {
    this.handSubsystem = handSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    addRequirements(handSubsystem, armSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toReleaseGamepiece();
  }

  @Override
  public boolean isFinished() {
    return handSubsystem.isFinished();
  }
}
