package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;

public class RetrieveGamePieceCommand extends InstantCommand {
  private ArmSubsystem armSubsystem;
  private HandSubsystem handSubsystem;
  private RobotStateSubsystem robotStateSubsystem;

  public RetrieveGamePieceCommand(
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem,
      RobotStateSubsystem robotStateSubsystem) {
    addRequirements(handSubsystem, armSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.handSubsystem = handSubsystem;
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toRetrieveGamepiece();
  }
}
