package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;

public class GrabConeCommand extends CommandBase {
  private HandSubsystem handSubsystem;
  private RobotStateSubsystem robotStateSubsystem;

  public GrabConeCommand(
      HandSubsystem handSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem) {
    this.handSubsystem = handSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    addRequirements(handSubsystem, armSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toGrabGamepiece(GamePiece.CONE);
  }

  @Override
  public boolean isFinished() {
    return handSubsystem.isFinished();
  }
}
