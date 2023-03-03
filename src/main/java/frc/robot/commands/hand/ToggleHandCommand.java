package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HandSubsystem.HandStates;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;

public class ToggleHandCommand extends CommandBase {
  private HandSubsystem handSubsystem;
  private RobotStateSubsystem robotStateSubsystem;

  public ToggleHandCommand(
      HandSubsystem handSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem) {
    this.handSubsystem = handSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    addRequirements(handSubsystem, armSubsystem);
  }

  @Override
  public void initialize() {
    if (handSubsystem.getHandState() == HandStates.OPEN) {
      robotStateSubsystem.toGrabGamepiece(GamePiece.CONE);
    } else if (handSubsystem.getHandState() == HandStates.CUBE_CLOSED
        || handSubsystem.getHandState() == HandStates.CONE_CLOSED) {
      robotStateSubsystem.toReleaseGamepiece();
    }
  }

  @Override
  public boolean isFinished() {
      return handSubsystem.isFinished();
  }
}
