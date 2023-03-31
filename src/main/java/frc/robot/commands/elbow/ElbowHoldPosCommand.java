package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class ElbowHoldPosCommand extends CommandBase {
  private ElbowSubsystem elbowSubsystem;
  private RobotStateSubsystem robotStateSubsystem;

  public ElbowHoldPosCommand(ElbowSubsystem elbowSubsystem, RobotStateSubsystem robotStateSubsystem) {
    addRequirements(elbowSubsystem);
    this.elbowSubsystem = elbowSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    if (robotStateSubsystem.getRobotState() != RobotState.RELEASE_GAME_PIECE) {
      elbowSubsystem.setPos(elbowSubsystem.getPos());
    }
  }

  @Override
  public boolean isFinished() {
    return elbowSubsystem.isFinished();
  }
}
