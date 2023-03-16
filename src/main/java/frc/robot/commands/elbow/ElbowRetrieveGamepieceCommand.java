package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class ElbowRetrieveGamepieceCommand extends InstantCommand {
  private ElbowSubsystem elbowSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private double percentOutput;

  public ElbowRetrieveGamepieceCommand(
      ElbowSubsystem elbowSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      double percentOutput) {
    addRequirements(elbowSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.elbowSubsystem = elbowSubsystem;
    this.percentOutput = percentOutput;
  }

  @Override
  public void initialize() {
    if (robotStateSubsystem.getRobotState() == RobotState.RETRIEVE_GAMEPIECE) {
      elbowSubsystem.rotateOpenLoop(percentOutput);
    }
  }
}
