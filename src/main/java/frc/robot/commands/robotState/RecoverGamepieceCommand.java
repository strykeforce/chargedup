package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RecoverGamepieceCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;
  private HandSubsystem handSubsystem;
  private static final Logger logger = LoggerFactory.getLogger(RecoverGamepieceCommand.class);

  public RecoverGamepieceCommand(
      RobotStateSubsystem robotStateSubsystem, HandSubsystem handSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    logger.info("Going to Recover Gamepiece");
    robotStateSubsystem.toRecoverGamepiece();
  }
}
