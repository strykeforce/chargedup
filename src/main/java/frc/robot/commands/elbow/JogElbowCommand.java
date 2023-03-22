package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElbowConstants;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class JogElbowCommand extends CommandBase {
  private ElbowSubsystem elbowSubsystem;
  private RobotStateSubsystem robotStateSubsystem;
  private int sign;
  private Logger logger = LoggerFactory.getLogger(JogElbowCommand.class);

  public JogElbowCommand(
      ElbowSubsystem elbowSubsystem, RobotStateSubsystem robotStateSubsystem, int sign) {
    addRequirements(elbowSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.elbowSubsystem = elbowSubsystem;
    this.sign = sign;
  }

  @Override
  public void initialize() {
    logger.info("Moving elbow");
  }

  @Override
  public void execute() {
    elbowSubsystem.setPos(elbowSubsystem.getPos() + ElbowConstants.kJogElbowTicks * sign);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
