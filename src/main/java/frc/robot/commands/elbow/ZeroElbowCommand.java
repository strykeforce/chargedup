package frc.robot.commands.elbow;

import com.ctre.phoenix.CANifier.PWMChannel;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElbowSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ZeroElbowCommand extends InstantCommand {
  private ElbowSubsystem elbowSubsystem;
  private static final Logger logger = LoggerFactory.getLogger(ZeroElbowCommand.class);

  public ZeroElbowCommand(ElbowSubsystem elbowSubsystem) {
    this.elbowSubsystem = elbowSubsystem;
  }

  @Override
  public void initialize() {
    int absolute = elbowSubsystem.getPulseWidthFor(PWMChannel.PWMChannel0);
    logger.info(
        "Elbow at zero, absolute: {}, zero ticks: {}, difference: {}",
        absolute,
        Constants.kElbowZeroTicks,
        Math.abs(absolute - Constants.kElbowZeroTicks));
    elbowSubsystem.zeroElbowStow();
  }
}
