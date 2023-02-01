package frc.robot.commands.RGBlights;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RGBlightsSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RGBsetPieceCommand extends InstantCommand {
  private RGBlightsSubsystem rgblightsSubsystem;
  private Logger logger = LoggerFactory.getLogger(RGBlightsSubsystem.class);

  public RGBsetPieceCommand(RGBlightsSubsystem rgblightsSubsystem, GamePiece gamePiece) {
    this.rgblightsSubsystem = rgblightsSubsystem;
    if (gamePiece == GamePiece.CUBE) {
      this.rgblightsSubsystem.setCubeColor();
      logger.info("set color to the Color of the {}", gamePiece);
    }
    if (gamePiece == GamePiece.CONE) {
      this.rgblightsSubsystem.setConeColor();
      logger.info("set color to the Color of the {}", gamePiece);
    } else {
      logger.info(
          "failed to set Color to the color of the \"{}\". reason: invalid argument.", gamePiece);
    }
  }
}
