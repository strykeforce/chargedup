package frc.robot.commands.RGBlights;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RGBlightsSubsystem;

public class RGBsetPieceCommand extends InstantCommand{
    private RGBlightsSubsystem rgblightsSubsystem;
    private Logger logger = LoggerFactory.getLogger(RGBlightsSubsystem.class);

    public RGBsetPieceCommand(RGBlightsSubsystem rgblightsSubsystem, String CubeOrCone) {
        this.rgblightsSubsystem = rgblightsSubsystem;
        if (CubeOrCone == "Cube") {
            this.rgblightsSubsystem.setCubeColor();
            logger.info("set color to the Color of the {}", CubeOrCone);
        } if (CubeOrCone == "Cone") {
            this.rgblightsSubsystem.setConeColor();
            logger.info("set color to the Color of the {}", CubeOrCone);
        } else {
            logger.info("failed to set Color to the color of the \"{}\"". reason: invalid argument.", CubeOrCone);
        }
    }
}
