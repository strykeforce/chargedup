package frc.robot.commands.RGBlights;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RGBlightsSubsystem;

public class RGBsetColorCommand extends InstantCommand{
    private RGBlightsSubsystem rgblightsSubsystem;

    public RGBsetColorCommand(RGBlightsSubsystem rgblightsSubsystem, int red, int green, int blue) {
        this.rgblightsSubsystem = rgblightsSubsystem;
        this.rgblightsSubsystem.setColor(red, green, blue);
    }
}
