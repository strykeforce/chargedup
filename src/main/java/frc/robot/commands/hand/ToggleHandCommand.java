package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HandSubsystem.HandStates;

public class ToggleHandCommand extends InstantCommand{
    private HandSubsystem handSubsystem;

    public ToggleHandCommand(HandSubsystem handSubsystem) {
        this.handSubsystem = handSubsystem;
    }

    @Override
    public void initialize() {
        if (handSubsystem.getHandState() == HandStates.OPEN) {
            handSubsystem.grabCone();
        }
        else if (handSubsystem.getHandState() == HandStates.CUBE_CLOSED || handSubsystem.getHandState() == HandStates.CONE_CLOSED) {
            handSubsystem.open();
        }
    }
}
