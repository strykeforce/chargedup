package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.HandConstants;
import frc.robot.subsystems.HandSubsystem;

public class AutoGrabConeCommand extends InstantCommand {
  private HandSubsystem handSubsystem;

  public AutoGrabConeCommand(HandSubsystem handSubsystem) {
    this.handSubsystem = handSubsystem;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.runRollers(HandConstants.kRollerConeHoldSpeed);
    handSubsystem.setLeftPos(HandConstants.kConeGrabbingPosition);
  }
  ;
}
