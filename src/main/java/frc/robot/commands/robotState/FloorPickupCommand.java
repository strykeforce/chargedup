package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class FloorPickupCommand extends CommandBase {
  private RobotStateSubsystem robotStateSubsystem;

  public FloorPickupCommand(ArmSubsystem armSubsystem, RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toFloorPickup();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getRobotState() == RobotState.FLOOR_GRAB_CONE;
  }
}
