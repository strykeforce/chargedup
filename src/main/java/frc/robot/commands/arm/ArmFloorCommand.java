package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.RobotState;

public class ArmFloorCommand extends CommandBase {
  private ArmSubsystem armSubsystem;
  private RobotStateSubsystem robotStateSubsystem;

  public ArmFloorCommand(ArmSubsystem armSubsystem, RobotStateSubsystem robotStateSubsystem) {
    this.armSubsystem = armSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.toFloorPos();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getRobotState() == RobotState.FLOOR_PICKUP;
  }
}
