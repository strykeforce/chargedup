package frc.robot.commands.robot_state;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;

public class SetGamePieceCommand extends InstantCommand {
    private RobotStateSubsystem robotStateSubsystem;
    private GamePiece gamePiece;

    public SetGamePieceCommand(RobotStateSubsystem robotStateSubsystem, GamePiece gamePiece) {
        this.robotStateSubsystem = robotStateSubsystem;
        this.gamePiece = gamePiece;
    }

    @Override
    public void initialize() {
        robotStateSubsystem.setGamePiece(gamePiece);
    }
}
