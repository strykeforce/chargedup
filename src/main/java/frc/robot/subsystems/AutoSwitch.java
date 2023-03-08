package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.auto.AutoCommandInterface;
import frc.robot.commands.auto.DefaultAutoCommand;
import frc.robot.commands.auto.TwoPieceWithDockAutoCommandGroup;
import java.util.ArrayList;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.util.AutonSwitch;

public class AutoSwitch {
  private final AutonSwitch autoSwitch;
  private ArrayList<DigitalInput> switchInputs = new ArrayList<>();
  private int currAutoSwitchPos = -1;
  private int newAutoSwitchPos;
  private int autoSwitchStableCounts = 0;
  private Logger logger = LoggerFactory.getLogger(AutoSwitch.class);

  private AutoCommandInterface autoCommand;
  private final RobotStateSubsystem robotStateSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ShoulderSubsystem shoulderSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final ElbowSubsystem elbowSubsystem;
  private final HandSubsystem handSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final RGBlightsSubsystem rgBlightsSubsystem;

  public AutoSwitch(
      RobotStateSubsystem robotStateSubsystem,
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem,
      ShoulderSubsystem shoulderSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ElbowSubsystem elbowSubsystem,
      HandSubsystem handSubsystem,
      VisionSubsystem visionSubsystem,
      RGBlightsSubsystem rgBlightsSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
    this.shoulderSubsystem = shoulderSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.elbowSubsystem = elbowSubsystem;
    this.handSubsystem = handSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.rgBlightsSubsystem = rgBlightsSubsystem;

    for (int i = AutonConstants.kStartSwitchID; i <= AutonConstants.kEndSwitchId; i++) {
      switchInputs.add(new DigitalInput(i));
    }
    autoSwitch = new AutonSwitch(switchInputs);
  }

  public void checkSwitch() {
    if (hasSwitchChanged()) {
      logger.info(
          "Initializing Auto Switch Position: {}", String.format("%02X", currAutoSwitchPos));
      autoCommand = getAutoCommand(currAutoSwitchPos);
      if (!autoCommand.hasGenerated()) autoCommand.generateTrajectory();
    }
  }

  public void resetSwitchPos() {
    if (currAutoSwitchPos == -1) {
      logger.info("Reset Auto Switch");
    }
    currAutoSwitchPos = -1;
  }

  public AutoCommandInterface getAutCommand() {
    return autoCommand;
  }

  private boolean hasSwitchChanged() {
    boolean changed = false;
    int switchPos = autoSwitch.position();

    if (switchPos != newAutoSwitchPos) {
      autoSwitchStableCounts = 0;
      newAutoSwitchPos = switchPos;
    } else autoSwitchStableCounts++;

    if (autoSwitchStableCounts > AutonConstants.kSwitchStableCounts
        && currAutoSwitchPos != newAutoSwitchPos) {
      changed = true;
      currAutoSwitchPos = newAutoSwitchPos;
    }

    return changed;
  }

  private AutoCommandInterface getAutoCommand(int switchPos) {
    switch (switchPos) {
      case 0x30:
        return new TwoPieceWithDockAutoCommandGroup(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            "pieceOneFetchPath",
            "pieceOnePlacePath",
            "pieceTwoToDockPath");
      default:
        String msg = String.format("no auto command assigned for switch pos: %02X", switchPos);
        DriverStation.reportWarning(msg, false);
        return new DefaultAutoCommand(
            driveSubsystem, robotStateSubsystem, elevatorSubsystem, handSubsystem, armSubsystem);
    }
  }
}
