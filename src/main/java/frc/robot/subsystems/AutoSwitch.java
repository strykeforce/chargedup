package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.auto.AutoCommandInterface;
import frc.robot.commands.auto.DefaultAutoCommand;
import frc.robot.commands.auto.DoNothingAutonCommand;
import frc.robot.commands.auto.MiddleToDock;
import frc.robot.commands.auto.MiddleToDockWithMobility;
import frc.robot.commands.auto.ThreePieceBumpAutoCommandGroup;
import frc.robot.commands.auto.ThreePieceBumpFallbackAutoCommandGroup;
import frc.robot.commands.auto.ThreePieceBumpLowAutoCommandGroup;
import frc.robot.commands.auto.ThreePieceSmoothAutoCommandGroup;
import frc.robot.commands.auto.TwoPieceBumpWithDockAutoCommandGroup;
import frc.robot.commands.auto.TwoPieceLvl3AutoCommandGroup;
import frc.robot.commands.auto.TwoPieceLvl3BumpAutoCommandGroup;
import frc.robot.commands.auto.TwoPieceMiddleBalanceAutoCommandGroup;
import frc.robot.commands.auto.TwoPieceWithDockAutoCommandGroup;
import frc.robot.commands.auto.TwoPieceWithDockAutoMidCommandGroup;
import java.util.ArrayList;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import org.strykeforce.thirdcoast.util.AutonSwitch;

public class AutoSwitch extends MeasurableSubsystem {
  private final AutonSwitch autoSwitch;
  private ArrayList<DigitalInput> switchInputs = new ArrayList<>();
  private int currAutoSwitchPos = -1;
  private int newAutoSwitchPos;
  private int autoSwitchStableCounts = 0;
  private Logger logger = LoggerFactory.getLogger(AutoSwitch.class);
  private static SendableChooser<Integer> sendableChooser = new SendableChooser<>();

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
  AutoCommandInterface defaultCommand;
  private boolean useVirtualSwitch = false;

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

    sendableChooser.addOption("00  Cone Lvl 3, Cube Lvl 3, Auto Balance", 0x00);
    sendableChooser.setDefaultOption("01  Cone Lvl 3, Cube Lvl 3", 0x01);
    sendableChooser.setDefaultOption("02  Same as #1 but scores cone mid", 0x02);
    sendableChooser.setDefaultOption("03  Cone lvl 3, Cube lvl 3, Cube lvl 2", 0x03);
    sendableChooser.setDefaultOption("10  Cone lvl 3, cube lvl 3, balance", 0x10);
    sendableChooser.setDefaultOption("11  Middle to dock", 0x11);
    sendableChooser.setDefaultOption("12  Middle to dock with mobility", 0x12);
    sendableChooser.setDefaultOption("20  Cone Lvl 3, Cube Lvl 3", 0x20);
    sendableChooser.setDefaultOption("21  FALLBACK - Cone Lvl 3, cube lvl 2 and 3", 0x21);
    sendableChooser.setDefaultOption("22  Cone Lvl3, Cube Lvl3", 0x22);
    sendableChooser.setDefaultOption("23  Cone Lvl 3, cube lvl 2 and 3", 0x23);
    sendableChooser.setDefaultOption("24  Cone Lvl 1, Cube Lvl 3 and 2", 0x24);
    sendableChooser.setDefaultOption("30  Do nothing", 0x30);
    SmartDashboard.putData("Auto Mode", sendableChooser);

    defaultCommand =
        new DefaultAutoCommand(
            driveSubsystem, robotStateSubsystem, elevatorSubsystem, handSubsystem, armSubsystem);

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

  public AutoCommandInterface getAutoCommand() {
    if (autoCommand == null) {
      return defaultCommand;
    } else return this.autoCommand;
  }

  private boolean hasSwitchChanged() {
    boolean changed = false;
    int switchPos = useVirtualSwitch ? sendableChooser.getSelected() : autoSwitch.position();

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

  public void toggleVirtualSwitch() {
    logger.info("toggledSwitch:function");
    if (useVirtualSwitch) {
      useVirtualSwitch = false;
    } else {
      useVirtualSwitch = true;
    }
    // useVirtualSwitch = useVirtualSwitch ? false : true;
  }

  public String getSwitchPos() {
    return Integer.toHexString(currAutoSwitchPos);
  }

  public boolean isUseVirtualSwitch() {
    return useVirtualSwitch;
  }

  public SendableChooser<Integer> getSendableChooser() {
    return sendableChooser;
  }

  private AutoCommandInterface getAutoCommand(int switchPos) {
    switch (switchPos) {
        // Non-Bump Side
      case 0x00:
        // Cone Lvl 3, Cube Lvl 3, Auto Balance
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
      case 0x01:
        // Cone Lvl 3, Cube Lvl 3
        return new TwoPieceLvl3AutoCommandGroup(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            "pieceOneFetchPath",
            "pieceOnePlacePath");
      case 0x02:
        // Same as 0x00 but scores cone mid
        return new TwoPieceWithDockAutoMidCommandGroup(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            "pieceOneFetchPath",
            "pieceOnePlacePath",
            "pieceTwoToDockPath");
      case 0x03:
        // Cone lvl 3, Cube lvl 3, Cube lvl 2
        return new ThreePieceSmoothAutoCommandGroup(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            visionSubsystem,
            "pieceOneFetchCubeThreePath",
            "pieceOnePlaceCubeThreePath",
            "pieceTwoFetchCubeFourPath",
            "pieceTwoPlaceCubeFourPath",
            "pieceScoreWithoutAutoPath");
      case 0x10:
        // Cone lvl 3, cube lvl 3, balance
        return new TwoPieceMiddleBalanceAutoCommandGroup(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            "pieceFetchChargeStation",
            "pieceScoreChargeStation",
            "pieceScoreWithoutAutoChargeStation",
            "middleScoreToBalance");

      case 0x11:
        return new MiddleToDock(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            visionSubsystem,
            "middleScoreToBalance");

      case 0x12:
        return new MiddleToDockWithMobility(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            visionSubsystem,
            "piecePlaceOverLinePath",
            "tinyLittleToBalancePath");
        // Bump Side
      case 0x20:
        // Cone Lvl 3, Cube Lvl 3
        return new TwoPieceLvl3BumpAutoCommandGroup(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            "pieceOneFetchPathBump",
            "pieceOneDeliverBumpPathPt1",
            "pieceOneDeliverBumpPathPt2");
      case 0x21:
        // FALLBACK - Cone Lvl 3, cube lvl 2 and 3
        return new ThreePieceBumpFallbackAutoCommandGroup(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            visionSubsystem,
            "pieceOneFetchBumpCubeOneFallback",
            "pieceOneDeliverBumpCubeOneFallback",
            "pieceTwoFetchBumpCubeTwoFallback",
            "pieceTwoDeliverBumpCubeTwoFallback",
            "pieceScoreWithoutAutoBumpFallback");
      case 0x22:
        // Cone Lvl3, Cube Lvl3
        return new TwoPieceBumpWithDockAutoCommandGroup(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            visionSubsystem,
            "pieceOneFetchPathBump",
            "pieceOneDeliverBumpPath",
            "pieceTwoToDockBumpPath",
            "pieceScoreWithoutAutoBump");
      case 0x23:
        // Cone Lvl 3, cube lvl 2 and 3
        return new ThreePieceBumpAutoCommandGroup(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            visionSubsystem,
            "pieceOneFetchBumpCubeTwo",
            "pieceOneDeliverBumpCubeTwo",
            "pieceTwoFetchBumpCubeOne",
            "pieceTwoDeliverBumpCubeOne",
            "pieceScoreWithoutAutoBump");
      case 0x24:
        // Cone Lvl 1, Cube Lvl 3 and 2
        return new ThreePieceBumpLowAutoCommandGroup(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            visionSubsystem,
            "pieceOneFetchBumpCubeTwo",
            "pieceOneDeliverBumpCubeTwo",
            "pieceTwoFetchBumpCubeOne",
            "pieceTwoDeliverBumpCubeOne",
            "pieceScoreWithoutAutoBump");
      case 0x30:
        return new DoNothingAutonCommand(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem);
      default:
        String msg = String.format("no auto command assigned for switch pos: %02X", switchPos);
        DriverStation.reportWarning(msg, false);
        return new DefaultAutoCommand(
            driveSubsystem, robotStateSubsystem, elevatorSubsystem, handSubsystem, armSubsystem);
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("AutoSwitch", () -> currAutoSwitchPos),
        new Measure("usingVirtualSwitch", () -> this.useVirtualSwitch ? 1.0 : 0.0));
  }
}
