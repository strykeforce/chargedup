package frc.robot;

import ch.qos.logback.classic.util.ContextInitializer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HandConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.auto.AutoCommandInterface;
import frc.robot.commands.auto.TestBalanceCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.InterruptDriveCommand;
import frc.robot.commands.drive.LockZeroCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.drive.xLockCommand;
import frc.robot.commands.elbow.ElbowHoldPosCommand;
import frc.robot.commands.elbow.JogElbowCommand;
import frc.robot.commands.elbow.ZeroElbowCommand;
import frc.robot.commands.elevator.AdjustElevatorCommand;
import frc.robot.commands.elevator.ElevatorSpeedCommand;
import frc.robot.commands.elevator.HoldPositionCommand;
import frc.robot.commands.elevator.ZeroElevatorCommand;
import frc.robot.commands.hand.GrabConeCommand;
import frc.robot.commands.hand.GrabCubeCommand;
import frc.robot.commands.hand.HandLeftSpeedCommand;
import frc.robot.commands.hand.HandLeftToPositionCommand;
import frc.robot.commands.hand.HandToPositionCommand;
import frc.robot.commands.hand.ReleaseGamepieceCommand;
import frc.robot.commands.hand.ToggleHandCommand;
import frc.robot.commands.hand.ZeroHandCommand;
import frc.robot.commands.intake.IntakeExtendCommand;
import frc.robot.commands.intake.IntakeOpenLoopCommand;
import frc.robot.commands.robotState.AutoPlaceCommand;
import frc.robot.commands.robotState.FloorPickupCommand;
import frc.robot.commands.robotState.ManualScoreCommand;
import frc.robot.commands.robotState.RecoverGamepieceCommand;
import frc.robot.commands.robotState.RetrieveGamePieceCommand;
import frc.robot.commands.robotState.SetGamePieceCommand;
import frc.robot.commands.robotState.SetLevelAndColCommandGroup;
import frc.robot.commands.robotState.ShelfPickupCommand;
import frc.robot.commands.robotState.ShuffleBoardHealthCheckCommandGroup;
import frc.robot.commands.robotState.ShuffleBoardHealthCheckElbowCommandGroup;
import frc.robot.commands.robotState.ShuffleBoardHealthCheckHandCommandGroup;
import frc.robot.commands.robotState.ShuffleBoardHealthCheckIntakeCommandGroup;
import frc.robot.commands.robotState.StowRobotCommand;
import frc.robot.commands.robotState.ToggleIntakeCommand;
import frc.robot.commands.robotState.ToggleVirtualSwitchCommand;
import frc.robot.commands.shoulder.ShoulderSpeedCommand;
import frc.robot.commands.shoulder.StopTwistCommand;
import frc.robot.commands.shoulder.TwistShoulderCommand;
import frc.robot.commands.shoulder.ZeroShoulderCommand;
import frc.robot.commands.vision.ToggleUpdateWithVisionCommand;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.controllers.InterlinkJoystick;
import frc.robot.controllers.InterlinkJoystick.InterlinkButton;
import frc.robot.controllers.InterlinkJoystick.Shoulder;
import frc.robot.controllers.InterlinkJoystick.Trim;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.AutoSwitch;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveStates;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RGBlightsSubsystem;
import frc.robot.subsystems.RobotStateSubsystem;
import frc.robot.subsystems.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.RobotStateSubsystem.TargetCol;
import frc.robot.subsystems.RobotStateSubsystem.TargetLevel;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.Map;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.healthcheck.HealthCheckCommand;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {
  private final ShoulderSubsystem shoulderSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private final DriveSubsystem driveSubsystem;
  // private final VisionSubsystem visionSubsystem;
  private final ElbowSubsystem elbowSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ArmSubsystem armSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final RGBlightsSubsystem rgbLightsSubsystem;
  private final Constants constants;
  private final AutoSwitch autoSwitch;

  private final XboxController xboxController = new XboxController(1);
  private final Joystick driveJoystick = new Joystick(0);
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  private static final double kJoystickDeadband = Constants.kJoystickDeadband;

  // Dashboard Widgets
  private SuppliedValueWidget<Boolean> allianceColor;
  private Alliance alliance = Alliance.Invalid;
  private SuppliedValueWidget<Boolean> currGamePiece;
  private boolean isEvent = false;

  private TestBalanceCommand balancepath;
  private DriveAutonCommand fiveMeterTest;
  private HandSubsystem handSubsystem;
  private Logger logger;

  public RobotContainer() {
    DigitalInput eventFlag = new DigitalInput(10);
    boolean isEvent = eventFlag.get();
    this.isEvent = isEvent;
    if (isEvent && Constants.isCompBot) {
      // must be set before the first call to  LoggerFactory.getLogger();
      System.setProperty(ContextInitializer.CONFIG_FILE_PROPERTY, "logback-event.xml");
      System.out.println("Event Flag Removed - logging to file in ~lvuser/logs/");
    }
    logger = LoggerFactory.getLogger(this.getClass());

    constants = new Constants();
    handSubsystem = new HandSubsystem(constants);
    intakeSubsystem = new IntakeSubsystem(constants);
    driveSubsystem = new DriveSubsystem(constants);
    visionSubsystem = new VisionSubsystem(driveSubsystem);
    // visionSubsystem = new VisionSubsystem(driveSubsystem);
    elevatorSubsystem = new ElevatorSubsystem();
    elbowSubsystem = new ElbowSubsystem(constants);
    shoulderSubsystem = new ShoulderSubsystem(constants);
    armSubsystem =
        new ArmSubsystem(shoulderSubsystem, elevatorSubsystem, elbowSubsystem, xboxController);
    rgbLightsSubsystem = new RGBlightsSubsystem();
    robotStateSubsystem =
        new RobotStateSubsystem(
            intakeSubsystem,
            armSubsystem,
            handSubsystem,
            driveSubsystem,
            visionSubsystem,
            rgbLightsSubsystem,
            elbowSubsystem);

    autoSwitch =
        new AutoSwitch(
            robotStateSubsystem,
            driveSubsystem,
            intakeSubsystem,
            armSubsystem,
            shoulderSubsystem,
            elevatorSubsystem,
            elbowSubsystem,
            handSubsystem,
            visionSubsystem,
            rgbLightsSubsystem);

    driveSubsystem.setRobotStateSubsystem(robotStateSubsystem);
    visionSubsystem.setRobotStateSubsystem(robotStateSubsystem);

    driveSubsystem.setVisionSubsystem(visionSubsystem);
    // visionSubsystem.setFillBuffers(true); // FIXME TRUE

    configurePaths();
    // configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configureMatchDashboard();
    if (!isEvent || !Constants.isCompBot) {
      armSubsystem.setTwistEnd(true);
      configurePitImportantDashboard();
      configureTelemetry();
      //   configurePitDashboard();
      new Trigger(RobotController::getUserButton)
          .onTrue(new HealthCheckCommand(driveSubsystem, intakeSubsystem));
    }
  }

  public void configureMotionMagic(boolean isAuto) {
    shoulderSubsystem.setMotionMagic(isAuto);
    elbowSubsystem.setMotionMagic(isAuto);
  }

  public void setVisionEnabled(boolean isEnabled) {
    driveSubsystem.visionUpdates = isEnabled;
  }

  public void autoStowTele() {
    if (elevatorSubsystem.hasZeroed() && isEvent) {
      // robotStateSubsystem.toStow();
    }
  }

  public void driveSubsystemTele() {
    driveSubsystem.setDriveState(DriveStates.IDLE);
  }

  public void setDisabled(boolean isDisabled) {
    robotStateSubsystem.setDisabled(isDisabled);
  }

  public void setAuto(boolean isAuto) {
    robotStateSubsystem.setAutoMode(isAuto);
  }

  public AutoSwitch getAutoSwitch() {
    return autoSwitch;
  }

  public AutoCommandInterface getAutoCommand() {
    return autoSwitch.getAutoCommand();
  }

  public void checkCameraOnline() {
    if (visionSubsystem.isCameraWorking()) {
      rgbLightsSubsystem.setColor(0.0, 0.0, 0.0);
    } else {
      rgbLightsSubsystem.setColor(1.0, 0.0, 0.0);
    }
  }

  public void zeroElevator() {
    if (!elevatorSubsystem.hasZeroed()) elevatorSubsystem.zeroElevator();
  }

  public void raiseServo() {
    visionSubsystem.raiseServo();
  }

  private void configureTelemetry() {
    driveSubsystem.registerWith(telemetryService);
    visionSubsystem.registerWith(telemetryService);
    robotStateSubsystem.registerWith(telemetryService);
    elevatorSubsystem.registerWith(telemetryService);
    elbowSubsystem.registerWith(telemetryService);
    shoulderSubsystem.registerWith(telemetryService);
    armSubsystem.registerWith(telemetryService);
    handSubsystem.registerWith(telemetryService);
    intakeSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }

  // Path Configuration For Robot Container
  private void configurePaths() {
    fiveMeterTest = new DriveAutonCommand(driveSubsystem, "straightPathX", true, true);
    balancepath =
        new TestBalanceCommand(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            "TestAutoDrivePathTwo",
            "TestAutoDrivePathTwo");
  }

  public boolean configureDriverButtonBindings() {
    String joystick = DriverStation.getJoystickName(0);
    boolean success = false;
    switch (joystick) {
      case "InterLink-X":
        logger.info("Configuring Interlink Joystick");
        configureInterlinkDriverButtonBindings();
        success = true;
        break;
      case "FlySky NV14 Joystick":
        logger.info("Configuring Flysky Joystick");
        configureFlyskyDriverButtonBindings();
        success = true;
        break;
      default:
        // logger.info("No joystick type {} defined", joystick);
        break;
    }
    return success;
  }

  private void configureInterlinkDriverButtonBindings() {
    InterlinkJoystick interlink = new InterlinkJoystick(driveJoystick);
    // Drive
    driveSubsystem.setDefaultCommand(
        new DriveTeleopCommand(
            () -> interlink.getFwd(),
            () -> interlink.getStr(),
            () -> interlink.getYaw(),
            driveSubsystem,
            robotStateSubsystem,
            handSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.RESET.id)
        .onTrue(new ZeroGyroCommand(driveSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.X.id)
        .onTrue(new xLockCommand(driveSubsystem));

    // Zero/Stow
    new JoystickButton(driveJoystick, InterlinkButton.HAMBURGER.id)
        .onTrue(new ZeroElbowCommand(elbowSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.UP.id)
        .onTrue(new ZeroElevatorCommand(elevatorSubsystem));
    new JoystickButton(driveJoystick, InterlinkButton.DOWN.id)
        .onTrue(
            new StowRobotCommand(
                robotStateSubsystem, armSubsystem, intakeSubsystem, handSubsystem));

    // Release Gamepiece
    new JoystickButton(driveJoystick, Shoulder.LEFT_DOWN.id)
        .onTrue(new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem, armSubsystem))
        .onFalse(new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem, armSubsystem));
    new JoystickButton(driveJoystick, Shoulder.LEFT_UP.id)
        .onTrue(new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem, armSubsystem))
        .onFalse(new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem, armSubsystem));

    // Intake
    new JoystickButton(driveJoystick, Shoulder.RIGHT_DOWN.id)
        .onTrue(new ToggleIntakeCommand(robotStateSubsystem, intakeSubsystem))
        .onFalse(new ToggleIntakeCommand(robotStateSubsystem, intakeSubsystem));

    // Manual Score
    new JoystickButton(driveJoystick, Trim.RIGHT_Y_POS.id)
        .onTrue(new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem));
    new JoystickButton(driveJoystick, Trim.RIGHT_Y_NEG.id)
        .onTrue(new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem));

    // Auto Score
    new JoystickButton(driveJoystick, Trim.RIGHT_X_POS.id)
        .onTrue(
            new AutoPlaceCommand(
                driveSubsystem, robotStateSubsystem, armSubsystem, handSubsystem, false, 0.0))
        .onFalse(new InterruptDriveCommand(driveSubsystem));

    // Auto Corridor Movement Testing
    new JoystickButton()
  }

  private void configureFlyskyDriverButtonBindings() {
    FlyskyJoystick flysky = new FlyskyJoystick(driveJoystick);

    // Drive
    driveSubsystem.setDefaultCommand(
        new DriveTeleopCommand(
            () -> flysky.getFwd(),
            () -> flysky.getStr(),
            () -> flysky.getYaw(),
            driveSubsystem,
            robotStateSubsystem,
            handSubsystem));
    new JoystickButton(driveJoystick, Button.M_LTRIM_UP.id)
        .onTrue(new ZeroGyroCommand(driveSubsystem));
    new JoystickButton(driveJoystick, Button.M_RTRIM_UP.id)
        .onTrue(new xLockCommand(driveSubsystem));

    // Zero/Stow
    new JoystickButton(driveJoystick, Button.M_RTRIM_DWN.id)
        .onTrue(new ZeroElevatorCommand(elevatorSubsystem));
    new JoystickButton(driveJoystick, Button.SWD.id)
        .onTrue(
            new StowRobotCommand(robotStateSubsystem, armSubsystem, intakeSubsystem, handSubsystem))
        .onFalse(
            new StowRobotCommand(
                robotStateSubsystem, armSubsystem, intakeSubsystem, handSubsystem));

    // Release Gamepiece
    new JoystickButton(driveJoystick, Button.M_SWH.id)
        .onTrue(new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem, armSubsystem));
    // .onFalse(new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem, armSubsystem));
    // new JoystickButton(driveJoystick, Button.SWF_DWN.id)
    //     .onTrue(new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem, armSubsystem))
    //     .onFalse(new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem, armSubsystem));

    // Intake
    new JoystickButton(driveJoystick, Button.SWB_UP.id)
        .onTrue(new ToggleIntakeCommand(robotStateSubsystem, intakeSubsystem))
        .onFalse(new ToggleIntakeCommand(robotStateSubsystem, intakeSubsystem));
    new JoystickButton(driveJoystick, Button.SWB_DWN.id)
        .onTrue(new ToggleIntakeCommand(robotStateSubsystem, intakeSubsystem))
        .onFalse(new ToggleIntakeCommand(robotStateSubsystem, intakeSubsystem));

    // Manual Score
    new JoystickButton(driveJoystick, Button.M_SWE.id)
        .onTrue(new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem));
    // .onFalse(new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem));
    // new JoystickButton(driveJoystick, Button.SWG_DWN.id)
    //     .onTrue(new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem))
    //     .onFalse(new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem));

    // Auto Score
    new JoystickButton(driveJoystick, Button.M_SWC.id)
        .onTrue(
            new AutoPlaceCommand(
                driveSubsystem,
                robotStateSubsystem,
                armSubsystem,
                handSubsystem,
                isEvent,
                kJoystickDeadband))
        .onFalse(new InterruptDriveCommand(driveSubsystem));
  }

  public void configureOperatorButtonBindings() {
    // Rescue Trapped Piece
    new JoystickButton(xboxController, XboxController.Button.kRightStick.value)
        .onTrue(new RetrieveGamePieceCommand(armSubsystem, handSubsystem, robotStateSubsystem));
    new JoystickButton(xboxController, XboxController.Button.kStart.value)
        .onTrue(new RecoverGamepieceCommand(robotStateSubsystem, handSubsystem));

    // Stow
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
        .onTrue(
            new StowRobotCommand(
                robotStateSubsystem, armSubsystem, intakeSubsystem, handSubsystem));

    // Set Level/Col
    // Level 3
    Trigger leftTrigger =
        new Trigger(() -> xboxController.getLeftTriggerAxis() >= 0.1)
            .onTrue(
                new SetLevelAndColCommandGroup(
                    robotStateSubsystem, TargetLevel.HIGH, TargetCol.LEFT));
    Trigger rightTrigger =
        new Trigger(() -> xboxController.getRightTriggerAxis() >= 0.1)
            .onTrue(
                new SetLevelAndColCommandGroup(
                    robotStateSubsystem, TargetLevel.HIGH, TargetCol.RIGHT));
    // Level 2
    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
        .onTrue(
            new SetLevelAndColCommandGroup(robotStateSubsystem, TargetLevel.MID, TargetCol.LEFT));
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
        .onTrue(
            new SetLevelAndColCommandGroup(robotStateSubsystem, TargetLevel.MID, TargetCol.RIGHT));
    // Level 1
    Trigger floorPlace = new Trigger(() -> xboxController.getPOV() == 0);
    floorPlace.onTrue(
        new SetLevelAndColCommandGroup(robotStateSubsystem, TargetLevel.LOW, TargetCol.MID));

    // Hand
    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(new ToggleHandCommand(handSubsystem, robotStateSubsystem, armSubsystem));

    // Intake
    new JoystickButton(xboxController, XboxController.Button.kX.value)
        .onTrue(new ToggleIntakeCommand(robotStateSubsystem, intakeSubsystem));

    // Shelf
    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .onTrue(new ShelfPickupCommand(robotStateSubsystem));

    // Floor Cone
    Trigger dPadPressed = new Trigger(() -> xboxController.getPOV() == 180);
    dPadPressed.onTrue(new FloorPickupCommand(armSubsystem, robotStateSubsystem));

    // Clear Gamepiece
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(new SetGamePieceCommand(robotStateSubsystem, GamePiece.NONE));

    // Manual Adjust

    // Adjust elevator
    Trigger leftUp =
        new Trigger(() -> xboxController.getLeftY() <= -0.1)
            .onTrue(new AdjustElevatorCommand(elevatorSubsystem, robotStateSubsystem, -1000))
            .onFalse(new HoldPositionCommand(elevatorSubsystem, robotStateSubsystem));
    Trigger leftDown =
        new Trigger(() -> xboxController.getLeftY() >= 0.1)
            .onTrue(new AdjustElevatorCommand(elevatorSubsystem, robotStateSubsystem, 1000))
            .onFalse(new HoldPositionCommand(elevatorSubsystem, robotStateSubsystem));

    // Adjust Elbow
    Trigger rightDown =
        new Trigger(() -> xboxController.getRightY() <= -0.1)
            .onTrue(new JogElbowCommand(elbowSubsystem, robotStateSubsystem, 1))
            .onFalse(new ElbowHoldPosCommand(elbowSubsystem, robotStateSubsystem));
    Trigger rightUp =
        new Trigger(() -> xboxController.getRightY() >= 0.1)
            .onTrue(new JogElbowCommand(elbowSubsystem, robotStateSubsystem, -1))
            .onFalse(new ElbowHoldPosCommand(elbowSubsystem, robotStateSubsystem));

    // Twit Shoudler
    Trigger rightRight =
        new Trigger(() -> xboxController.getRightX() <= -0.1)
            .onTrue(new TwistShoulderCommand(shoulderSubsystem, armSubsystem, -1))
            .onFalse(new StopTwistCommand(shoulderSubsystem, armSubsystem));
    Trigger rightLeft =
        new Trigger(() -> xboxController.getRightX() >= 0.1)
            .onTrue(new TwistShoulderCommand(shoulderSubsystem, armSubsystem, 1))
            .onFalse(new StopTwistCommand(shoulderSubsystem, armSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void configureMatchDashboard() {
    allianceColor =
        Shuffleboard.getTab("Match")
            .addBoolean("AllianceColor", () -> alliance != Alliance.Invalid)
            .withProperties(Map.of("colorWhenFalse", "black"))
            .withSize(2, 2)
            .withPosition(0, 0);
    currGamePiece =
        Shuffleboard.getTab("Match")
            .addBoolean("Game Piece", () -> robotStateSubsystem.getGamePiece() != GamePiece.NONE)
            .withProperties(Map.of("colorWhenFalse", "black"))
            .withSize(2, 2)
            .withPosition(5, 0);

    Shuffleboard.getTab("Match")
        .add("Update With Vision", new ToggleUpdateWithVisionCommand(driveSubsystem))
        .withSize(1, 1)
        .withPosition(4, 1);
    Shuffleboard.getTab("Match")
        .addBoolean("Update With Vision True", () -> driveSubsystem.visionUpdates)
        .withSize(1, 1)
        .withPosition(4, 0);
    Shuffleboard.getTab("Match")
        .addBoolean("IsRight", () -> robotStateSubsystem.getTargetCol() == TargetCol.RIGHT)
        .withSize(1, 1)
        .withPosition(3, 0);
    Shuffleboard.getTab("Match")
        .addBoolean("IsLeft", () -> robotStateSubsystem.getTargetCol() == TargetCol.LEFT)
        .withSize(1, 1)
        .withPosition(3, 1);
    Shuffleboard.getTab("Match")
        .addBoolean("IsCameraWorking", () -> visionSubsystem.isCameraWorking())
        .withSize(1, 1)
        .withPosition(7, 0);
    Shuffleboard.getTab("Match")
        .addBoolean("IsTrajGenerated", () -> autoSwitch.getAutoCommand().hasGenerated())
        .withSize(1, 1)
        .withPosition(7, 1);
    Shuffleboard.getTab("Match")
        .addBoolean("Is Elbow Ok?", () -> armSubsystem.isElbowOk())
        .withSize(1, 1)
        .withPosition(8, 0);
    Shuffleboard.getTab("Match")
        .addBoolean("Is Navx Connected", () -> driveSubsystem.isNavxWorking())
        .withSize(1, 1)
        .withPosition(8, 1);
    Shuffleboard.getTab("Match")
        .addString("AutoSwitchPos", () -> autoSwitch.getSwitchPos())
        .withSize(1, 1)
        .withPosition(0, 2);
    Shuffleboard.getTab("Match")
        .add("VirtualAutonSwitch", autoSwitch.getSendableChooser())
        .withSize(1, 1)
        .withPosition(1, 2);
    Shuffleboard.getTab("Match")
        .add("ToggleVirtualSwitch", new ToggleVirtualSwitchCommand(autoSwitch))
        .withSize(1, 1)
        .withPosition(2, 2);
    Shuffleboard.getTab("Match")
        .addBoolean("VirtualSwitchUsed?", () -> autoSwitch.isUseVirtualSwitch())
        .withSize(1, 1)
        .withPosition(3, 2);
  }

  private void configurePitDashboard() {
    ShuffleboardTab pitTab = Shuffleboard.getTab("Pit");

    ShuffleboardLayout intakeCommands =
        pitTab.getLayout("Intake", BuiltInLayouts.kGrid).withPosition(3, 0).withSize(1, 4);
    intakeCommands
        .add("Intake Stop", new IntakeOpenLoopCommand(intakeSubsystem, 0))
        .withPosition(0, 5);
    intakeCommands
        .add("Intake Fwd", new IntakeOpenLoopCommand(intakeSubsystem, IntakeConstants.kIntakeSpeed))
        .withPosition(0, 1);
    intakeCommands
        .add(
            "Intake Rev",
            new IntakeOpenLoopCommand(intakeSubsystem, IntakeConstants.kIntakeEjectSpeed))
        .withPosition(0, 2);
    intakeCommands
        .add("Intake Ext", new IntakeExtendCommand(intakeSubsystem, true))
        .withPosition(0, 3);
    intakeCommands
        .add("Intake Ret", new IntakeExtendCommand(intakeSubsystem, false))
        .withPosition(0, 4);

    // Shoulder Commands
    ShuffleboardLayout shoulderCommands =
        pitTab.getLayout("Shoulder", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(1, 3);
    shoulderCommands
        .add("Shoulder Stop", new ShoulderSpeedCommand(shoulderSubsystem, 0))
        .withPosition(0, 0);
    shoulderCommands
        .add("Shoulder Zero", new ZeroShoulderCommand(shoulderSubsystem))
        .withPosition(0, 1);
    shoulderCommands
        .add("Shoulder Forward", new ShoulderSpeedCommand(shoulderSubsystem, 0.1))
        .withPosition(0, 2);
    shoulderCommands
        .add("Shoulder Reverse", new ShoulderSpeedCommand(shoulderSubsystem, -0.1))
        .withPosition(0, 3);

    // Elevator Commands
    ShuffleboardLayout elevatorCommands =
        pitTab.getLayout("Elevator", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(1, 3);
    elevatorCommands
        .add("Elevator Stop", new ElevatorSpeedCommand(elevatorSubsystem, 0))
        .withPosition(0, 0);
    elevatorCommands
        .add("Elevator Zero", new ZeroElevatorCommand(elevatorSubsystem))
        .withPosition(0, 1);
    elevatorCommands
        .add("Elevator Up", new ElevatorSpeedCommand(elevatorSubsystem, 0.1))
        .withPosition(0, 2);
    elevatorCommands
        .add("Elevator Down", new ElevatorSpeedCommand(elevatorSubsystem, -0.1))
        .withPosition(0, 3);

    // Hand Commands
    ShuffleboardLayout handCommands =
        pitTab.getLayout("Hand", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(1, 9);
    handCommands
        .add("Hand Left Stop", new HandLeftSpeedCommand(handSubsystem, 0))
        .withPosition(0, 0);
    handCommands.add("Hand Zero", new ZeroHandCommand(handSubsystem)).withPosition(0, 2);
    handCommands
        .add("Hand Left In", new HandLeftSpeedCommand(handSubsystem, 0.1))
        .withPosition(0, 3);
    handCommands
        .add("Hand Left Out", new HandLeftSpeedCommand(handSubsystem, -0.1))
        .withPosition(0, 5);
    handCommands.add("Grab Cube", new GrabCubeCommand(handSubsystem)).withPosition(0, 7);
    handCommands.add("Grab Cone", new GrabConeCommand(handSubsystem)).withPosition(0, 8);
    handCommands
        .add("Hand 0 Check", new HandLeftToPositionCommand(handSubsystem, 0))
        .withPosition(1, 7);

    // Set game piece
    ShuffleboardLayout gamePieceCommands =
        pitTab.getLayout("Hand", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(1, 2);
    gamePieceCommands
        .add("Set cone", new SetGamePieceCommand(robotStateSubsystem, GamePiece.CONE))
        .withPosition(0, 0);
    gamePieceCommands
        .add("Set cube", new SetGamePieceCommand(robotStateSubsystem, GamePiece.CUBE))
        .withPosition(0, 1);

    ShuffleboardLayout HealthCheck =
        pitTab.getLayout("HealthCheck", BuiltInLayouts.kGrid).withPosition(1, 0).withSize(1, 1);
    HealthCheck.add(
            "HealthCheck",
            new ShuffleBoardHealthCheckCommandGroup(
                elbowSubsystem,
                shoulderSubsystem,
                elevatorSubsystem,
                handSubsystem,
                driveSubsystem,
                intakeSubsystem,
                armSubsystem))
        .withPosition(0, 0);
    HealthCheck.add("LockZero", new LockZeroCommand(driveSubsystem)).withPosition(0, 2);
  }

  private void configurePitImportantDashboard() {

    ShuffleboardTab pitImportantTab = Shuffleboard.getTab("PitImportant");
    pitImportantTab
        .add(
            "HealthCheck",
            new ShuffleBoardHealthCheckCommandGroup(
                elbowSubsystem,
                shoulderSubsystem,
                elevatorSubsystem,
                handSubsystem,
                driveSubsystem,
                intakeSubsystem,
                armSubsystem))
        .withPosition(0, 0);
    pitImportantTab
        .add(
            "HealthCheck Elbow(Elev.)",
            new ShuffleBoardHealthCheckElbowCommandGroup(
                elbowSubsystem, elevatorSubsystem, driveSubsystem, armSubsystem))
        .withPosition(0, 1);
    pitImportantTab
        .add(
            "HealthCheck Hand",
            new ShuffleBoardHealthCheckHandCommandGroup(
                handSubsystem, driveSubsystem, armSubsystem))
        .withPosition(1, 1);
    pitImportantTab
        .add(
            "HealthCheck Intake",
            new ShuffleBoardHealthCheckIntakeCommandGroup(
                intakeSubsystem, driveSubsystem, armSubsystem))
        .withPosition(2, 1);
    pitImportantTab.add("LockZero", new LockZeroCommand(driveSubsystem)).withPosition(1, 0);

    pitImportantTab
        .add(
            "Grab Cube",
            new HandToPositionCommand(handSubsystem, HandConstants.kIntakeOpenPosition))
        .withPosition(2, 0);
    pitImportantTab
        .add(
            "Grab Cone", new HandToPositionCommand(handSubsystem, HandConstants.kShelfOpenPosition))
        .withPosition(3, 0);
    pitImportantTab
        .add("Hand Zero", new HandToPositionCommand(handSubsystem, 0))
        .withPosition(4, 0);
  }

  public void configureDebugDashboard() {
    ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
    ShuffleboardLayout retrieveGamepiece =
        debugTab.getLayout("Retrieve", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(1, 1);
    retrieveGamepiece
        .add(
            "Retrieve GamePiece",
            new RetrieveGamePieceCommand(armSubsystem, handSubsystem, robotStateSubsystem))
        .withPosition(0, 0)
        .withSize(1, 1);
  }

  public void setAllianceColor(Alliance alliance) {
    this.alliance = alliance;
    allianceColor.withProperties(
        Map.of(
            "colorWhenTrue", alliance == Alliance.Red ? "red" : "blue", "colorWhenFalse", "black"));
    robotStateSubsystem.setAllianceColor(alliance);
    fiveMeterTest.generateTrajectory();
    balancepath.generateTrajectory();
    // communityToDockCommandGroup.generateTrajectory();
    // twoPieceWithDockAutoCommandGroup.generateTrajectory();
    // threePiecePath.generateTrajectory();
    // twoPieceAutoPlacePathCommandGroup.generateTrajectory();
    // bumpSideTwoPieceCommandGroup.generateTrajectory();
    if (autoSwitch.getAutoCommand() != null) {
      autoSwitch.getAutoCommand().generateTrajectory();
    }
    // Flips gyro angle if alliance is red team
    if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(180));
    } else {
      driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(0));
    }
  }

  public void updateGamePiece() {
    currGamePiece.withProperties(
        Map.of(
            "colorWhenTrue",
            robotStateSubsystem.getGamePiece() == GamePiece.CUBE ? "purple" : "yellow",
            "colorWhenFalse",
            "black"));
  }
}
