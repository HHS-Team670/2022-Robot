/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.LEDColor;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.auton.AutoSelector;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.ClimberSystem;
import frc.team670.robot.subsystems.ClimberSystem.Climber;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.commands.auton.AutoSelector;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.ClimberSystem;
import frc.team670.robot.subsystems.ClimberSystem.Climber;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.LEDs;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

public class RobotContainer extends RobotContainerBase {

  private static MustangCommand m_autonomousCommand;
  
  private static PowerDistribution pd = new PowerDistribution(1, ModuleType.kRev);

  private static Deployer deployer = new Deployer();
  private static ConveyorSystem conveyorSystem = new ConveyorSystem(deployer);
  private static Intake intake = new Intake(conveyorSystem, deployer);
  private static Vision vision = new Vision(pd);
  private static Shooter shooter = new Shooter(vision, getOperatorController(), deployer, conveyorSystem);
  private static DriveBase driveBase = new DriveBase(getDriverController(), vision);
  private static ClimberSystem climbers = new ClimberSystem(getBackupController());
  private static Climber verticalClimber = climbers.getVerticalClimber();
  private static Climber diagonalClimber = climbers.getDiagonalClimber();
  private static LEDs leds = new LEDs(RobotMap.LED_PORT, RobotConstants.LED_START_INDEX, RobotConstants.LED_END_INDEX, shooter, intake, conveyorSystem, climbers);
  // private static ArrayList<Climber> climbers = ClimberContainer.getClimbers();
  
  private static OI oi = new OI();
  private static AutoSelector autoSelector = new AutoSelector();

  // private static AutoSelector autoSelector = new AutoSelector(driveBase,
  // intake, conveyor, indexer, shooter, turret,
  // vision);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super();
    addSubsystem(conveyorSystem, shooter, intake, deployer, vision, leds, verticalClimber, diagonalClimber, climbers);
  }

  public void robotInit() {
    leds.setIsDisabled(true);
    vision.switchLEDS(false);
    Alliance alliance = DriverStation.getAlliance();
    if(alliance == Alliance.Red) {
      leds.setAllianceColors(LEDColor.RED, LEDColor.BLUE);
    } else {
      leds.setAllianceColors(LEDColor.BLUE, LEDColor.RED);
    }
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public MustangCommand getAutonomousCommand() {
    // ------------ Edge2Ball path names (copy/paste) ------------
    //   - "ATarmacEdge2Ball"
    //   - "BTarmacEdgeCenter2Ball"
    //   - "BTarmacEdgeLower2Ball"
    

    // ------------ FourBallPath path names (copy/paste) ------------
    //   - "BTarmac4BallTerminal"
    //   - "BTarmacHighHubTerminal"
    //   - "ATarmacEdge4Ball"

    // MustangCommand autonCommand = new FourBallPath(driveBase, intake, conveyorSystem, shooter, deployer, AutonTrajectory.BTarmacHighHubTerminal);
    // MustangCommand autonCommand = new Edge2Ball(driveBase, intake, conveyorSystem, shooter, deployer, AutonTrajectory.ATarmacEdge2Ball, HubType.UPPER);
    // MustangCommand autonCommand = new Edge2Ball(driveBase, intake, conveyorSystem, shooter, deployer, AutonTrajectory.BTarmacEdgeCenter2Ball, HubType.UPPER);

    
    //MustangCommand autonCommand = new Long4MeterPath(driveBase, intake, conveyorSystem, shooter);
    int autonRoutine = driveBase.getSelectedRoutine();
    double delayTime = driveBase.getDelayTime();

    Logger.consoleLog("Inside getAutonomousCommand - delay time:" + delayTime);

    MustangCommand autonCommand = autoSelector.getCommandFromRoutine(autonRoutine, delayTime, driveBase, intake,
        conveyorSystem, shooter, deployer);
    if (autonCommand== null)
      Logger.consoleError("Auton Command is Null. Manually change Path and Deploy!");

    // MustangCommand autonCommand = new Long4MeterPath(driveBase, intake, conveyorSystem, shooter);

    // MustangCommand autonCommand = new FourBallPath(driveBase, intake, conveyorSystem, shooter, deployer, AutonTrajectory.BTarmacHighHubTerminal);

    // Logger.consoleLog("autonCommand: %s", autonCommand);
    return autonCommand;
  }

  public void autonomousInit() {
    deployer.setEncoderPositionFromAbsolute();
    driveBase.initBrakeMode();
    
    Logger.consoleLog("autoInit called");
    leds.setIsDisabled(false);
  }

  public void teleopInit() {
    leds.setIsDisabled(false);
    oi.configureButtonBindings(driveBase, conveyorSystem, shooter, intake, deployer, vision, verticalClimber, diagonalClimber);
    deployer.setEncoderPositionFromAbsolute();
    pd.setSwitchableChannel(false);
    // MustangScheduler.getInstance().schedule(new RetractClimber(verticalClimber, true));
  }

  @Override
  public void disabled() {
    leds.setIsDisabled(true);
    // deployer.deploy(false);
  }

  public static MustangController getOperatorController() {
    return OI.getOperatorController();
  }

  public static void rumbleDriverController() {
    notifyDriverController(1.0, 0.3);
  }

  public static void rumbleDriverController(double power, double time) {
    oi.rumbleDriverController(power, time);
  }

  public static void notifyDriverController(double power, double time) {
    oi.rumbleDriverController(power, time);
  }

  public static MustangController getDriverController() {
    return OI.getDriverController();
  }

  public static MustangController getBackupController() {
    return OI.getBackupController();
  }

  public void periodic() {
    SmartDashboard.putNumber("current", pd.getTotalCurrent());
    SmartDashboard.putNumber("energy", pd.getTotalEnergy());
    SmartDashboard.putNumber("power", pd.getTotalPower());
    for(int i = 0; i < pd.getNumChannels(); i++){
      SmartDashboard.putNumber(("Channel " + i), pd.getCurrent(i));
    }
  }
}