/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import java.util.ArrayList;

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
import frc.team670.robot.commands.auton.FourBallPath;
import frc.team670.robot.constants.AutonTrajectory;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.ClimberContainer;
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

  private static ConveyorSystem conveyorSystem = new ConveyorSystem();
  private static Deployer deployer = new Deployer();
  private static Intake intake = new Intake(conveyorSystem, deployer);
  private static Vision vision = new Vision(pd);
  private static Shooter shooter = new Shooter(vision);
  private static DriveBase driveBase = new DriveBase(getDriverController(), vision);
  private static LEDs leds = new LEDs(RobotMap.LED_PORT, RobotConstants.LED_LENGTH, shooter, intake, conveyorSystem);
  // private static ArrayList<Climber> climbers = ClimberContainer.getClimbers();
  private static Climber verticalClimber = ClimberContainer.getVerticalClimber();
  private static Climber diagonalClimber = ClimberContainer.getDiagonalClimber();
  private static OI oi = new OI(driveBase);
  // private static AutoSelector autoSelector = new AutoSelector(driveBase,
  // intake, conveyor, indexer, shooter, turret,
  // vision);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super();
    addSubsystem(conveyorSystem, shooter, intake, deployer, vision, leds, verticalClimber, diagonalClimber);
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

    MustangCommand autonCommand = new FourBallPath(driveBase, intake, conveyorSystem, shooter, deployer, AutonTrajectory.BTarmacHighHubTerminal);
    // MustangCommand autonCommand = new Edge2Ball(driveBase, intake, conveyorSystem, shooter, deployer, AutonTrajectory.ATarmacEdge2Ball, HubType.UPPER);
 
    
    //MustangCommand autonCommand = new Long4MeterPath(driveBase, intake, conveyorSystem, shooter);

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
    oi.configureButtonBindings(driveBase, conveyorSystem, shooter, intake, deployer, vision);
    driveBase.initDefaultCommand();
    deployer.setEncoderPositionFromAbsolute();
    pd.setSwitchableChannel(false);
  }

  @Override
  public void disabled() {
    leds.setIsDisabled(true);
    deployer.deploy(false);
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

  public void periodic() {
    SmartDashboard.putNumber("current", pd.getTotalCurrent());
    SmartDashboard.putNumber("energy", pd.getTotalEnergy());
    SmartDashboard.putNumber("power", pd.getTotalPower());
  }
}