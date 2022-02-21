/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.auton.AutoSelector;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

public class RobotContainer extends RobotContainerBase {

  private static MustangCommand m_autonomousCommand;

  
  private static ConveyorSystem conveyorSystem = new ConveyorSystem();
  private static Deployer deployer = new Deployer();
  private static Intake intake = new Intake(conveyorSystem, deployer);
  private static Shooter shooter = new Shooter();
  private static AutoSelector autoSelector = new AutoSelector();
  
  private static PowerDistribution pd = new PowerDistribution(1, ModuleType.kRev);

  private static Vision vision = new Vision(pd);
  private static DriveBase driveBase = new DriveBase(getDriverController(), vision);

  private static OI oi = new OI(driveBase);
  // private static AutoSelector autoSelector = new AutoSelector(driveBase,
  // intake, conveyor, indexer, shooter, turret,
  // vision);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super();
    addSubsystem(conveyorSystem, shooter, intake, deployer, vision);
  }

  public void robotInit() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public MustangCommand getAutonomousCommand() {
    AutoSelector.AutoRoutine autonRoutine = driveBase.getSelectedRoutine();
    double delayTime = driveBase.getDelayTime();

    Logger.consoleLog("Inside getAutonomousCommand - delay time:" + delayTime);
    
    MustangCommand autonCommand = autoSelector.getCommandFromRoutine(autonRoutine, 
    delayTime);
    // MustangCommand autonCommand = new LeftShoot2BallSide(driveBase, intake, conveyor, indexer, turret, shooter);
    // MustangCommand autonCommand = new CenterSho ot3BallSide(driveBase, intake, conveyor, indexer, turret, shooter, vision);
    // MustangCommand autonCommand = new RightShootTrench(driveBase, intake, conveyor, indexer, turret, shooter, vision);
    Logger.consoleLog("autonCommand: %s", autonCommand);
    return autonCommand;
    // MustangCommand autonCommand = new MoveForwards(driveBase);
    // MustangCommand autonCommand = new RightShootTrench(driveBase);

    // Logger.consoleLog("autonCommand: %s", autonCommand);
    // return null;
  }

  public void autonomousInit() {
    Logger.consoleLog("autoInit called");

  }

  public void teleopInit() {
    oi.configureButtonBindings(driveBase, conveyorSystem, shooter, intake, deployer, vision);
    driveBase.initDefaultCommand();
    deployer.setEncoderPositionFromAbsolute();
    pd.setSwitchableChannel(false);
  }

  @Override
  public void disabled() {
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
    
  }

}