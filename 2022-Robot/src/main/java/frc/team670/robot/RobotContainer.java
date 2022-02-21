/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.auton.Edge2Ball;
import frc.team670.robot.commands.auton.FourBallPath;
import frc.team670.robot.commands.auton.Long4MeterPath;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
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
    // ------------ Edge2Ball path names (copy/paste) ------------
    //   - "ATarmacEdge2Ball"
    //   - "BTarmacEdgeCenter2Ball"
    //   - "BTarmacEdgeLower2Ball"

    // ------------ FourBallPath path names (copy/paste) ------------
    //   - "BTarmac4BallTerminal"
    //   - "BTarmac4BallTerminal2Ball"
    //   - "ATarmacEdge4Ball"

    MustangCommand autonCommand = new FourBallPath(driveBase, intake, conveyorSystem, shooter, "BTarmacHighHubTerminal");
    // MustangCommand autonCommand = new Edge2Ball(driveBase, intake, conveyorSystem, shooter, "ATarmacEdge2Ball");
 
    // MustangCommand autonCommand = new Long4MeterPath(driveBase, intake, conveyorSystem, shooter);

    // MustangCommand autonCommand = new BTarmac5BallTerminal(driveBase, intake, conveyorSystem, shooter, vision);

    // Logger.consoleLog("autonCommand: %s", autonCommand);
    return autonCommand;
    // return null;
  }

  public void autonomousInit() {
    deployer.setEncoderPositionFromAbsolute();
    driveBase.initBrakeMode();
    Logger.consoleLog("autoInit called");

  }

  public void teleopInit() {
    driveBase.initCoastMode(); // InitCoastMode was added by auton so we could reset the bot more easily. Remove if needed.
    
    oi.configureButtonBindings(driveBase, conveyorSystem, shooter, intake, deployer, vision);
    driveBase.initDefaultCommand();
    deployer.setEncoderPositionFromAbsolute();
    pd.setSwitchableChannel(false);
  }

  @Override
  public void disabled() {
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

  public void periodic() {
    
  }

}