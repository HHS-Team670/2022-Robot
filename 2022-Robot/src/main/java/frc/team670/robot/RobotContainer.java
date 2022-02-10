/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.auton.ATarmacEdge4Ball;
import frc.team670.robot.commands.auton.ATarmacFlushed1Ball;
import frc.team670.robot.commands.auton.BTarmac4BallTerminal;
import frc.team670.robot.commands.auton.BTarmacTriangle;
import frc.team670.robot.commands.auton.LeftTarmac2Shoot;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.DriveBase;


public class RobotContainer extends RobotContainerBase {

  private static OI oi = new OI();
  private DriveBase driveBase = new DriveBase(getDriverController());
  int i = 0;

  private MustangCommand m_autonomousCommand;

  // private static AutoSelector autoSelector = new AutoSelector(driveBase, intake, conveyor, indexer, shooter, turret,
  //     vision);

  BeamBreak break1 = new BeamBreak(9);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super();
    addSubsystem(driveBase);
    oi.configureButtonBindings(driveBase);
    
    
  }

  public void robotInit() {
 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public MustangCommand getAutonomousCommand() {
    MustangCommand autonCommand = new ATarmacEdge4Ball(driveBase);

    Logger.consoleLog("autonCommand: %s", autonCommand);
    return autonCommand;
  }


  public void autonomousInit() {
    Logger.consoleLog("autoInit called");

    m_autonomousCommand = getAutonomousCommand();
    if (m_autonomousCommand != null) {
      MustangScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  public void teleopInit() {
    Logger.consoleLog(driveBase.getPose().toString());
    
  }

  @Override
  public void disabled() {
    
  }

  public static Joystick getOperatorController() {
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
    break1.sendBeamBreakDataToDashboard();
    // driveBase.getHeading();
    SmartDashboard.putNumber("navX", driveBase.getHeading());
    SmartDashboard.putString("Encoder Position", String.format("(%f, %f)", driveBase.getPose().getX(), driveBase.getPose().getY()));
  }

}