/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import com.fasterxml.jackson.databind.util.Converter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Timer;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.auton.AutoSelector;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Conveyors;


public class RobotContainer extends RobotContainerBase {

  private static OI oi = new OI();

  private static DriveBase driveBase = new DriveBase(getDriverController());

  private static AutoSelector autoSelector = new AutoSelector();

  int i = 0;

  private MustangCommand m_autonomousCommand;
  private Conveyors conveyors = new Conveyors();

  // private static AutoSelector autoSelector = new AutoSelector(driveBase, intake, conveyor, indexer, shooter, turret,
  //     vision);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super();
    addSubsystem(driveBase);
    
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
    
    MustangCommand autonCommand = autoSelector.getCommandFromRoutine(autonRoutine, 
    delayTime);
    // MustangCommand autonCommand = new LeftShoot2BallSide(driveBase, intake, conveyor, indexer, turret, shooter);
    // MustangCommand autonCommand = new CenterSho ot3BallSide(driveBase, intake, conveyor, indexer, turret, shooter, vision);
    // MustangCommand autonCommand = new RightShootTrench(driveBase, intake, conveyor, indexer, turret, shooter, vision);
    Logger.consoleLog("autonCommand: %s", autonCommand);
    return autonCommand;
  }

  public void autonomousInit() {
    
  }

  public void teleopInit() {
    Timer.delay(1);
    Logger.consoleLog("WORKSSS??");
  
    conveyors.runConveyor(Conveyors.Status.INTAKING);
  
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
   //break1.sendBeamBreakDataToDashboard();
  }

}