/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer extends RobotContainerBase {

  private static MustangCommand m_autonomousCommand;

  private static DriveBase driveBase = new DriveBase(getDriverController());
  private static ConveyorSystem conveyorSystem = new ConveyorSystem();
  private static Intake intake = new Intake(conveyorSystem);
  private static Shooter shooter = new Shooter();

  private static OI oi = new OI(driveBase);
  private SparkMAXLite flipout;
  private DutyCycleEncoder absEncoder = new DutyCycleEncoder(2);
  RelativeEncoder mEncoder;
  // private static AutoSelector autoSelector = new AutoSelector(driveBase,
  // intake, conveyor, indexer, shooter, turret,
  // vision);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super();
    addSubsystem(conveyorSystem, shooter, intake);
    flipout = SparkMAXFactory.buildFactorySparkMAX(RobotMap.FLIP_OUT, Motor_Type.NEO);
    flipout.setInverted(true);
    mEncoder = flipout.getEncoder();
  }

  public void robotInit() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public MustangCommand getAutonomousCommand() {
    // MustangCommand autonCommand = new MoveForwards(driveBase);
    // MustangCommand autonCommand = new RightShootTrench(driveBase);

    // Logger.consoleLog("autonCommand: %s", autonCommand);
    return null;
  }

  public void autonomousInit() {
    Logger.consoleLog("autoInit called");

  }

  public void teleopInit() {
    oi.configureButtonBindings(driveBase, conveyorSystem, shooter, intake);
    driveBase.initDefaultCommand();
  }

  @Override
  public void disabled() {

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
    SmartDashboard.putNumber("intake-motor-pos", mEncoder.getPosition());
    SmartDashboard.putNumber("abs-motor-pos", absEncoder.get());
  }

  // 8.142 - out 0- in
  // 0.23 out -0.086 in

  /**
   * max a: 700 (down), 1900 (up), 
   * max v: 3500 (both)
   * 
   * p: 0.00015
   * f: 0.000176
   */

}