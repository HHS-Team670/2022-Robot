/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.conveyor.RunConveyor;
import frc.team670.robot.commands.routines.ShootAllBalls;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Shooter;

public class RobotContainer extends RobotContainerBase {

  private static MustangCommand m_autonomousCommand;
  
  private DriveBase driveBase = new DriveBase(getDriverController());
  private static ConveyorSystem conveyorSystem = new ConveyorSystem();
  private static Shooter shooter = new Shooter();
  private static Intake intake = new Intake();

  private static OI oi = new OI(conveyorSystem, shooter);
  // private static AutoSelector autoSelector = new AutoSelector(driveBase,
  // intake, conveyor, indexer, shooter, turret,
  // vision);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super();
    addSubsystem(driveBase, conveyorSystem, shooter, intake);
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
    oi.configureButtonBindings(driveBase, conveyorSystem, shooter);
    driveBase.initDefaultCommand();
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
    conveyorSystem.debugBeamBreaks();
  }

}