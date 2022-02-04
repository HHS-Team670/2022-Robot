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
import frc.team670.robot.commands.conveyor.StopConveyor;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.Conveyors;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Shooter;





public class RobotContainer extends RobotContainerBase {

  private static OI oi = new OI();
  // private DriveBase driveBase = new DriveBase(getDriverController());
  int i = 0;

  private MustangCommand m_autonomousCommand;
  private Conveyors conveyors = new Conveyors();
  private Shooter shooter = new Shooter();

  // private static AutoSelector autoSelector = new AutoSelector(driveBase, intake, conveyor, indexer, shooter, turret,
  //     vision);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super();
    addSubsystem(conveyors, shooter);
    
    
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
      //  MustangCommand autonCommand = new RightShootTrench(driveBase);

    // Logger.consoleLog("autonCommand: %s", autonCommand);
    return null;
  }

  public void autonomousInit() {
    Logger.consoleLog("autoInit called");

  }

  public void teleopInit() {
  
  //  conveyors.runConveyor(Conveyors.Status.INTAKING);
  // MustangScheduler.getInstance().schedule);
  configureButtonBindings();
  
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
    // conveyors.debugBeamBreaks();
    // beam.sendBeamBreakDataToDashboard();
    conveyors.debugBeamBreaks();
  }

  private void configureButtonBindings(){
    JoystickButton triggerIntaking = new JoystickButton(getDriverController(), XboxButtons.A);
    JoystickButton triggerOuttaking = new JoystickButton(getDriverController(), XboxButtons.B);
    JoystickButton triggerShooting = new JoystickButton(getDriverController(), XboxButtons.X);
    JoystickButton startShooter = new JoystickButton(getDriverController(), XboxButtons.LEFT_BUMPER);
    JoystickButton stopShooter = new JoystickButton(getDriverController(), XboxButtons.RIGHT_BUMPER);
    triggerIntaking.whenPressed((new RunConveyor(conveyors, Conveyors.Status.INTAKING)));
    triggerOuttaking.whenPressed((new RunConveyor(conveyors, Conveyors.Status.OUTTAKING)));
    triggerShooting.whenPressed((new RunConveyor(conveyors, Conveyors.Status.SHOOTING)));
    startShooter.whenPressed((new StartShooter(shooter)));
    stopShooter.whenPressed((new StopShooter(shooter)));
  }

}