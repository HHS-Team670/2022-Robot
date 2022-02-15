package frc.team670.robot.constants;

<<<<<<< HEAD
import javax.security.auth.x500.X500Principal;

import edu.wpi.first.wpilibj.Joystick;
=======
>>>>>>> dev
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague.FlipDriveDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
<<<<<<< HEAD
import frc.team670.robot.commands.conveyor.*;
import frc.team670.robot.commands.intake.*;
import frc.team670.robot.subsystems.*;
import frc.team670.robot.commands.routines.ShootAllBalls;
import frc.team670.robot.commands.routines.StopIntakeConveyor;
=======
import frc.team670.robot.commands.conveyor.RunConveyor;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.commands.intake.StopIntake;
import frc.team670.robot.commands.routines.intake.EmptyRobot;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
>>>>>>> dev
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.mustanglib.commands.drive.teleop.ResetNavX;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

public class OI extends OIBase {

  private static MustangController driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
<<<<<<< HEAD
  private static Joystick operatorController = new Joystick(RobotMap.OPERATOR_CONTROLLER_PORT);
  // private static XKeys xkeys;

  // operator buttons
  

  // xbox buttons
  private static JoystickButton xboxRunIntakeWithConveyor= new JoystickButton(getDriverController(),XboxButtons.Y);
  private static JoystickButton xboxStopIntakeConveyor= new JoystickButton(getDriverController(),XboxButtons.B);


=======
  private static MustangController operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);
>>>>>>> dev

  private static JoystickButton triggerIntaking = new JoystickButton(getOperatorController(), XboxButtons.X);
  private static JoystickButton triggerOuttaking = new JoystickButton(getOperatorController(), XboxButtons.B);
  private static JoystickButton stopShooter = new JoystickButton(getOperatorController(), XboxButtons.RIGHT_BUMPER);
  private static JoystickButton shootAllBalls = new JoystickButton(getOperatorController(), XboxButtons.LEFT_BUMPER);

  private static JoystickButton resetNavx = new JoystickButton(getDriverController(), XboxButtons.LEFT_BUMPER);
  
  private static JoystickButton stopIntake = new JoystickButton(getOperatorController(), XboxButtons.A);
  
  private static JoystickButton toggleReverseDrive = new JoystickButton(getDriverController(), XboxButtons.LEFT_BUMPER);

  private DriveBase driveBase;

  public OI(DriveBase driveBase) {
    this.driveBase = driveBase;
  }
  public boolean isQuickTurnPressed() {
    return driverController.getRightBumper();
  }

  /**
   * Sets the rumble on the driver controller
   * 
   * @param power The desired power of the rumble [0, 1]
   * @param time  The time to rumble for in seconds
   */
  public void rumbleDriverController(double power, double time) {
    rumbleController(driverController, power, time);
  }

  public static MustangController getDriverController() {
    return driverController;
  }

  public static MustangController getOperatorController() {
    return operatorController;
  }

  public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
<<<<<<< HEAD
    Intake intake = (Intake) subsystemBases[1];
    ConveyorSystem conveyor = (ConveyorSystem) subsystemBases[2];
    //XboxButtons
    xboxRunIntakeWithConveyor.whenPressed(new RunIntakeWithConveyor(intake,conveyor));
    xboxStopIntakeConveyor.whenPressed(new StopIntakeConveyor(intake,conveyor));
    DriveBase drivebase = (DriveBase) subsystemBases[0];
    ConveyorSystem conveyorSystem = (ConveyorSystem) subsystemBases[0];
    Shooter shooter = (Shooter) subsystemBases[0];
=======
    ConveyorSystem conveyorSystem = (ConveyorSystem) subsystemBases[1];
    Shooter shooter = (Shooter) subsystemBases[2];
    Intake intake = (Intake) subsystemBases[3];
>>>>>>> dev

    toggleReverseDrive.whenPressed(new FlipDriveDirection());

    triggerIntaking.whenPressed(new RunIntakeWithConveyor(intake, conveyorSystem));
    triggerOuttaking.whenPressed(new EmptyRobot(intake, conveyorSystem));

    stopIntake.whenPressed((new StopIntake(intake)));

    shootAllBalls.whenPressed(new ShootAllBalls(conveyorSystem, shooter));
    stopShooter.whenPressed((new StopShooter(shooter)));

    resetNavx.whenPressed(new ResetNavX(driveBase.getNavX()));
  }
}