package frc.team670.robot.constants;

import javax.security.auth.x500.X500Principal;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague.FlipDriveDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.conveyor.*;
import frc.team670.robot.commands.intake.*;
import frc.team670.robot.subsystems.*;
import frc.team670.robot.commands.routines.ShootAllBalls;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Shooter;

public class OI extends OIBase {

  private static MustangController driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
  private static Joystick operatorController = new Joystick(RobotMap.OPERATOR_CONTROLLER_PORT);
  // private static XKeys xkeys;

  // operator buttons
  

  // xbox buttons
  private static JoystickButton xboxRunIntakeWithConveyor= new JoystickButton(getDriverController(),XboxButtons.Y);
  private static JoystickButton xboxStopIntakeConveyor= new JoystickButton(getDriverController(),XboxButtons.B);



  private static JoystickButton triggerIntaking = new JoystickButton(getDriverController(), XboxButtons.A);
  private static JoystickButton triggerOuttaking = new JoystickButton(getDriverController(), XboxButtons.B);
  private static JoystickButton triggerShooting = new JoystickButton(getDriverController(), XboxButtons.X);
  private static JoystickButton stopShooter = new JoystickButton(getDriverController(), XboxButtons.RIGHT_BUMPER);
  private static JoystickButton shootAllBalls = new JoystickButton(getDriverController(), XboxButtons.Y);
  private static JoystickButton toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);

  public OI(ConveyorSystem conveyorSystem, Shooter shooter) {
    
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

  public static Joystick getOperatorController() {
    return operatorController;
  }

  public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
    Intake intake = (Intake) subsystemBases[1];
    ConveyorSystem conveyor = (ConveyorSystem) subsystemBases[2];
    //XboxButtons
    xboxRunIntakeWithConveyor.whenPressed(new RunIntakeWithConveyor(intake,conveyor));
    xboxStopIntakeConveyor.whenPressed(new StopIntakeConveyor(intake,conveyor));
    DriveBase drivebase = (DriveBase) subsystemBases[0];
    ConveyorSystem conveyorSystem = (ConveyorSystem) subsystemBases[0];
    Shooter shooter = (Shooter) subsystemBases[0];

    toggleReverseDrive.whenPressed(new FlipDriveDirection());

    triggerIntaking.whenPressed((new RunConveyor(conveyorSystem, ConveyorSystem.Status.INTAKING)));
    triggerOuttaking.whenPressed((new RunConveyor(conveyorSystem, ConveyorSystem.Status.OUTTAKING)));
    triggerShooting.whenPressed((new RunConveyor(conveyorSystem, ConveyorSystem.Status.SHOOTING)));

    shootAllBalls.whenPressed(new ShootAllBalls(conveyorSystem, shooter));
    stopShooter.whenPressed((new StopShooter(shooter)));
  }
}