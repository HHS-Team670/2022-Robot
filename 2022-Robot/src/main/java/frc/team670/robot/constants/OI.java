package frc.team670.robot.constants;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague.FlipDriveDirection;
import frc.team670.mustanglib.commands.vision.SetVisionLEDs;
import frc.team670.mustanglib.commands.vision.ToggleLEDs;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.deployer.ToggleIntake;
import frc.team670.robot.commands.routines.StopAll;
import frc.team670.robot.commands.routines.intake.EmptyRobot;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

public class OI extends OIBase {

  private static MustangController driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
  private static MustangController operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);

  private static JoystickButton triggerIntaking = new JoystickButton(getOperatorController(), XboxButtons.X);
  private static JoystickButton triggerOuttaking = new JoystickButton(getOperatorController(), XboxButtons.B);
  private static JoystickButton stopAll = new JoystickButton(getOperatorController(), XboxButtons.A);
  private static JoystickButton toggleIntake = new JoystickButton(getOperatorController(), XboxButtons.Y);
  private static JoystickButton stopShooter = new JoystickButton(getOperatorController(), XboxButtons.RIGHT_BUMPER);
  private static JoystickButton shootAllBalls = new JoystickButton(getOperatorController(), XboxButtons.LEFT_BUMPER);
  
  private static JoystickButton toggleReverseDrive = new JoystickButton(getDriverController(), XboxButtons.LEFT_BUMPER);
  private static JoystickButton turnVisionLEDsOn = new JoystickButton(getDriverController(), XboxButtons.X);
  private static JoystickButton turnVisionLEDsOff = new JoystickButton(getDriverController(), XboxButtons.B);
  //private static JoystickButton resetNavx = new JoystickButton(getDriverController(), XboxButtons.LEFT_BUMPER);

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
    DriveBase driveBase = (DriveBase) subsystemBases[0];
    ConveyorSystem conveyorSystem = (ConveyorSystem) subsystemBases[1];
    Shooter shooter = (Shooter) subsystemBases[2];
    Intake intake = (Intake) subsystemBases[3];
    Deployer deployer = (Deployer) subsystemBases [4];
    Vision vision = (Vision) subsystemBases [5];

    toggleReverseDrive.whenPressed(new FlipDriveDirection());

    triggerIntaking.whenPressed(new RunIntakeWithConveyor(intake, conveyorSystem));
    triggerOuttaking.whenPressed(new EmptyRobot(intake, conveyorSystem, deployer));

    stopAll.whenPressed((new StopAll(intake, conveyorSystem, shooter)));

    shootAllBalls.whenPressed(new ShootAllBalls(driveBase, conveyorSystem, shooter, vision));
    stopShooter.whenPressed((new StopShooter(shooter)));

    toggleIntake.whenPressed(new ToggleIntake(deployer));

    turnVisionLEDsOn.whenPressed(new SetVisionLEDs(true, vision));
    turnVisionLEDsOff.whenPressed(new SetVisionLEDs(false, vision));

    //resetNavx.whenPressed(new ResetNavX(driveBase.getNavX()));
  }
}