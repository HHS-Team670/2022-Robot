package frc.team670.robot.constants;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague.FlipDriveDirection;
import frc.team670.mustanglib.commands.vision.ToggleLEDs;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.climber.ExtendClimber;
import frc.team670.robot.commands.climber.RetractClimber;
import frc.team670.robot.commands.deployer.ToggleIntake;
import frc.team670.robot.commands.routines.intake.EmptyRobot;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

public class OI extends OIBase {

  private static MustangController driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
  private static MustangController operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);

  // operator controls
  private static JoystickButton triggerIntaking = new JoystickButton(getOperatorController(), XboxButtons.X);
  private static JoystickButton triggerOuttaking = new JoystickButton(getOperatorController(), XboxButtons.B);
  private static JoystickButton stopAll = new JoystickButton(getOperatorController(), XboxButtons.A);
  private static JoystickButton toggleIntake = new JoystickButton(getOperatorController(), XboxButtons.Y);
  private static JoystickButton stopShooter = new JoystickButton(getOperatorController(), XboxButtons.RIGHT_BUMPER);
  private static JoystickButton shootAllBalls = new JoystickButton(getOperatorController(), XboxButtons.LEFT_BUMPER);

  // driver controls
  private static JoystickButton extendVerticalClimber = new JoystickButton(getDriverController(), XboxButtons.Y);
  private static JoystickButton retractVerticalClimber = new JoystickButton(getDriverController(), XboxButtons.A);
  private static JoystickButton extendDiagonalClimber = new JoystickButton(getDriverController(), XboxButtons.RIGHT_BUMPER);
  private static JoystickButton retractDiagonalClimber = new JoystickButton(getDriverController(), XboxButtons.LEFT_BUMPER);
  private static JoystickButton turnVisionLEDsOn = new JoystickButton(getDriverController(), XboxButtons.X);
  private static JoystickButton turnVisionLEDsOff = new JoystickButton(getDriverController(), XboxButtons.B); //TODO: make this be used
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
    Climber verticalClimber = (Climber) subsystemBases[6];
    Climber diagonalClimber = (Climber) subsystemBases[7];

    // fullClimb.whenPressed(new FullClimb(climberSystem));

    triggerIntaking.whenPressed(new RunIntakeWithConveyor(intake, conveyorSystem));
    triggerOuttaking.whenPressed(new EmptyRobot(intake, conveyorSystem, deployer));


    shootAllBalls.whenPressed(new ShootAllBalls(driveBase, conveyorSystem, shooter, vision));
    stopShooter.whenPressed((new StopShooter(shooter)));

    toggleIntake.whenPressed(new ToggleIntake(deployer));

    turnVisionLEDsOn.whenPressed(new ToggleLEDs(vision));

    extendVerticalClimber.whenPressed(new ExtendClimber(verticalClimber, Climber.Level.MID));
    retractVerticalClimber.whenPressed(new RetractClimber(verticalClimber, false));

    extendDiagonalClimber.whenPressed(new ExtendClimber(diagonalClimber, Climber.Level.HIGH));
    retractDiagonalClimber.whenPressed(new RetractClimber(diagonalClimber, false));

    //resetNavx.whenPressed(new ResetNavX(driveBase.getNavX()));
  }
}