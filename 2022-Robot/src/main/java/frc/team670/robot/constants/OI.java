package frc.team670.robot.constants;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague.FlipDriveDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.conveyor.RunConveyor;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.commands.intake.StopIntake;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

public class OI extends OIBase {

  private static MustangController driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
  private static MustangController operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);

  private static JoystickButton triggerIntaking = new JoystickButton(getOperatorController(), XboxButtons.X);
  private static JoystickButton triggerOuttaking = new JoystickButton(getOperatorController(), XboxButtons.B);
  private static JoystickButton stopShooter = new JoystickButton(getOperatorController(), XboxButtons.RIGHT_BUMPER);
  private static JoystickButton shootAllBalls = new JoystickButton(getOperatorController(), XboxButtons.LEFT_BUMPER);
  
  private static JoystickButton runIntake = new JoystickButton(getOperatorController(), XboxButtons.Y);
  private static JoystickButton stopIntake = new JoystickButton(getOperatorController(), XboxButtons.A);
  
  private static JoystickButton toggleReverseDrive = new JoystickButton(getDriverController(), XboxButtons.LEFT_BUMPER);

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

  public static MustangController getOperatorController() {
    return operatorController;
  }

  public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
    DriveBase drivebase = (DriveBase) subsystemBases[0];
    ConveyorSystem conveyorSystem = (ConveyorSystem) subsystemBases[1];
    Shooter shooter = (Shooter) subsystemBases[2];
    Intake intake = (Intake) subsystemBases[3];

    toggleReverseDrive.whenPressed(new FlipDriveDirection());

    triggerIntaking.whenPressed((new RunConveyor(conveyorSystem, ConveyorSystem.Status.INTAKING)));
    triggerOuttaking.whenPressed((new RunConveyor(conveyorSystem, ConveyorSystem.Status.OUTTAKING)));

    runIntake.toggleWhenPressed((new RunIntake(intake)));
    stopIntake.whenPressed((new StopIntake(intake)));

    shootAllBalls.whenPressed(new ShootAllBalls(conveyorSystem, shooter));
    stopShooter.whenPressed((new StopShooter(shooter)));
  }
}