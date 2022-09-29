package frc.team670.robot.constants;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.vision.SetVisionLEDs;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
// import frc.team670.robot.commands.climber.ExtendClimber;
// import frc.team670.robot.commands.climber.RetractClimber;
// import frc.team670.robot.commands.climber.StepClimber;
import frc.team670.robot.commands.climber.*;

import frc.team670.robot.commands.conveyor.ToggleConveyor;
import frc.team670.robot.commands.drivebase.AlignAngleToTarget;
import frc.team670.robot.commands.drivebase.HoldPosition;
import frc.team670.robot.commands.intake.StopIntake;
import frc.team670.robot.commands.routines.StopAll;
import frc.team670.robot.commands.routines.drivebase.AlignAngleToTargetAndShoot;
import frc.team670.robot.commands.routines.intake.EjectCargo;
import frc.team670.robot.commands.routines.intake.RaiseIntakeToAngle;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.intake.ToggleIntakeAndConveyor;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.commands.vision.*;

import frc.team670.robot.subsystems.ClimberSystem;
import frc.team670.robot.subsystems.ClimberSystem.Climber;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

public class OI extends OIBase {

  private static MustangController driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
  private static MustangController operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);
  private static MustangController backupController = new MustangController(RobotMap.BACKUP_CONTROLLER_PORT);

  // operator controls
  private static JoystickButton triggerOuttaking = new JoystickButton(getOperatorController(), XboxButtons.B);
  private static JoystickButton reverseClimber = new JoystickButton(getOperatorController(), XboxButtons.A);
  private static JoystickButton toggleIntake = new JoystickButton(getOperatorController(), XboxButtons.Y);
  private static JoystickButton stepClimber = new JoystickButton(getOperatorController(), XboxButtons.X);
  private static JoystickButton shootAllBalls = new JoystickButton(getOperatorController(), XboxButtons.LEFT_BUMPER);
  private static JoystickButton toggleLED = new JoystickButton(getOperatorController(), XboxButtons.BACK);
  private static JoystickButton stopAll = new JoystickButton(getOperatorController(), XboxButtons.START);
  // private static JoystickButton turnVisionLEDsOffOp = new
  // JoystickButton(getOperatorController(), XboxButtons.START);
  // driver controls
  private static JoystickButton alignToTarget = new JoystickButton(getDriverController(), XboxButtons.RIGHT_BUMPER);
  private static JoystickButton toggleRaisedIntake = new JoystickButton(getDriverController(), XboxButtons.Y);
  private static JoystickButton holdPosition = new JoystickButton(getDriverController(), XboxButtons.A);
  private static JoystickButton alignAndShoot = new JoystickButton(getDriverController(), XboxButtons.LEFT_BUMPER);
  private static JoystickButton stopShooter = new JoystickButton(getDriverController(), XboxButtons.RIGHT_BUMPER);

  // backup controls
  private static JoystickButton toggleConveyor = new JoystickButton(getBackupController(), XboxButtons.X);
  private static JoystickButton raiseC1 = new JoystickButton(getBackupController(), XboxButtons.Y);
  private static JoystickButton lowerC1 = new JoystickButton(getBackupController(), XboxButtons.A);
  private static JoystickButton lowerC1ForC2 = new JoystickButton(getBackupController(), XboxButtons.B);
  private static JoystickButton raiseC2 = new JoystickButton(getBackupController(), XboxButtons.RIGHT_BUMPER);
  private static JoystickButton lowerC2 = new JoystickButton(getBackupController(), XboxButtons.LEFT_BUMPER);
  private static JoystickButton turnVisionLEDsOn = new JoystickButton(getBackupController(), XboxButtons.BACK);
  private static JoystickButton turnVisionLEDsOff = new JoystickButton(getBackupController(), XboxButtons.START);

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

  public static MustangController getBackupController() {
    return backupController;
  }

  public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
    DriveBase driveBase = (DriveBase) subsystemBases[0];
    ConveyorSystem conveyorSystem = (ConveyorSystem) subsystemBases[1];
    Shooter shooter = (Shooter) subsystemBases[2];
    Intake intake = (Intake) subsystemBases[3];
    Deployer deployer = (Deployer) subsystemBases[4];
    Vision vision = (Vision) subsystemBases[5];
    Climber verticalClimber = (Climber) subsystemBases[6];
    Climber diagonalClimber = (Climber) subsystemBases[7];
    ClimberSystem climber = (ClimberSystem) subsystemBases[8];

    driveBase.initDefaultCommand();
    shooter.initDefaultCommand();

    // operator
    triggerOuttaking.whenPressed(new EjectCargo(intake, conveyorSystem, deployer));
    reverseClimber.whenPressed(new StepClimber(climber, deployer, true));
    toggleIntake.whenPressed(new ToggleIntakeAndConveyor(intake, conveyorSystem));
    // stopIntake.whenPressed(new StopIntake(intake));
    alignAndShoot.whenPressed(new AlignAngleToTargetAndShoot(driveBase, vision, conveyorSystem, shooter));
    stopShooter.whenPressed(new StopShooter(shooter));
    stepClimber.whenPressed(new StepClimber(climber, deployer, false));
    toggleLED.whenPressed(new ToggleLED(vision));
    stopAll.whenPressed(new StopAll());

    // turnVisionLEDsOffOp.whenPressed(new SetVisionLEDs(false, vision));
    // driver
    shootAllBalls.whenPressed(new ShootAllBalls(conveyorSystem, shooter));
    
    

    alignToTarget.whenPressed(new AlignAngleToTarget(driveBase, vision));
    toggleRaisedIntake.whenPressed(new RaiseIntakeToAngle(60, deployer, intake)); // TODO: still doesn't work, gotta
                                                                                  // figure it out, angles off
    holdPosition.toggleWhenPressed(new HoldPosition(driveBase));

    // backup
    toggleConveyor.whenPressed(new ToggleConveyor(conveyorSystem));
    raiseC1.whenPressed(new ExtendClimber(verticalClimber, ClimberSystem.Level.MID));
    lowerC1.whenPressed(new RetractClimber(verticalClimber, false));
    lowerC1ForC2.whenPressed(new ExtendClimber(verticalClimber, ClimberSystem.Level.INTERMEDIATE_MID));
    raiseC2.whenPressed(new ExtendClimber(diagonalClimber, ClimberSystem.Level.HIGH));
    lowerC2.whenPressed(new RetractClimber(diagonalClimber, false));
    turnVisionLEDsOn.whenPressed(new SetVisionLEDs(true, vision));
    turnVisionLEDsOff.whenPressed(new SetVisionLEDs(false, vision));
  }
}