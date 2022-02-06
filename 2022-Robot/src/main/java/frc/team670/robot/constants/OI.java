// COPIED FROM 2020

package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.Joystick;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.subsystems.Vision;

public class OI extends OIBase {

  private static MustangController driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
  private static Joystick operatorController = new Joystick(RobotMap.OPERATOR_CONTROLLER_PORT);
  

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
    // Subsystem declaration
    DriveBase drivebase = (DriveBase) subsystemBases[0];
    Vision vision = (Vision) subsystemBases[1];

    // Command directing
    // toggleIntake.whenPressed(new ToggleIntake(intake));
    // runIntakeIn.toggleWhenPressed((new RunIntakeConveyor(intake, conveyor, indexer, false)));
    // runIntakeOut.toggleWhenPressed((new RunIntakeConveyor(intake, conveyor, indexer, true)));
    // toggleShooter.toggleWhenPressed(new ToggleShooter(shooter, vision));
    // extendClimb.whenPressed(new ExtendClimber(climber));
    // retractClimb.whenPressed(new Climb(climber));
    // autoOuttake.whenPressed(new ShootAllBalls(indexer, conveyor, shooter, vision));
    // autoIntake.whenPressed(new AutoIndex(intake, conveyor, indexer));
    // manualRunIndexerIn.whileHeld(new ManualRunIndexer(indexer, conveyor, intake, false));
    // manualRunIndexerOut.whileHeld(new ManualRunIndexer(indexer, conveyor, intake, true));

    // Xbox related commands
    // xboxVision.whenPressed(new RotateTurret(turret, drivebase, vision));
    // xboxIncreaseSpeed.whenPressed(new SetRPMAdjuster(50, shooter));
    // xboxDecreaseSpeed.whenPressed(new SetRPMAdjuster(-50, shooter));
    // xboxLowerClimber.whenPressed(new Climb(climber));
    // xboxRaiseClimber.whenPressed(new ExtendClimber(climber));
    // toggleReverseDrive.whenPressed(new FlipDriveDirection());

  }
}