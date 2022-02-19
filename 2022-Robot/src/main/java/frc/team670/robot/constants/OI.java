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

  }
}