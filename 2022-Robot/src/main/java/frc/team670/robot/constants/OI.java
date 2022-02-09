// COPIED FROM 2020

package frc.team670.robot.constants;

import javax.security.auth.x500.X500Principal;

import edu.wpi.first.wpilibj.Joystick;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.intake.StopIntakeConveyor;
import frc.team670.robot.subsystems.*;

public class OI extends OIBase {

  private static MustangController driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
  private static Joystick operatorController = new Joystick(RobotMap.OPERATOR_CONTROLLER_PORT);
  // private static XKeys xkeys;

  // operator buttons
  

  // xbox buttons
  private static JoystickButton xboxRunIntakeWithConveyor= new JoystickButton(getDriverController(),XboxButtons.Y);
  private static JoystickButton xboxStopIntakeConveyor= new JoystickButton(getDriverController(),XboxButtons.B);


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
    Conveyor conveyor = (Conveyor) subsystemBases[2];
    //XboxButtons
    xboxRunIntakeWithConveyor.whenPressed(new RunIntakeWithConveyor(intake,conveyor));
    xboxStopIntakeConveyor.whenPressed(new StopIntakeConveyor(intake,conveyor));
  }
}