
package frc.team670.robot.commands.vision;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;

/**
 * Set vision leds to turned on or off
 */
public class ToggleLED extends InstantCommand implements MustangCommand {

  VisionSubsystemBase vision;

  public ToggleLED(VisionSubsystemBase vision) {
    super();

    this.vision = vision;
  }

  // Called once when the command executes
  public void initialize() {
    // Logger.consoleLog("LEds turned to: %s", turnOn);
    vision.switchLEDS(!vision.LEDsTurnedOn(), true);
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // TODO Auto-generated method stub
    return null;
  }

}
