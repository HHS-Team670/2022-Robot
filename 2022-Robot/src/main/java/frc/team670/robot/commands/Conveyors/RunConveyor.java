package frc.team670.robot.commands.Conveyors;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Conveyors;

/**
 * Runs the conveyor in the given mode
 * 
 * @author Armaan
 * @author Soham
 * @author Nancy
 * @author Scintilla
 * 
 */
public class RunConveyor extends InstantCommand implements MustangCommand {

  private Conveyors conveyors;
  private Map<MustangSubsystemBase, HealthState> healthReqs;
  private Conveyors.Status mode;

  public RunConveyor(Conveyors conveyors, Conveyors.Status mode) {
    this.conveyors = conveyors;
    addRequirements(conveyors);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(conveyors, HealthState.GREEN);
    this.mode = mode;
  }

  public void initialize() {
    conveyors.runConveyor(mode);
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }

}
