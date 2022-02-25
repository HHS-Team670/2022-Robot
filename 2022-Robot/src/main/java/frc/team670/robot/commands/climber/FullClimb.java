package frc.team670.robot.commands.climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ClimberSystem;

/**
 * Once the driver aligns the back wheels to the bars under the hangar 
 * and drives straight to align with the climbing bar, this climbs on to the high bar.
 */
public class FullClimb extends SequentialCommandGroup implements MustangCommand {
  
  private ClimberSystem climberSystem;
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public FullClimb(ClimberSystem climberSystem) {
    this.climberSystem = climberSystem;

    addRequirements(climberSystem);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(climberSystem, HealthState.GREEN);
    
    addCommands(
      new ExtendClimber(climberSystem.getClimber1()),
      new HookOnBar(climberSystem.getClimber2()),
      new ExtendClimber(climberSystem.getClimber2()),
      new HookOnBar(climberSystem.getClimber2()),
      new UnhookFromBar(climberSystem.getClimber1()),
      new RetractClimber(climberSystem.getClimber1()));
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
  
}