package frc.team670.robot.commands.climber;

import java.util.HashMap;
import java.util.Map;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ClimberSystem;

/**
 * Lower the Climber mechanism.
 */
public class RetractClimber implements MustangCommand{

  private ClimberSystem climberSystem;
  private boolean vertical;
  private HashMap<MustangSubsystemBase, HealthState> healthReqs;
  
  public RetractClimber(ClimberSystem climberSystem, boolean vertical) {
    this.climberSystem = climberSystem;
    this.vertical = vertical;
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(vertical ? climberSystem.getClimber1() : climberSystem.getClimber2(), HealthState.GREEN);
  }

  public void initialize() {
    climberSystem.stop();
    climberSystem.climb(vertical, 0);
  }


  public boolean isFinished() {
    return climberSystem.isAtTarget(vertical);
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // TODO Auto-generated method stub
    return healthReqs;
  }
}
