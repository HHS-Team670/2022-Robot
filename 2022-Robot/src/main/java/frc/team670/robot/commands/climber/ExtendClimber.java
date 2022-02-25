package frc.team670.robot.commands.climber;

import java.util.HashMap;
import java.util.Map;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ClimberSystem;

/**
 * Raise the climber mechanism to its maximum allowed height so it can reach the
 * generator bar.
 */
public class ExtendClimber implements MustangCommand{

  private ClimberSystem climberSystem;
  private boolean vertical;
  private HashMap<MustangSubsystemBase, HealthState> healthReqs;

  
  public ExtendClimber(ClimberSystem climberSystem, boolean vertical) {
    this.climberSystem = climberSystem;
    this.vertical = vertical;
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(vertical ? climberSystem.getClimber1() : climberSystem.getClimber2(), HealthState.GREEN);
  }


  @Override
  public void initialize() {
    climberSystem.climb(vertical, climber.MAX_EXTENDING_HEIGHT_CM);
  }

  @Override
  public boolean isFinished() {
    return climber.isAtTarget();
  }


  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // TODO Auto-generated method stub
    return null;
  }
}
