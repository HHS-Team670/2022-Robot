package frc.team670.robot.commands.climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;

/**
 * Lower the Climber mechanism.
 */
public class RetractClimber extends CommandBase implements MustangCommand{

  private Climber climber;
  private HashMap<MustangSubsystemBase, HealthState> healthReqs;
  
  public RetractClimber(Climber climber) {
    this.climber = climber;
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(climber, HealthState.GREEN);
  }

  @Override
  public void initialize() {
    climber.stop();
    climber.climb(0); //TODO: ask mech should be 0 or 1 ?
  }


  @Override
  public boolean isFinished() {
    return climber.isAtTarget() && climber.reverseLimitSwitchTripped();
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
}
