package frc.team670.robot.commands.climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ClimberSystem;

/**
 * Raise the climber mechanism to its maximum allowed height so it can reach the
 * generator bar.
 */
public class ExtendClimber extends CommandBase implements MustangCommand {

  private ClimberSystem.Climber climber;
  private ClimberSystem.Level level;
  private HashMap<MustangSubsystemBase, HealthState> healthReqs;

  
  public ExtendClimber(ClimberSystem.Climber climber, ClimberSystem.Level level) {
    this.climber = climber;
    this.level = level;
    addRequirements(climber);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(climber, HealthState.GREEN);
  }

  @Override
  public void initialize() {
    climber.climb(level);
  }

  @Override
  public boolean isFinished() {
    return climber.isAtTarget();
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
}
