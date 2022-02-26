package frc.team670.robot.commands.climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;

/**
 * Raise the climber mechanism to its maximum allowed height so it can reach the
 * generator bar.
 */
public class ExtendClimber extends CommandBase implements MustangCommand {

  private Climber climber;
  // private boolean vertical;
  private HashMap<MustangSubsystemBase, HealthState> healthReqs;

  
  public ExtendClimber(Climber climber) {
    this.climber = climber;
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(climber, HealthState.GREEN);
  }

  @Override
  public void initialize() {
    climber.climbToMaxHeight();
  }

  @Override
  public boolean isFinished() {
    return climber.isAtTarget();
  }

  public void end() {
    climber.stop();
  }


  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // TODO Auto-generated method stub
    return healthReqs;
  }
}
