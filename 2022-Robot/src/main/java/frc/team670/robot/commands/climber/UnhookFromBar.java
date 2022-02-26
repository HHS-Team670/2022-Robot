package frc.team670.robot.commands.climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;

/**
 * Moves the climber slowly down until it hooks on the generator bar. 
 */
public class UnhookFromBar extends CommandBase implements MustangCommand {

  private Climber climber;
  private HashMap<MustangSubsystemBase, HealthState> healthReqs;

  public UnhookFromBar(Climber climber) {
    this.climber = climber;
    addRequirements(climber);

    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(climber, HealthState.GREEN);
  }

  @Override
  public void initialize() {
    climber.unhookFromBar();
  }

  @Override
  public boolean isFinished() {
    return !climber.isHookedOnBar();
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // TODO Auto-generated method stub
    return healthReqs;
  }
}
