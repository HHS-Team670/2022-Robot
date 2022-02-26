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
public class HookOnBar extends CommandBase implements MustangCommand {

  Climber climber;
  private HashMap<MustangSubsystemBase, HealthState> healthReqs;
  

  public HookOnBar(Climber climber) {
    this.climber = climber;

    healthReqs = new HashMap<>();
    healthReqs.put(climber, HealthState.GREEN);
  }

  @Override
  public void initialize() {
    climber.stop();
    climber.hookOnBar();
  }

  @Override
  public boolean isFinished() {
    return climber.isHookedOnBar();
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // TODO Auto-generated method stub
    return healthReqs;
  }
}