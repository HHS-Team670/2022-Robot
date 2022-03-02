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
  private boolean isAuto;
  private HashMap<MustangSubsystemBase, HealthState> healthReqs;
  
  public RetractClimber(Climber climber, boolean isAuto) {
    this.climber = climber;
    this.isAuto = isAuto;
    addRequirements(climber);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(climber, HealthState.GREEN);
  }

  @Override
  public void initialize() {
    climber.retractToMinHeight();; //TODO: ask mech should be 0 or 1 ?
  }


  @Override
  public boolean isFinished() {
    if (isAuto) {
      return climber.reverseLimitSwitchTripped();
    } else {
      return climber.isAtTarget() || climber.reverseLimitSwitchTripped();
    }
  }

  public void end() {
    climber.stop();
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
}
