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
  private boolean zeroClimber;
  private final double RETRACTING_POWER = 0.05;
  private HashMap<MustangSubsystemBase, HealthState> healthReqs;
  
  public RetractClimber(Climber climber, boolean zeroClimber) {
    this.climber = climber;
    this.zeroClimber = zeroClimber;
    addRequirements(climber);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(climber, HealthState.GREEN);
  }

  @Override
  public void initialize() {
    if (zeroClimber) {
      climber.run(-RETRACTING_POWER);
    } else {
      climber.retract(); 
    }
  }




  @Override
  public boolean isFinished() {
    if (zeroClimber) {
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
