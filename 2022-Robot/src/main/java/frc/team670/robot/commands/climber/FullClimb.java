package frc.team670.robot.commands.climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ClimberSystem;

/**
 * Once the driver aligns with the mid bar, climbs to the mid bar. It then climbs
 to the high bar from the mid bar, letting go of the mid bar.
 */
public class FullClimb extends SequentialCommandGroup implements MustangCommand {
  
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public FullClimb(ClimberSystem climberSystem) {
    addRequirements(climberSystem);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(climberSystem, HealthState.GREEN);

    addCommands(
      new ExtendClimber(climberSystem.getVerticalClimber()),
      new HookOnBar(climberSystem.getVerticalClimber()),
      new RetractClimber(climberSystem.getVerticalClimber()),
      new ExtendClimber(climberSystem.getDiagonalClimber()),
      new HookOnBar(climberSystem.getDiagonalClimber()),
      new RetractClimber(climberSystem.getDiagonalClimber()));
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
  
}