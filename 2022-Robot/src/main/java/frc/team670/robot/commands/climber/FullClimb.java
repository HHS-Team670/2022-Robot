package frc.team670.robot.commands.climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ClimberSystem;

/**
 * Once the driver aligns the back wheels to the bars under the hangar 
 * and drives straight to align with the climbing bar, this climbs on to the high bar.
 */
public class FullClimb extends SequentialCommandGroup implements MustangCommand {
  
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public FullClimb(ClimberSystem climberSystem) {
    addRequirements(climberSystem);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(climberSystem, HealthState.GREEN);
    
    addCommands(
      new ParallelCommandGroup( // extend vertical climbers
            new ExtendClimber(climberSystem.getVerticalClimbers().get(0)),
            new ExtendClimber(climberSystem.getVerticalClimbers().get(1))
      ),
      new ParallelCommandGroup( // hook vertical climbers
            new HookOnBar(climberSystem.getVerticalClimbers().get(0)),
            new HookOnBar(climberSystem.getVerticalClimbers().get(1))
      ),
      new ParallelCommandGroup( // retract vertical climbers
            new RetractClimber(climberSystem.getVerticalClimbers().get(0)),
            new RetractClimber(climberSystem.getVerticalClimbers().get(1))
      ),
      new ParallelCommandGroup( // extend diagonal climbers
            new ExtendClimber(climberSystem.getDiagonalClimbers().get(0)),
            new ExtendClimber(climberSystem.getDiagonalClimbers().get(1))
      ),
      new ParallelCommandGroup( // hook diagoanl climbers
            new HookOnBar(climberSystem.getDiagonalClimbers().get(0)),
            new HookOnBar(climberSystem.getDiagonalClimbers().get(1))
      ),
      new ParallelCommandGroup( // retract diagonal climbers; by geometry, vertical climbers unhook
            new RetractClimber(climberSystem.getDiagonalClimbers().get(0)),
            new RetractClimber(climberSystem.getDiagonalClimbers().get(1))
      ));
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
  
}