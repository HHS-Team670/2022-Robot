package frc.team670.robot.commands.climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;

/**
 * Once the driver aligns with the mid bar, climbs to the mid bar. It then climbs
 to the high bar from the mid bar, letting go of the mid bar.
 */
public class FullClimb extends SequentialCommandGroup implements MustangCommand {
  
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public FullClimb(Climber verticalClimber, Climber diagonalClimber) {
    addRequirements(verticalClimber, diagonalClimber);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(verticalClimber, HealthState.GREEN);
    healthReqs.put(diagonalClimber, HealthState.GREEN);

    addCommands(
      new ExtendClimber(verticalClimber, Climber.Level.MID),
      new HookOnBar(verticalClimber), // TODO do we need? find out
      new RetractClimber(verticalClimber, false), // TODO find actual retraction amount
      new ExtendClimber(diagonalClimber, Climber.Level.HIGH),
      new HookOnBar(diagonalClimber),
      new RetractClimber(diagonalClimber, false));
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
  
}