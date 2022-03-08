package frc.team670.robot.commands.routines.climb;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.climber.ExtendClimber;
import frc.team670.robot.commands.climber.RetractClimber;
import frc.team670.robot.subsystems.ClimberSystem;
import frc.team670.robot.subsystems.ClimberSystem.Climber;

/**
 * Once the driver aligns with the mid bar, climbs to the mid bar. It then climbs
 to the high bar from the mid bar, letting go of the mid bar.
 */
public class FullClimb extends SequentialCommandGroup implements MustangCommand {
  
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public FullClimb(Climber verticalClimber, Climber diagonalClimber, MustangController mController, int nextCommandButton) {
    addRequirements(verticalClimber, diagonalClimber);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(verticalClimber, HealthState.GREEN);
    healthReqs.put(diagonalClimber, HealthState.GREEN);

    BooleanSupplier buttonPressed = () -> mController.getRawButtonPressed(nextCommandButton);

    addCommands(
      new ParallelCommandGroup( // extend vertical, and diagonal partially to save time
        new ExtendClimber(verticalClimber, ClimberSystem.Level.MID),
        new ExtendClimber(diagonalClimber, ClimberSystem.Level.INTERMEDIATE_HIGH)
      ),
      new WaitUntilCommand(buttonPressed),
      new RetractClimber(verticalClimber, false), // TODO find actual retraction amount
      new WaitUntilCommand(buttonPressed),
      new ExtendClimber(diagonalClimber, ClimberSystem.Level.HIGH), // finish extending diagonal
      new WaitUntilCommand(buttonPressed),
      new ExtendClimber(verticalClimber, ClimberSystem.Level.INTERMEDIATE_MID),
      new WaitUntilCommand(buttonPressed),
      new RetractClimber(diagonalClimber, false));
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
  
}