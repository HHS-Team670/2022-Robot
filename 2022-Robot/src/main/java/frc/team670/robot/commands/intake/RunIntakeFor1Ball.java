package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Once the driver aligns the back wheels to the bars under the generator, 
 * drives straight to align with the climbing bar and extends the climber when in position.
 */
public class RunIntakeFor1Ball extends SequentialCommandGroup implements MustangCommand {
  
  private Intake intake;
  private Map<MustangSubsystemBase, HealthState> healthReqs;
/*
sets up everything
*/
  public RunIntakeFor1Ball(Intake i) {
    this.intake = i;
    addRequirements(i);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(i, HealthState.GREEN);
    addCommands(
        new DeployIntake(intake),
        new RunIntake(false, intake), // TODO: Add conveyor
        new WaitCommand(RobotConstants.TIME_TO_COLLECT_1_BALL_S), // TODO: use Conveyor sensors when Conveyor comes
        new StopIntake(intake)
      );
  }
  /*
  returns health state
  */  
  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
  
}