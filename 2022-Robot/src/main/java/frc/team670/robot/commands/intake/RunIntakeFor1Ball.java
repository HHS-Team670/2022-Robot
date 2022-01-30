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
 * Runs the intake for the time required to intake one ball? Needs to be checked
 * @author Santan
 */
public class RunIntakeFor1Ball extends SequentialCommandGroup implements MustangCommand {

  private Intake intake;
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  // Sets up everything

  public RunIntakeFor1Ball(Intake intake) {
    this.intake = intake;
    addRequirements(this.intake);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(this.intake, HealthState.GREEN);
    addCommands(
        new DeployIntake(this.intake),
        new RunIntake(false, this.intake), // TODO: Add conveyor
        new WaitCommand(RobotConstants.TIME_TO_COLLECT_1_BALL_S), // TODO: use Conveyor sensors when Conveyor comes
        new StopIntake(this.intake));
  }

  // Returns health state

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }

}
