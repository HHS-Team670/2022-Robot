package frc.team670.robot.commands.climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
// import frc.team670.mustanglib.commands.InstantCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.subsystems.ClimberSystem;
import frc.team670.robot.subsystems.ClimberSystem.Climber;
import frc.team670.robot.subsystems.Deployer;

/**
 * FullClimb is constantly being run (isFinished() always returns false).
 * When the D-pad is pressed on the controller, the climb advances by one stage
 */
public class StepClimber extends CommandBase implements MustangCommand {

  private Map<MustangSubsystemBase, HealthState> healthReqs;

  private MustangController controller;
  private ClimberSystem climbers;

  private int currentStep = 0;
  private boolean justAdvanced = false;

  public StepClimber(ClimberSystem climbers, Deployer deployer) {
    Climber verticalClimber = climbers.getVerticalClimber();
    Climber diagonalClimber = climbers.getDiagonalClimber();
    
    this.climbers = climbers;
    
    addRequirements(verticalClimber, diagonalClimber, climbers);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(verticalClimber, HealthState.GREEN);
    healthReqs.put(diagonalClimber, HealthState.GREEN);
    healthReqs.put(climbers, HealthState.GREEN);
  }

  @Override
  public void execute() {
    climbers.stepClimber();
  }

@Override
public boolean isFinished(){
    return true;
}


  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }

}