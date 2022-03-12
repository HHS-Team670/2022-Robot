package frc.team670.robot.commands.routines.climb;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.subsystems.ClimberSystem;
import frc.team670.robot.subsystems.ClimberSystem.Climber;

/**
 * Once the driver aligns with the mid bar, climbs to the mid bar. It then
 * climbs
 * to the high bar from the mid bar, letting go of the mid bar.
 */
public class FullClimb extends CommandBase implements MustangCommand {

  private Map<MustangSubsystemBase, HealthState> healthReqs;

  private MustangController controller;
  private ClimberSystem climbers;

  private int currentStep = 0;
  private boolean justAdvanced = false;

  public FullClimb(ClimberSystem climbers, MustangController mController) {
    Climber verticalClimber = climbers.getVerticalClimber();
    Climber diagonalClimber = climbers.getDiagonalClimber();
    addRequirements(verticalClimber, diagonalClimber, climbers);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(verticalClimber, HealthState.GREEN);
    healthReqs.put(diagonalClimber, HealthState.GREEN);
    healthReqs.put(climbers, HealthState.GREEN);
    this.controller = mController;
    this.climbers = climbers;
  }

  // Called once when the command executes
  @Override
  public void execute() {
    SmartDashboard.putNumber("climb stp", currentStep);
    Logger.consoleLog("%s", controller.getDPadState());
    if(controller.getDPadState() == MustangController.DPadState.NEUTRAL && !justAdvanced){
      if(controller.getDPadState() == MustangController.DPadState.RIGHT){
        climbers.climbProcedure(++currentStep);
        justAdvanced = true;
      } 
      else if(controller.getDPadState() == MustangController.DPadState.LEFT){
        climbers.climbProcedure(--currentStep);
        justAdvanced = true;
      }
    }
    else{
      justAdvanced = false;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }

}