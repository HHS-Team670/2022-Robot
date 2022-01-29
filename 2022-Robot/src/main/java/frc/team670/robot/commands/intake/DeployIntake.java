package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.*;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.subsystems.Intake;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public class DeployIntake extends CommandBase implements MustangCommand {

  private Map<MustangSubsystemBase, HealthState> healthReqs;
  private Intake intake;

  /*
   * @param isDeploy true if it is to deploy, false if it is to pick up
   * @param intake the intake 
   Intakes the ball
  */
  public DeployIntake(Intake intake) {
    this.intake = intake;
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(intake, HealthState.GREEN);
    addRequirements(intake);
  }
/*

runs the intake
*/
  public void initialize(){
    intake.deploy();
  }

  public void end(){
    intake.stopDeployer();
  }

  public boolean isFinished() {
    return intake.isDeployed();
  }


/*
returns the health state of the intake
*/

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
}