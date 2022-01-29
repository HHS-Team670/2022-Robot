package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.subsystems.Intake;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;


/*
Duplicate class (similar functionality in DeployIntake and StopIntake)
*/
public class ToggleIntake extends InstantCommand implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Intake intake;

    /*
     * @param isDeploy true if it is to deploy, false if it is to pick up
     */
    public ToggleIntake(Intake intake) {
        this.intake = intake;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.GREEN);
        addRequirements(intake);
    }
/*
toggles the intake, if it's on it turns it off, if it's off it turns it on
*/
    public void execute(){
        intake.deploy(!intake.isDeployed());
    }
/*
returns the health state
*/
    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
    
}