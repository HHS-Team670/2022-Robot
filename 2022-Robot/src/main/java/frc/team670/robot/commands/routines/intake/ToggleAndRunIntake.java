package frc.team670.robot.commands.routines.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.deployer.ToggleIntake;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.commands.intake.StopIntake;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.Intake;


/**
 * Toggles intake out and runs or backwards, pulls it in and stops
 * @author lakshbhambhani
 */

 
public class ToggleAndRunIntake extends ParallelCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    // Sets up everything

    public ToggleAndRunIntake(Intake intake, Deployer deployer) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(deployer, HealthState.GREEN);
        if(deployer.isDeployed()){
            addCommands(
                new ToggleIntake(deployer),
                new StopIntake(intake)
            );
        }
        else{
            addCommands(
                new ToggleIntake(deployer),
                new RunIntake(intake)
            );
        }
       
    }

    // Returns health state

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
