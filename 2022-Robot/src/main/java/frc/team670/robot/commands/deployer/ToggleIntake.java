package frc.team670.robot.commands.deployer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Deployer;

/**
 * Starts ramping up the shooter and runs it
 * 
 * @author lakshbhambhani
 */
public class ToggleIntake extends CommandBase implements MustangCommand {

    private Deployer deployer;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    /**
     * 
     * @param deployer The deployer object
     */
    public ToggleIntake(Deployer deployer) {
        this.deployer = deployer;
        addRequirements(deployer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(deployer, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        deployer.deploy(!deployer.isDeployed());
    }

    @Override
    public boolean isFinished() {
        return deployer.hasReachedTargetPosition();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}