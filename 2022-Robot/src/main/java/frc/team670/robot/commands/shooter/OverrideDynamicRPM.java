package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Shooter;

/**
 * 
 * @author lakshbhambhani
 */
public class OverrideDynamicRPM extends CommandBase implements MustangCommand {

    private Shooter shooter;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    /**
     * @param operatorController the operator controller object
     */
    public OverrideDynamicRPM(Shooter shooter) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.useDynamicSpeed(!shooter.isUsingDynamicSpeed());
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}