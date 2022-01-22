package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;

/**
 * Sets the shooter's target RPM
 * @author ctychen
 */
public class SetRPMTarget extends InstantCommand implements MustangCommand {

    private Shooter shooter;
    private double target;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    /**
     * @param rpm The shooter target speed, note that this should be absolute
     */
    public SetRPMTarget(double rpm, Shooter shooter) {
        this.target = rpm;
        this.shooter = shooter;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        Logger.consoleLog("Setting shooter RPM to %s", target);
        shooter.setVelocityTarget(target);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}