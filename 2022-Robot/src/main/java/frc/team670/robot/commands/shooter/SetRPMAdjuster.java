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
 * Use this to set the "adjuster" for the shooter speed, for example when trying
 * to shoot a little bit faster or slower when needed in middle of a match
 * 
 * @author smishra467
 */
public class SetRPMAdjuster extends InstantCommand implements MustangCommand {

    private Shooter shooter;
    private double diff;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    /**
     * @param diff The amount (RPM) to change the shooter RPM-adjuster by
     */
    public SetRPMAdjuster(double diff, Shooter shooter) {
        this.diff = diff;
        this.shooter = shooter;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        // Logger.consoleLog("Changing the Shooter RPM adjuster by %s", diff);
        shooter.adjustRPMAdjuster(diff);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}