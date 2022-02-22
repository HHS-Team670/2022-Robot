package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

/**
 * Starts ramping up the shooter and runs it
 * 
 * @author wilsonpotato, palldas
 */
public class StartShooter extends CommandBase implements MustangCommand {

    private Shooter shooter;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean useDynamicSpeed;

    /**
     * @param shooter         the shooter object
     * @param useDynamicSpeed if the user wants to use vision
     */
    public StartShooter(Shooter shooter, boolean useDynamicSpeed) {
        this.shooter = shooter;
        this.useDynamicSpeed = useDynamicSpeed;
        addRequirements(shooter);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    /**
     * 
     * @param shooter the shooter object
     */
    public StartShooter(Shooter shooter) {
        this(shooter, false);
    }

    @Override
    public void initialize() {
        if(useDynamicSpeed)
            shooter.setRPM();
        else{
            shooter.setTargetRPM(shooter.getDefaultRPM());
        }
        shooter.run();
    }

    @Override
    public boolean isFinished() {
        return shooter.isUpToSpeed();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}