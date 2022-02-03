package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.Shooter;

/**
 * Starts ramping up the shooter and runs it
 * 
 * @author wilsonpotato, palldas
 */
public class StartShooter extends CommandBase implements MustangCommand {

    private Shooter shooter;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean useVision;

    /**
     * 
     * @param useVision if the user wants to use vision
     */
    public StartShooter(Shooter shooter, boolean useVision) {
        this.shooter = shooter;
        this.useVision = useVision;
        addRequirements(shooter);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        setRPM();
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

    /**
     * If vision works, it gets the distance to target from vision,
     * predicts the RPM based off the distance,
     * and sets that as the Target RPM
     * If vision doesn't work, just sets the default RPM as the target RPM
     */

    private void setRPM() {
        // if (vision.getHealth(true) == HealthState.GREEN) {
        //     double distanceToTarget = vision.getDistanceToTargetM();
        //     shooter.setRPMForDistance(distanceToTarget);
        // } else {
            shooter.setTargetRPM(shooter.getDefaultRPM());
       // }
    }

}